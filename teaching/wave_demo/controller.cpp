// Isolated motion-only demo. No factory state, recovery or gripper commands.
#include "../moving_conveyor/model.hpp"
#include "../../digital_twin/bridge/state_transport.hpp"
#include <franka/gripper.h>
#include <franka/rate_limiting.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <mutex>
#include <sys/prctl.h>

using Tree=boost::property_tree::ptree;
template<size_t N> std::array<double,N> array(const Tree&t){std::array<double,N>a{};size_t i=0;for(auto&v:t){if(i==N)throw std::runtime_error("Array size");a[i++]=v.second.get_value<double>();}if(i!=N)throw std::runtime_error("Array size");for(double v:a)if(!std::isfinite(v))throw std::runtime_error("Nonfinite value");return a;}
Pose asPose(const M4&m){Pose p{};std::copy(m.data(),m.data()+16,p.begin());return p;}
struct Segment{double t;std::array<std::array<double,6>,7>c;};
struct Path{
  std::string name;double duration;bool wave;Q q0,q1;Pose target;std::vector<Segment>segments;
  explicit Path(const Tree&t):name(t.get<std::string>("name")),duration(t.get<double>("duration_s")),wave(t.get<bool>("wave")),q0(array<7>(t.get_child("q0"))),q1(array<7>(t.get_child("q1"))),target(array<16>(t.get_child("target"))){
    std::vector<double>knots;for(auto&v:t.get_child("knots_s"))knots.push_back(v.second.get_value<double>());
    size_t i=0;for(auto&v:t.get_child("coefficients")){if(i>=knots.size())throw std::runtime_error("Spline dimensions");Segment s;s.t=knots[i++];int j=0;for(auto&c:v.second){if(j==7)throw std::runtime_error("Spline dimensions");s.c[j++]=array<6>(c.second);}if(j!=7)throw std::runtime_error("Spline dimensions");segments.push_back(s);}
    if(i!=knots.size()||i==0||i>1024||knots[0]!=0||!std::isfinite(duration)||duration<1||duration>120)throw std::runtime_error("Spline duration/size");
    for(size_t k=0;k<i;++k)if(!std::isfinite(knots[k])||knots[k]>=duration||(k&&knots[k]<=knots[k-1]))throw std::runtime_error("Spline knots");
  }
  Q eval(double t,int derivative=0)const{t=std::clamp(t,0.,duration);auto it=std::upper_bound(segments.begin(),segments.end(),t,[](double x,const Segment&s){return x<s.t;});if(it!=segments.begin())--it;double u=t-it->t;Q q{};for(int j=0;j<7;++j)for(int k=0;k<=5-derivative;++k){double f=1;for(int r=0;r<derivative;++r)f*=5-k-r;q[j]=q[j]*u+it->c[j][k]*f;}return q;}
};
struct Program{
  Model model;Pose tool,neutral;Q lo,hi;V3 lower,upper;Tree config;std::vector<Path>paths;
  double speed,acc,jerk,min_sigma,max_condition,extension,max_tilt,cart_speed,angular_speed,duration_budget,pos_tol,angle_tol,joint_tracking,tcp_tracking,gap;int cpu,worker;
  explicit Program(const Tree&t):tool(array<16>(t.get_child("tool"))),lo(array<7>(t.get_child("lower_q"))),hi(array<7>(t.get_child("upper_q"))),config(t.get_child("config")){
    if(t.get<std::string>("task")!="isolated_empty_hand_wave")throw std::runtime_error("Wrong program task");
    auto l=array<3>(config.get_child("workspace_bounds_m.min")),u=array<3>(config.get_child("workspace_bounds_m.max"));lower=V3(l.data());upper=V3(u.data());
    for(int j=0;j<3;++j)if(lower[j]>=upper[j])throw std::runtime_error("Workspace bounds");
    auto limits=config.get_child("limits");speed=limits.get<double>("joint_velocity_rad_s");acc=limits.get<double>("joint_acceleration_rad_s2");jerk=limits.get<double>("joint_jerk_rad_s3");min_sigma=limits.get<double>("min_full_sigma");max_condition=limits.get<double>("max_full_condition");extension=limits.get<double>("min_elbow_extension_margin_rad");max_tilt=config.get<double>("maximum_measured_tilt_deg")*M_PI/180;cpu=config.get<int>("controller_cpu");worker=config.get<int>("worker_cpu");
    cart_speed=limits.get<double>("translation_velocity_m_s");angular_speed=limits.get<double>("rotation_velocity_rad_s");duration_budget=t.get<double>("planned_duration_s")+8;
    pos_tol=limits.get<double>("endpoint_position_m");angle_tol=limits.get<double>("endpoint_angle_rad");joint_tracking=limits.get<double>("maximum_joint_tracking_rad");tcp_tracking=limits.get<double>("maximum_tcp_tracking_m");gap=limits.get<double>("maximum_callback_gap_s");
    if(!(pos_tol>0&&pos_tol<=.0015&&angle_tol>0&&angle_tol<=.0174533&&joint_tracking>0&&joint_tracking<=.02&&tcp_tracking>0&&tcp_tracking<=.005&&gap>0&&gap<=.02))throw std::runtime_error("Invalid wave tracking/endpoint limits");
    double opening=config.get<double>("empty_hand_minimum_opening_m");if(!(opening>=.069&&opening<=.08))throw std::runtime_error("Invalid empty-hand width");
    if(cpu<0||worker<0||cpu>=CPU_SETSIZE||worker>=CPU_SETSIZE||cpu==worker||!(cart_speed>0&&cart_speed<=.4&&angular_speed>0&&angular_speed<=.8&&duration_budget>0&&duration_budget<600))throw std::runtime_error("Invalid demo CPU/speed/duration");
    if(!(speed>0&&speed<=1&&acc>0&&acc<=2&&jerk>0&&jerk<=40&&min_sigma>=.15&&max_condition<=35&&extension>=.2&&max_tilt>0&&max_tilt<=35*M_PI/180))throw std::runtime_error("Unsafe demo limits");
    model.length=t.get<double>("characteristic_length_m");size_t i=0;for(auto&v:t.get_child("fixed_model.joint_origins")){if(i==7)throw std::runtime_error("Model size");model.origins[i++]=matrix(array<16>(v.second));}if(i!=7||model.length!=.3)throw std::runtime_error("Model size/scale");model.ee=matrix(array<16>(t.get_child("fixed_model.joint7_to_ee")));
    neutral=asPose(model.evaluate(array<7>(config.get_child("upright_q_rad"))).first);
    if(std::acos(std::clamp(neutral[8],-1.,1.))>1e-6)throw std::runtime_error("Neutral fingers must point world +X");
    for(auto&v:t.get_child("paths"))paths.emplace_back(v.second);if(paths.size()!=6)throw std::runtime_error("Expected six demo phases");
    const char*names[]={"home_5_entry","vertical_ascent_home_5","straighten_to_upright","wave_world_y","return_from_upright","vertical_descent_home_5"};
    for(size_t k=0;k<paths.size();++k){const auto&p=paths[k];
      if(p.name!=names[k]||p.wave!=(k==3))throw std::runtime_error("Invalid isolated demo phase order");
      for(int j=0;j<7;++j)if(std::abs(p.eval(0)[j]-p.q0[j])>1e-7||std::abs(p.eval(p.duration)[j]-p.q1[j])>1e-7||(k&&std::abs(paths[k-1].q1[j]-p.q0[j])>1e-7))throw std::runtime_error("Disconnected trajectory");
      for(double s:{0.,p.duration})for(int d:{1,2})for(double v:p.eval(s,d))if(std::abs(v)>1e-5)throw std::runtime_error("Nonstationary phase boundary");
      for(int n=0;n<=2000;++n){double time=p.duration*n/2000;auto q=p.eval(time);guard(q);auto fk=model.evaluate(q).first;workspace(fk.block<3,1>(0,3));
        if(k==0){auto h=array<16>(config.get_child("home_5_O_T_EE"));double half=config.get<double>("home_gate_half_width_m");V3 d=fk.block<3,1>(0,3)-xyz(h);if(d.head<2>().cwiseAbs().maxCoeff()>half||d[2]<-half)throw std::runtime_error("Entry leaves vertical home5 column");}
        if(p.wave&&(angle(asPose(fk),neutral)>max_tilt||std::acos(std::clamp(fk(0,2),-1.,1.))>1e-6))throw std::runtime_error("Planned wave tilt or +X finger direction");
        for(int d=1;d<=3;++d)for(double v:p.eval(time,d))if(std::abs(v)>(d==1?speed*.85:d==2?acc:jerk)*1.001)throw std::runtime_error("Planned derivative limit");
      }
    }
  }
  void workspace(const V3&p)const{if(!p.allFinite()||(p.array()<lower.array()).any()||(p.array()>upper.array()).any())throw std::runtime_error("Demo workspace");}
  void guard(const Q&q)const{
    for(int j=0;j<7;++j)if(!std::isfinite(q[j])||q[j]<lo[j]||q[j]>hi[j])throw std::runtime_error("Demo joint bound");
    auto j=model.evaluate(q).second;Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>>e(j*j.transpose(),Eigen::EigenvaluesOnly);double s=std::sqrt(std::max(0.,e.eigenvalues()[0]));
    if(e.info()!=Eigen::Success||s<min_sigma||std::sqrt(e.eigenvalues()[5])/s>max_condition||std::abs(q[3]+.467002423653011)<extension)throw std::runtime_error("Full seven-joint singularity guard");
  }
};
void ordinary(int cpu){sched_param p{};if(pthread_setschedparam(pthread_self(),SCHED_OTHER,&p))throw std::runtime_error("Worker priority");cpu_set_t mask;CPU_ZERO(&mask);CPU_SET(cpu,&mask);if(pthread_setaffinity_np(pthread_self(),sizeof(mask),&mask))throw std::runtime_error("Worker affinity");}
int main(int argc,char**argv){
  if(argc!=3||(std::string(argv[2])!="--check"&&std::string(argv[2])!="--execute"))return 2;
  std::signal(SIGINT,stop);std::signal(SIGTERM,stop);std::signal(SIGPIPE,SIG_IGN);std::cout<<std::setprecision(17);std::mutex output;
  try{
    Tree tree;boost::property_tree::read_json(argv[1],tree);Program p(tree);
    if(std::string(argv[2])=="--check"){std::cout<<"MODEL_CHECK_PASS no SDK connection\n";return 0;}
    if(prctl(PR_SET_PDEATHSIG,SIGTERM)||getppid()==1)throw std::runtime_error("Demo parent unavailable");
    cpu_set_t cpus;sched_getaffinity(0,sizeof(cpus),&cpus);if(!CPU_ISSET(p.cpu,&cpus)||!CPU_ISSET(p.worker,&cpus))throw std::runtime_error("Demo CPUs unavailable");CPU_ZERO(&cpus);CPU_SET(p.cpu,&cpus);if(sched_setaffinity(0,sizeof(cpus),&cpus))throw std::runtime_error("Controller affinity");
    twin::SdkLease sdk(true);franka::Robot robot("172.16.0.2",franka::RealtimeConfig::kEnforce);
    std::ifstream frozen(std::filesystem::path(argv[1]).parent_path()/"robot_model.urdf");std::string expected_model{std::istreambuf_iterator<char>(frozen),std::istreambuf_iterator<char>()};if(expected_model.empty()||robot.getRobotModel()!=expected_model)throw std::runtime_error("Robot model changed since wave planning");
    twin::HandLease hand_lease;franka::Gripper hand("172.16.0.2");auto initial_hand=hand.readOnce();
    if(!std::isfinite(initial_hand.width)||initial_hand.width<p.config.get<double>("empty_hand_minimum_opening_m"))throw std::runtime_error("Empty-hand wave requires an already open gripper; no gripper command was sent");
    std::atomic<bool>done{false},request_stop{false},stop_called{false},hand_failed{false},stop_failed{false};std::atomic<int>phase{-1};std::atomic<double>phase_time{0};std::atomic<bool>worker_ready{false},worker_ok{false};
    twin::StateTap tap([&](const twin::StateTap::Item&s){std::lock_guard<std::mutex>lock(output);std::cout<<"{\"type\":\"state\",\"unix_ns\":"<<s.unix_ns<<",\"phase\":"<<s.phase<<",\"phase_time_s\":"<<s.phase_time<<",\"state\":"<<s.state<<"}\n"<<std::flush;},p.worker);
    auto initial=robot.readOnce();tap.offer(initial);if(initial.current_errors||initial.robot_mode!=franka::RobotMode::kIdle)throw std::runtime_error("Wave requires fault-free Idle; no automatic recovery");p.guard(initial.q);
    for(int j=0;j<7;++j)if(std::abs(initial.q[j]-p.paths[0].q0[j])>.001||std::abs(initial.dq[j])>.01)throw std::runtime_error("Robot changed since wave preflight");
    for(int j=0;j<16;++j)if(std::abs(initial.F_T_EE[j]-p.tool[j])>1e-6)throw std::runtime_error("Wave tool frame changed");
    if((p.model.evaluate(initial.q).first.block<3,1>(0,3)-xyz(initial.O_T_EE)).norm()>.0001)throw std::runtime_error("Wave FK model mismatch");
    std::thread hand_reader,stopper;
    bool completed=false;std::string failure;double elapsed=0,settled=0,hold=0,total=0,stop_stable=0;size_t index=0;Q raw=initial.q_d,sent=initial.q_d,velocity{},acceleration{},offset{};bool first=true;
    try{
      hand_reader=std::thread([&]{try{ordinary(p.worker);while(!done.load()){auto s=hand.readOnce();tap.updateHand(s.width,std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());if(!std::isfinite(s.width)||s.width<p.config.get<double>("empty_hand_minimum_opening_m")){hand_failed.store(true);request_stop.store(true);}std::this_thread::sleep_for(std::chrono::milliseconds(50));}}catch(...){hand_failed.store(true);request_stop.store(true);}});
      stopper=std::thread([&]{try{ordinary(p.worker);worker_ok.store(true);worker_ready.store(true);while(!done.load()){if((request_stop.load()||interrupted)&&!stop_called.exchange(true))try{robot.stop();}catch(...){stop_failed.store(true);}std::this_thread::sleep_for(std::chrono::milliseconds(1));}}catch(...){worker_ready.store(true);stop_failed.store(true);}});
      while(!worker_ready.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
      if(!worker_ok.load())throw std::runtime_error("Stop worker unavailable");
      robot.control([&](const franka::RobotState&s,franka::Duration period)->franka::JointPositions{
        const double dt=period.toSec();total+=dt;const auto&path=p.paths[index];
        try{
          if(s.current_errors||interrupted||hand_failed.load()||stop_failed.load()||dt>p.gap+1e-6||total>p.duration_budget)throw std::runtime_error("Wave fault, timing or stop request");
          p.guard(s.q);p.guard(s.q_d);p.workspace(xyz(s.O_T_EE));
          for(int j=0;j<7;++j)if(std::abs(s.dq[j])>p.speed+.01||std::abs(s.q[j]-s.q_d[j])>p.joint_tracking)throw std::runtime_error("Wave measured joint speed/tracking");
          for(int j=0;j<16;++j)if(std::abs(s.F_T_EE[j]-p.tool[j])>1e-6)throw std::runtime_error("Wave live tool frame changed");
          if(first){sent=s.q_d;raw=sent;velocity=s.dq_d;acceleration=s.ddq_d;for(int j=0;j<7;++j){offset[j]=sent[j]-path.q0[j];if(std::abs(offset[j])>.001)throw std::runtime_error("FCI startup reference changed too far");}first=false;return franka::JointPositions(sent);}
          if(!request_stop.load()){
            elapsed+=dt;raw=path.eval(elapsed);if(index==0)for(int j=0;j<7;++j)raw[j]+=offset[j]*(1-blend(elapsed/path.duration));
            auto expected=p.model.evaluate(raw).first;
            Pose expected_pose{};std::copy(expected.data(),expected.data()+16,expected_pose.begin());
            if((xyz(s.O_T_EE)-expected.block<3,1>(0,3)).norm()>p.tcp_tracking||angle(s.O_T_EE,expected_pose)>p.angle_tol)throw std::runtime_error("Wave TCP/orientation tracking corridor");
            auto jac=p.model.evaluate(s.q).second;Eigen::Matrix<double,7,1>dq=Eigen::Map<const Eigen::Matrix<double,7,1>>(s.dq.data());auto cart=jac*dq;
            if(cart.head<3>().norm()*p.model.length>p.cart_speed+.01||cart.tail<3>().norm()>p.angular_speed+.02)throw std::runtime_error("Wave measured Cartesian speed");
            if(path.wave&&(angle(s.O_T_EE,p.neutral)>p.max_tilt||std::acos(std::clamp(s.O_T_EE[8],-1.,1.))>p.angle_tol))throw std::runtime_error("Measured wave tilt or +X finger direction");
            if(elapsed>=path.duration){double v=0;for(double d:s.dq)v=std::max(v,std::abs(d));bool settled_now=(xyz(s.O_T_EE)-xyz(path.target)).norm()<=p.pos_tol&&angle(s.O_T_EE,path.target)<=p.angle_tol&&v<.01;settled=settled_now?settled+dt:0;hold+=dt;if(hold>.6)throw std::runtime_error("Wave phase endpoint did not settle");if(settled>=.03){if(index+1==p.paths.size())completed=true;else{++index;elapsed=settled=hold=0;}}}
          }
        }catch(const std::exception&e){if(failure.empty())failure=e.what();request_stop.store(true);raw=sent;}
        auto upper=robot.getUpperJointVelocityLimits(s.q_d),lower=robot.getLowerJointVelocityLimits(s.q_d);for(int j=0;j<7;++j){upper[j]=std::min(upper[j],p.speed);lower[j]=std::max(lower[j],-p.speed);}
        if(dt>.0015){sent=s.q_d;velocity=s.dq_d;acceleration=s.ddq_d;}
        auto next=franka::limitRate(upper,lower,franka::kMaxJointAcceleration,franka::kMaxJointJerk,raw,sent,velocity,acceleration);Q v{},a{};for(int j=0;j<7;++j){v[j]=(next[j]-sent[j])/.001;a[j]=(v[j]-velocity[j])/.001;}sent=next;velocity=v;acceleration=a;
        phase.store(int(index));phase_time.store(elapsed);tap.offer(s,&raw,&sent,int(index),elapsed);franka::JointPositions result(sent);if(completed&&!request_stop.load())return franka::MotionFinished(result);
        bool stationary=true;for(int j=0;j<7;++j)stationary=stationary&&std::abs(s.dq[j])<.01&&std::abs(s.dq_d[j])<.01;stop_stable=request_stop.load()&&stationary?stop_stable+dt:0;if(stop_stable>=.05)return franka::MotionFinished(result);return result;
      },franka::ControllerMode::kJointImpedance,false,franka::kMaxCutoffFrequency);
    }catch(const std::exception&e){if(failure.empty())failure=e.what();}
    done.store(true);if(stopper.joinable())stopper.join();if(hand_reader.joinable())hand_reader.join();
    if(!completed||!failure.empty())throw std::runtime_error(failure.empty()?"Wave stopped before completion":failure);
    auto final=robot.readOnce();tap.offer(final);if(final.current_errors||final.robot_mode!=franka::RobotMode::kIdle||(xyz(final.O_T_EE)-xyz(p.paths.back().target)).norm()>p.pos_tol||angle(final.O_T_EE,p.paths.back().target)>p.angle_tol)throw std::runtime_error("Final home verification failed");for(double v:final.dq)if(std::abs(v)>.01)throw std::runtime_error("Final home is not stationary");
    std::cout<<"WAVE_COMPLETE returned stationary to home5\n";return 0;
  }catch(const std::exception&e){std::cerr<<"RECOVERABLE_STOP "<<e.what()<<'\n';return 1;}
}
