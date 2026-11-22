// Persistent SDK owner. No parsing, allocation, files or gripper I/O in callback.
#include "../moving_conveyor/model.hpp"
#include "../../digital_twin/bridge/state_transport.hpp"
#include <franka/gripper.h>
#include <franka/rate_limiting.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <unordered_map>

using Tree=boost::property_tree::ptree;
template<size_t N>std::array<double,N> array(const Tree&t){std::array<double,N>a{};size_t i=0;for(auto&v:t){if(i>=N)throw std::runtime_error("Array length");a[i++]=v.second.get_value<double>();}if(i!=N)throw std::runtime_error("Array length");for(double v:a)if(!std::isfinite(v))throw std::runtime_error("Nonfinite input");return a;}
template<class A>void jsonArray(std::ostream&o,const A&a){o<<'[';bool first=true;for(auto v:a){if(!first)o<<',';first=false;o<<v;}o<<']';}
int64_t unixNow(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();}
double now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
std::mutex output_mutex;
void event(const std::string&type,int id=0,const std::string&detail=""){
  std::lock_guard<std::mutex>lock(output_mutex);std::cout<<"{\"type\":\""<<type<<"\",\"id\":"<<id<<",\"unix_ns\":"<<unixNow()<<",\"detail\":\"";
  for(char c:detail)std::cout<<(c=='"'||c=='\\'?' ':c=='\n'?' ':c);std::cout<<"\"}\n"<<std::flush;
}
struct Segment{double t;std::array<std::array<double,6>,7>c;};
struct Path{
  int id=0;double duration=0,release_at=-1,release_deadline=-1;bool tracking=false,servo=false;
  Q q0{},q1{},max_v{},max_a{},max_j{};Pose target{};V3 tracking_origin;std::vector<Segment>segments;
  Q eval(double t,int d=0)const{
    t=std::clamp(t,0.,duration);auto it=std::upper_bound(segments.begin(),segments.end(),t,[](double x,const Segment&s){return x<s.t;});if(it!=segments.begin())--it;
    Q q{};double u=t-it->t;for(int j=0;j<7;++j){const auto&c=it->c[j];double value=0;for(int k=0;k<=5-d;++k){double factor=1;for(int r=0;r<d;++r)factor*=5-k-r;value=value*u+c[k]*factor;}q[j]=value;}return q;
  }
};
struct Config{
  bool enforce_release_speed=true;
#ifdef LAYER_STACKING_CONTROLLER
  double contact_force_limit=0,contact_lateral_limit=0,contact_torque_limit=0,cycle_timeout=60;
#else
  const double cycle_timeout=20;
#endif
  Model model;Pose tool{};Q lo{},hi{};V3 lower,upper;double speed=1,acc=2,jerk=40,full=.15,fixed=.12,condition=35,endpoint=.001,angle_tol=.0174533;
  double endpoint_stable=.03,endpoint_timeout=.2,heartbeat=.6,open=.07,close=.06,hand_speed=.02,open_speed=.08,contact_upper=.0655,cart_speed=.4,belt_speed=0,release_lower=.012,release_upper=.02;int cpu=6,worker_cpu=2;
  explicit Config(const Tree&t){tool=array<16>(t.get_child("tool"));lo=array<7>(t.get_child("lower_q"));hi=array<7>(t.get_child("upper_q"));auto l=array<3>(t.get_child("lower_xyz")),u=array<3>(t.get_child("upper_xyz"));lower=V3(l.data());upper=V3(u.data());
    auto f=t.get_child("fixed_model");int k=0;for(auto&v:f.get_child("joint_origins")){auto a=array<16>(v.second);model.origins[k++]=matrix(a);}if(k!=7)throw std::runtime_error("Seven joint origins required");model.ee=matrix(array<16>(f.get_child("joint7_to_ee")));model.length=t.get<double>("characteristic_length");
    full=t.get<double>("min_full");fixed=t.get<double>("min_fixed");condition=t.get<double>("max_condition");auto s=t.get_child("settings");cpu=s.get<int>("controller_cpu");worker_cpu=s.get<int>("worker_cpu");speed=s.get<double>("joint_velocity_rad_s");acc=s.get<double>("joint_acceleration_rad_s2");jerk=s.get<double>("joint_jerk_rad_s3");endpoint=s.get<double>("endpoint_position_m");angle_tol=s.get<double>("endpoint_angle_rad");endpoint_stable=s.get<double>("endpoint_stable_s");endpoint_timeout=s.get<double>("endpoint_timeout_s");heartbeat=s.get<double>("heartbeat_timeout_s");
    auto g=t.get_child("gripper");open=g.get<double>("approach_opening_m");close=g.get<double>("close_target_m");hand_speed=g.get<double>("position_speed_m_s");contact_upper=g.get<double>("close_contact_upper_m");
    belt_speed=t.get<double>("conveyor_speed_m_s");
    release_lower=t.get<double>("minimum_release_vy_m_s",belt_speed*.8);release_upper=t.get<double>("maximum_release_vy_m_s",std::min(.02,belt_speed*1.2));
#ifdef FACTORY_MOVING_CONVEYOR_20MM
    if(std::abs(belt_speed-.02)>1e-9)throw std::runtime_error("Moving physical conveyor requires 0.02 m/s");
    enforce_release_speed=!t.get<bool>("execution_profile.task_execution",false);
    const double release_cap=.02500001;
#else
    const double release_cap=.02000001;
#endif
    if(!std::isfinite(release_lower)||!std::isfinite(release_upper)||release_lower<=0||release_lower>belt_speed||release_upper<belt_speed||release_upper>release_cap)throw std::runtime_error("Invalid measured release speed bounds");
    open_speed=t.get<double>("release_speed_m_s",.08);
    if(!std::isfinite(open_speed)||open_speed<=0||open_speed>.08)throw std::runtime_error("Invalid opening speed");
    if(!std::isfinite(belt_speed)||belt_speed<=0||belt_speed>.12)throw std::runtime_error("Invalid world +Y conveyor speed");
    if(speed<=0||speed>1||acc<=0||acc>2.00001||jerk<=0||jerk>40.0001||endpoint<=0||endpoint>.0015||full<.1||fixed<.08||condition>50||open!=.07||close!=.06)throw std::runtime_error("Invalid controller limits");
#ifdef LAYER_STACKING_CONTROLLER
    contact_force_limit=t.get<double>("layer_contact.maximum_force_N");contact_lateral_limit=t.get<double>("layer_contact.maximum_lateral_force_N");contact_torque_limit=t.get<double>("layer_contact.maximum_torque_Nm");
    if(!std::isfinite(contact_force_limit)||!std::isfinite(contact_lateral_limit)||!std::isfinite(contact_torque_limit)||contact_force_limit<=0||contact_lateral_limit<=0||contact_torque_limit<=0)throw std::runtime_error("Uncommissioned layer contact limits");
#endif
  }
  Metrics guard(const Q&q)const{for(int j=0;j<7;++j)if(!std::isfinite(q[j])||q[j]<lo[j]||q[j]>hi[j])throw std::runtime_error("Joint limit");auto m=model.metrics(q);if(m.full<full||m.fixed<fixed||m.condition>condition||m.extension<.15)throw std::runtime_error("Singularity guard");return m;}
};
void ordinary(int cpu){sched_param p{};if(pthread_setschedparam(pthread_self(),SCHED_OTHER,&p))throw std::runtime_error("Cannot demote worker");cpu_set_t mask;CPU_ZERO(&mask);CPU_SET(cpu,&mask);if(pthread_setaffinity_np(pthread_self(),sizeof(mask),&mask))throw std::runtime_error("Cannot pin worker");}
struct Online{
  double duration=0;std::array<std::array<double,6>,7>c{};
  Q eval(double t,int d=0)const{t=std::clamp(t,0.,duration);Q q{};for(int j=0;j<7;++j){double value=0;for(int k=5;k>=d;--k){double factor=1;for(int r=0;r<d;++r)factor*=k-r;value=value*t+c[j][k]*factor;}q[j]=value;}return q;}
  void fit(const Q&q,const Q&v,const Q&a,const Q&target,double seconds,const Config&cfg){
    duration=seconds;
    for(int attempt=0;attempt<10;++attempt){
      bool valid=true;double T=duration;
      for(int j=0;j<7;++j){double x=target[j]-q[j];c[j]={q[j],v[j],a[j]/2,10*x/std::pow(T,3)-6*v[j]/(T*T)-1.5*a[j]/T,-15*x/std::pow(T,4)+8*v[j]/std::pow(T,3)+1.5*a[j]/(T*T),6*x/std::pow(T,5)-3*v[j]/std::pow(T,4)-.5*a[j]/std::pow(T,3)};
        for(int d=0;d<=3;++d){const int degree=5-d;std::array<double,6>power{},bern{};for(int k=d;k<=5;++k){double factor=1;for(int r=0;r<d;++r)factor*=k-r;power[k-d]=c[j][k]*factor*std::pow(T,k-d);}for(int k=0;k<=degree;++k)for(int r=0;r<=k;++r){double ratio=1;for(int z=0;z<r;++z)ratio*=double(k-z)/(degree-z);bern[k]+=power[r]*ratio;}for(int k=0;k<=degree;++k){double z=bern[k];if(d==0)valid=valid&&z>=cfg.lo[j]&&z<=cfg.hi[j];else valid=valid&&std::abs(z)<=(d==1?cfg.speed*.85:d==2?cfg.acc:cfg.jerk);}}
      }
      if(valid)return;duration*=1.2;if(duration>.8)break;
    }throw std::runtime_error("Online correction cannot meet derivative bounds");
  }
};
int main(int argc,char**argv){
#ifdef FACTORY_MOVING_CONVEYOR_20MM
  const bool validate_only=argc==3&&std::string(argv[2])=="--validate-only";
  if(argc!=2&&!validate_only)return 2;
#else
  if(argc!=2)return 2;
#endif
  std::signal(SIGINT,stop);std::signal(SIGTERM,stop);std::signal(SIGPIPE,SIG_IGN);std::cout<<std::setprecision(17);
  try{
    Tree tree;boost::property_tree::read_json(argv[1],tree);
    if(!tree.get<bool>("commissioning_authorized",false))throw std::runtime_error("Experimental persistent controller is not commissioned; no robot connection");
    Config cfg(tree);
#ifdef FACTORY_MOVING_CONVEYOR_20MM
    // Validate the new variant's real parser without acquiring SDK ownership.
    if(validate_only){event("configuration_validated",0,cfg.enforce_release_speed?"No SDK connection; gripper release speed guard enforced":"No SDK connection; gripper release speed guard warning-only");return 0;}
#endif
    cpu_set_t allowed;sched_getaffinity(0,sizeof(allowed),&allowed);if(!CPU_ISSET(cfg.cpu,&allowed)||!CPU_ISSET(cfg.worker_cpu,&allowed))throw std::runtime_error("Configured CPUs unavailable");
    cpu_set_t control_cpu;CPU_ZERO(&control_cpu);CPU_SET(cfg.cpu,&control_cpu);if(sched_setaffinity(0,sizeof(control_cpu),&control_cpu))throw std::runtime_error("Cannot pin controller");
    twin::SdkLease sdk(true);franka::Robot robot("172.16.0.2",franka::RealtimeConfig::kEnforce);
    twin::HandLease hand_lease;franka::Gripper hand("172.16.0.2");
    std::atomic<bool>shutdown{false},begin{false},control{false},finish{false},halt{false},faulted{false};
#ifdef LAYER_STACKING_CONTROLLER
    std::atomic<bool>contact_guard_enabled{false};std::array<std::atomic<double>,6>contact_bias{};
#endif
    std::atomic<bool>request_stop{false},stop_called{false},stop_worker_end{false},stop_worker_ready{false},stop_worker_ok{false},stop_failed{false};
    std::atomic<int>last_done{0},active_id{0},hand_request{0},hand_done{0},hand_kind{0};
    std::atomic<double>last_ping{now()},hand_width{0},hand_time{0},hand_latency{0},phase_time{0};
    std::atomic<bool>hand_ok{false};std::atomic<Path*>pending{nullptr};std::unordered_map<int,std::unique_ptr<Path>>paths;
    std::mutex latest_mutex;franka::RobotState latest{};
    twin::StateTap tap([&](const twin::StateTap::Item&sample){
      {std::lock_guard<std::mutex>lock(latest_mutex);latest=sample.state;}
      std::lock_guard<std::mutex>lock(output_mutex);std::cout<<"{\"type\":\"state\",\"unix_ns\":"<<sample.unix_ns<<",\"state\":"<<sample.state<<",\"requested\":";jsonArray(std::cout,sample.requested);std::cout<<",\"sent\":";jsonArray(std::cout,sample.sent);
      std::cout<<",\"phase\":"<<sample.phase<<",\"phase_time_s\":"<<sample.phase_time<<",\"control\":"<<(control.load()?"true":"false")<<",\"last_done\":"<<last_done.load()<<",\"hand_width\":"<<hand_width.load()<<",\"hand_stamp\":"<<hand_time.load()<<",\"hand_done\":"<<hand_done.load()<<",\"hand_ok\":"<<(hand_ok.load()?"true":"false")<<",\"hand_latency_s\":"<<hand_latency.load()<<"}\n"<<std::flush;
    },cfg.worker_cpu);
    std::atomic<bool>hand_ready{false},hand_worker_ok{false};
    std::thread hand_worker([&]{try{ordinary(cfg.worker_cpu);hand_worker_ok.store(true);hand_ready.store(true);while(!shutdown.load()){
      const int request=hand_request.load();
      if(request!=hand_done.load()){
        if(faulted.load()){hand_ok.store(false);hand_done.store(request);continue;}
        bool ok=false;double start=now();std::string detail;
        try{
          const int kind=hand_kind.load();
          const bool moved=hand.move(kind==1?cfg.close:cfg.open,kind==1?cfg.hand_speed:cfg.open_speed);
          if(!moved)throw std::runtime_error("Hand position command returned failure");
          const double deadline=now()+.25;double stable_since=0;uint64_t previous_packet=0;
          do{
            const auto s=hand.readOnce();const double stamp=now();
            hand_width.store(s.width);hand_time.store(stamp);tap.updateHand(s.width,unixNow());
            const bool inside=std::isfinite(s.width)&&(kind==1?s.width>=cfg.close-.001&&s.width<=cfg.contact_upper:std::abs(s.width-cfg.open)<=.001);
            {std::lock_guard<std::mutex>lock(output_mutex);std::cout<<"{\"type\":\"hand_sample\",\"id\":"<<request<<",\"width_m\":"<<s.width<<",\"sdk_time_ms\":"<<s.time.toMSec()<<",\"elapsed_s\":"<<stamp-start<<"}\n"<<std::flush;}
            if(inside){if(stable_since==0)stable_since=stamp;ok=stamp-stable_since>=.02&&s.time.toMSec()>previous_packet;}else stable_since=0;
            previous_packet=s.time.toMSec();if(ok)break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
          }while(now()<deadline&&!shutdown.load()&&!faulted.load());
          if(!ok)detail="Hand did not settle inside unchanged width tolerance";
        }catch(const std::exception&e){ok=false;detail=e.what();}
        hand_latency.store(now()-start);hand_ok.store(ok);hand_done.store(request);event(ok?"hand_done":"hand_failed",request,detail);
      }else{try{auto s=hand.readOnce();hand_width.store(s.width);hand_time.store(now());tap.updateHand(s.width,unixNow());}catch(...){hand_time.store(0);}std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    }}catch(...){hand_ready.store(true);}});
    while(!hand_ready.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if(!hand_worker_ok.load()){shutdown.store(true);hand_worker.join();throw std::runtime_error("Hand worker initialization");}
    // SDK stop is explicitly supported while control runs in another thread.
    // The controller's own stop generator handles deceleration. Never chase a
    // frozen position with a low-jerk limiter as an emergency brake.
    std::thread stop_worker([&]{
      try{ordinary(cfg.worker_cpu);stop_worker_ok.store(true);stop_worker_ready.store(true);
        while(!stop_worker_end.load()){
          if(control.load()&&(request_stop.load()||halt.load()||shutdown.load()||interrupted||now()-last_ping.load()>cfg.heartbeat)){
            if(!stop_called.exchange(true)){
              request_stop.store(true);
              try{robot.stop();event("sdk_stop_sent");}
              catch(const std::exception&e){stop_failed.store(true);event("sdk_stop_failed",0,e.what());}
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
      }catch(...){stop_worker_ready.store(true);}
    });
    while(!stop_worker_ready.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if(!stop_worker_ok.load()){shutdown.store(true);stop_worker_end.store(true);stop_worker.join();hand_worker.join();throw std::runtime_error("Stop worker initialization");}
    std::thread input([&]{try{ordinary(cfg.worker_cpu);std::string line;while(std::getline(std::cin,line)){
      if(line.size()>2000000)throw std::runtime_error("Command too large");Tree t;std::istringstream stream(line);boost::property_tree::read_json(stream,t);const auto op=t.get<std::string>("op");const int id=t.get<int>("id",0);last_ping.store(now());
      try{
        if(op=="ping")continue;
#ifdef LAYER_STACKING_CONTROLLER
        if(op=="contact_guard"){
          if(active_id.load()!=0||pending.load()||faulted.load())throw std::runtime_error("Contact guard changes require stationary control");
          const bool enable=t.get<bool>("enabled");
          if(enable){const auto bias=array<6>(t.get_child("bias"));for(int i=0;i<6;++i)contact_bias[i].store(bias[i]);}
          contact_guard_enabled.store(enable);event("ack",id);continue;
        }
#endif
        if(op=="shutdown"){request_stop.store(true);shutdown.store(true);halt.store(true);break;}
        if(op=="stop"){request_stop.store(true);halt.store(true);event("ack",id);continue;}
        if(op=="begin"){if(control.load()||faulted.load())throw std::runtime_error("Controller unavailable");finish.store(false);halt.store(false);request_stop.store(false);stop_called.store(false);stop_failed.store(false);begin.store(true);event("ack",id);continue;}
        if(op=="finish"){finish.store(true);event("ack",id);continue;}
        if(op=="recover_idle"){
          if(control.load()||pending.load()||hand_request.load()!=hand_done.load())throw std::runtime_error("Idle recovery requires completed arm/hand operations");
          franka::RobotState measured;{std::lock_guard<std::mutex>lock(latest_mutex);measured=latest;}
          if(measured.robot_mode!=franka::RobotMode::kIdle||measured.current_errors)throw std::runtime_error("Idle recovery cannot clear robot errors");
          for(double v:measured.dq)if(std::abs(v)>.01)throw std::runtime_error("Idle recovery requires stationary arm");
          cfg.guard(measured.q);faulted.store(false);halt.store(false);request_stop.store(false);stop_called.store(false);stop_failed.store(false);event("ack",id);continue;
        }
        if(op=="hand"){if(hand_request.load()!=hand_done.load()||faulted.load())throw std::runtime_error("Hand busy or robot fault");hand_ok.store(false);hand_kind.store(t.get<int>("kind"));hand_request.store(id);event("ack",id);continue;}
        if(op=="load"){
          if(paths.count(id))throw std::runtime_error("Path identifiers are immutable");auto p=std::make_unique<Path>();p->id=id;auto d=t.get_child("path");p->duration=d.get<double>("duration_s");p->q0=array<7>(d.get_child("q0"));p->q1=array<7>(d.get_child("q1"));p->target=array<16>(d.get_child("target"));p->tracking=d.get<bool>("tracking",false);p->release_at=d.get<double>("release_at",-1);p->release_deadline=d.get<double>("release_deadline",-1);
          if(p->tracking){auto o=array<3>(d.get_child("tracking_origin"));p->tracking_origin=V3(o.data());if(p->release_at<0||p->release_deadline<=p->release_at||p->release_deadline>p->duration)throw std::runtime_error("Release schedule");}
          std::vector<double>knots;for(auto&v:d.get_child("knots_s"))knots.push_back(v.second.get_value<double>());size_t k=0;for(auto&v:d.get_child("coefficients")){if(k>=knots.size())throw std::runtime_error("Spline dimensions");Segment s;s.t=knots[k++];int j=0;for(auto&c:v.second){if(j>=7)throw std::runtime_error("Spline dimensions");s.c[j++]=array<6>(c.second);}if(j!=7)throw std::runtime_error("Spline dimensions");p->segments.push_back(s);}
          if(p->duration<=0||p->duration>10||p->segments.empty()||k!=knots.size()||k>1024||knots[0]!=0)throw std::runtime_error("Path size or duration");
          for(size_t i=1;i<k;++i)if(knots[i]<=knots[i-1]||knots[i]>=p->duration)throw std::runtime_error("Path knots");
          for(int i=0;i<=1000;++i){double x=p->duration*i/1000;auto q=p->eval(x);cfg.guard(q);auto pose=cfg.model.evaluate(q).first;auto pos=pose.block<3,1>(0,3);if((pos.array()<cfg.lower.array()-1e-9).any()||(pos.array()>cfg.upper.array()+1e-9).any())throw std::runtime_error("Path workspace");for(int derv=1;derv<=3;++derv){auto derivative=p->eval(x,derv);for(int j=0;j<7;++j){double z=std::abs(derivative[j]);if(z>(derv==1?cfg.speed*.86:derv==2?cfg.acc*1.001:cfg.jerk*1.001))throw std::runtime_error("Path derivative limit");auto& bound=derv==1?p->max_v:derv==2?p->max_a:p->max_j;bound[j]=std::max(bound[j],z);}}}
          if((cfg.model.evaluate(p->q1).first.block<3,1>(0,3)-xyz(p->target)).norm()>.0001)throw std::runtime_error("Path endpoint model mismatch");
          if(mlock(p->segments.data(),p->segments.size()*sizeof(Segment)))throw std::runtime_error("Cannot lock cached path");paths[id]=std::move(p);event("ack",id);continue;
        }
        if(op=="go"){if(!control.load()||pending.load())throw std::runtime_error("No control session or pending path");pending.store(paths.at(id).get());event("ack",id);continue;}
        if(op=="servo"){
          if(!control.load()||pending.load()||paths.count(id))throw std::runtime_error("Servo unavailable");auto p=std::make_unique<Path>();p->id=id;p->servo=true;p->duration=.08;p->q1=array<7>(t.get_child("q"));p->target=array<16>(t.get_child("target"));cfg.guard(p->q1);paths[id]=std::move(p);pending.store(paths[id].get());event("ack",id);continue;
        }
        throw std::runtime_error("Unknown command");
      }catch(const std::exception&e){event("rejected",id,e.what());}
    }}catch(const std::exception&e){event("input_failed",0,e.what());}halt.store(true);shutdown.store(true);});
    event("ready");int entry_number=0;
    while(!shutdown.load()&&!interrupted){
      auto state=robot.readOnce();tap.offer(state);if(!begin.exchange(false))continue;
      if(state.current_errors||state.robot_mode!=franka::RobotMode::kIdle){event("cycle_stopped",0,"Robot must be fault-free Idle");continue;}
      control.store(true);last_done.store(0);active_id.store(0);Path*path=nullptr;double elapsed=0,settled=0,hold_age=0,cycle_start=now(),speed_stable=0;Q raw=state.q_d,sent=state.q_d,velocity{},acceleration{},path_offset{};Online online;bool release_triggered=false,first_callback=true;int release_id=0;std::string reason;
      struct EntrySample{double dt=0;uint64_t time=0;Q desired{},dq{},ddq{},raw{},sent{};bool command=false,error=false;};
      std::array<EntrySample,128> entry{};size_t entry_count=0;
      if(mlock(entry.data(),sizeof(entry)))throw std::runtime_error("Cannot lock entry diagnostics");
      struct TrackingSample{double t=0,dt=0,vy=0,monitor_vy=0,window_s=0,vz=0,x=0,y=0,z=0,width=0;uint64_t ms=0;int phase=0;bool requested=false,opened=false,matched=false;};
      std::array<TrackingSample,10000> tracking{};size_t tracking_count=0;
      if(mlock(tracking.data(),sizeof(tracking)))throw std::runtime_error("Cannot lock release diagnostics");
      event("cycle_started");
      try{robot.control([&](const franka::RobotState&s,franka::Duration period)->franka::JointPositions{
        const double dt=period.toSec();
        EntrySample* debug=nullptr;if(entry_count<entry.size()){debug=&entry[entry_count++];debug->dt=dt;debug->time=s.time.toMSec();debug->desired=s.q_d;debug->dq=s.dq_d;debug->ddq=s.ddq_d;debug->error=bool(s.current_errors);}
        if(s.current_errors)throw std::runtime_error("Robot fault");
        cfg.guard(s.q);cfg.guard(s.q_d);
        if(first_callback){
          // Entering FCI may refresh q_d from the measured posture. A readOnce
          // reference taken before control can differ by tens of microradians,
          // enough to violate the first 1ms acceleration check.
          raw=s.q_d;sent=s.q_d;velocity=s.dq_d;acceleration=s.ddq_d;
          if(debug){debug->raw=raw;debug->sent=sent;debug->command=true;}
          first_callback=false;tap.offer(s,&raw,&sent,0,0);
          return franka::JointPositions(sent);
        }
        for(int j=0;j<7;++j)if(std::abs(s.dq[j])>cfg.speed+.01||std::abs(s.q[j]-s.q_d[j])>.02)throw std::runtime_error("Measured joint speed/tracking");
        for(int i=0;i<16;++i)if(std::abs(s.F_T_EE[i]-cfg.tool[i])>1e-6)throw std::runtime_error("Tool frame changed");
        V3 actual=xyz(s.O_T_EE);if(!actual.allFinite()||(actual.array()<cfg.lower.array()-.001).any()||(actual.array()>cfg.upper.array()+.001).any())throw std::runtime_error("Live workspace");
        if(interrupted||halt.load()||shutdown.load()||now()-last_ping.load()>cfg.heartbeat||now()-cycle_start>cfg.cycle_timeout||dt>.020001)request_stop.store(true);
#ifdef LAYER_STACKING_CONTROLLER
        if(contact_guard_enabled.load()){
          std::array<double,6>w{};for(int i=0;i<6;++i){w[i]=s.O_F_ext_hat_K[i]-contact_bias[i].load();if(!std::isfinite(w[i]))throw std::runtime_error("Invalid layer contact wrench");}
          if(std::sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2])>cfg.contact_force_limit||std::hypot(w[0],w[1])>cfg.contact_lateral_limit||std::sqrt(w[3]*w[3]+w[4]*w[4]+w[5]*w[5])>cfg.contact_torque_limit){reason="Layer contact force/torque guard";request_stop.store(true);}
        }
#endif
        if(stop_failed.load())throw std::runtime_error("SDK stop failed");
        {
          const bool stopping=request_stop.load();
          Path*next=stopping?nullptr:pending.exchange(nullptr);
          if(next){
            bool bridge_valid=true;
            if(path&&!next->servo)throw std::runtime_error("Cached path replaced before completion");
            if(next->servo){if(path&&!path->servo)throw std::runtime_error("Servo while cached path active");
              if((cfg.model.evaluate(next->q1).first.block<3,1>(0,3)-actual).norm()>.0035)throw std::runtime_error("Visual correction is not local");
              online.fit(sent,velocity,acceleration,next->q1,.08,cfg);
            }else{for(int j=0;j<7;++j){path_offset[j]=sent[j]-next->q0[j];const double offset=std::abs(path_offset[j]),T=next->duration;
              if(offset>.001){reason="Cached path starts at a different command posture";bridge_valid=false;break;}
              if(next->max_v[j]+1.875*offset/T>cfg.speed||next->max_a[j]+5.774*offset/(T*T)>cfg.acc||next->max_j[j]+60*offset/(T*T*T)>cfg.jerk){reason="Reference bridge exceeds derivative headroom";bridge_valid=false;break;}
            }}
            if(bridge_valid){path=next;elapsed=0;settled=0;hold_age=0;speed_stable=0;release_triggered=false;active_id.store(path->id);}
            else request_stop.store(true); // SDK stop worker; never throw a recoverable bridge rejection out of FCI.
          }
          if(path){elapsed+=dt;phase_time.store(elapsed);double duration=path->servo?online.duration:path->duration;raw=path->servo?online.eval(elapsed):path->eval(elapsed);
            if(!path->servo){const double u=std::clamp(elapsed/duration,0.,1.);for(int j=0;j<7;++j)raw[j]+=path_offset[j]*(1-blend(u));}
            if(path->tracking&&!stopping){
              double y=actual[1]-path->tracking_origin[1];if(y<-.002||y>.07||std::abs(actual[0]-path->tracking_origin[0])>.002){reason="Conveyor corridor/stroke";request_stop.store(true);}
              auto jac=cfg.model.evaluate(s.q).second;double vy=0,vz=0;for(int j=0;j<7;++j){vy+=cfg.model.length*jac(1,j)*s.dq[j];vz+=cfg.model.length*jac(2,j)*s.dq[j];}
              const double tolerance=std::min(.02,cfg.belt_speed*.2);
              // Measured TCP displacement over20ms rejects encoder-derivative
              // spikes while retaining the configured conveyor-speed corridor.
              double monitor_vy=vy,window_s=0;
              if(tracking_count){size_t i=tracking_count-1;while(i>0&&elapsed-tracking[i].t<.020)--i;
                window_s=elapsed-tracking[i].t;
                if(window_s>=.019&&window_s<=.040)monitor_vy=(actual[1]-tracking[i].y)/window_s;
              }
              const bool window_ready=window_s>=.019&&window_s<=.040;
              const bool raw_bounded=vy>=-tolerance&&vy<=cfg.release_upper;
              const bool matched=window_ready&&raw_bounded&&monitor_vy>=cfg.release_lower&&monitor_vy<=cfg.release_upper;
              const bool contact_ready=std::abs(actual[2]-path->tracking_origin[2])<=cfg.endpoint&&std::abs(vz)<=.002;
              speed_stable=(matched||!cfg.enforce_release_speed)&&contact_ready?speed_stable+dt:0;
              // Recoverable speed/height stops use the SDK stop worker. Throwing
              // out of the callback here previously left a communication Reflex.
              if(cfg.enforce_release_speed&&release_triggered&&hand_done.load()!=release_id&&!matched){reason="Gripper +Y speed mismatch during opening";request_stop.store(true);}
              if(elapsed>=path->release_at&&!release_triggered&&speed_stable>=.010&&!request_stop.load()){
                if(hand_request.load()!=hand_done.load()){reason="Hand busy at release";request_stop.store(true);}
                else{release_id=1000000+path->id;hand_kind.store(2);hand_ok.store(false);hand_request.store(release_id);release_triggered=true;}
              }
              if(elapsed>=path->release_at+.04&&!release_triggered){reason="Placement speed/height did not settle before opening";request_stop.store(true);}
              if(elapsed>=path->release_deadline&&(!hand_ok.load()||hand_done.load()!=release_id)){reason="Release deadline missed";request_stop.store(true);}
              if(tracking_count<tracking.size()){
                auto&d=tracking[tracking_count++];d.t=elapsed;d.dt=dt;d.ms=s.time.toMSec();d.phase=path->id;
                d.vy=vy;d.monitor_vy=monitor_vy;d.window_s=window_s;d.vz=vz;d.x=actual[0];d.y=actual[1];d.z=actual[2];d.width=hand_width.load();
                d.requested=release_triggered;d.opened=release_triggered&&hand_done.load()==release_id&&hand_ok.load();d.matched=matched;
              }else{reason="Release diagnostics capacity exceeded";request_stop.store(true);}
            }
            if(path&&elapsed>=duration){
              double pos=(actual-xyz(path->target)).norm();double rotation=angle(s.O_T_EE,path->target);double v=0;for(double x:s.dq)v=std::max(v,std::abs(x));
              settled=pos<=cfg.endpoint&&rotation<=cfg.angle_tol&&v<.01?settled+dt:0;hold_age+=dt;
              if(settled>=cfg.endpoint_stable){last_done.store(path->id);path=nullptr;active_id.store(0);}
              else if(hold_age>cfg.endpoint_timeout){reason="Endpoint did not settle; inspect requested/sent/desired/measured diagnostics";request_stop.store(true);}
            }
          }
        }
        auto expected=cfg.model.evaluate(raw).first;
        if(!request_stop.load()&&(actual-expected.block<3,1>(0,3)).norm()>.005)throw std::runtime_error("TCP tracking corridor");
        // Apply the exact SDK limiter explicitly, so post-limiter commands are
        // observable. Robot::control's duplicate limiter is disabled below.
        auto upper=robot.getUpperJointVelocityLimits(s.q_d),lower=robot.getLowerJointVelocityLimits(s.q_d);
        for(int j=0;j<7;++j){upper[j]=std::min(upper[j],cfg.speed);lower[j]=std::max(lower[j],-cfg.speed);}
        // Keep the issued double-precision command and its derivatives. Feeding
        // quantized q_d back into a low-jerk position limiter differentiates
        // rounding noise three times and produced the measured oscillation.
        // A genuinely missed state packet explicitly reanchors to SDK state.
        if(dt>.0015){sent=s.q_d;velocity=s.dq_d;acceleration=s.ddq_d;}
        const auto next=franka::limitRate(upper,lower,franka::kMaxJointAcceleration,franka::kMaxJointJerk,raw,sent,velocity,acceleration);
        Q next_velocity{},next_acceleration{};
        for(int j=0;j<7;++j){next_velocity[j]=(next[j]-sent[j])/.001;next_acceleration[j]=(next_velocity[j]-velocity[j])/.001;}
        sent=next;velocity=next_velocity;acceleration=next_acceleration;cfg.guard(sent);
        if(debug){debug->raw=raw;debug->sent=sent;debug->command=true;}
        tap.offer(s,&raw,&sent,active_id.load(),elapsed);
        franka::JointPositions result(sent);bool stationary=true;for(int j=0;j<7;++j)stationary=stationary&&std::abs(s.dq[j])<.01&&std::abs(s.dq_d[j])<.01;
        if(!request_stop.load()&&finish.load()&&!path&&!pending.load()&&stationary)return franka::MotionFinished(result);
        return result;
      },franka::ControllerMode::kJointImpedance,false,franka::kMaxCutoffFrequency);
      }catch(const std::exception&e){if(reason.empty())reason=e.what();else reason+="; "+std::string(e.what());faulted.store(true);}
      {std::ofstream diagnostics(std::filesystem::path(argv[1]).parent_path()/("control_entry_"+std::to_string(++entry_number)+".csv"));diagnostics<<std::setprecision(17)<<"robot_ms,period_s,error,command";for(const char*name:{"desired","dq","ddq","raw","sent"})for(int j=0;j<7;++j)diagnostics<<','<<name<<j;diagnostics<<'\n';for(size_t i=0;i<entry_count;++i){const auto&d=entry[i];diagnostics<<d.time<<','<<d.dt<<','<<d.error<<','<<d.command;for(const auto*q:{&d.desired,&d.dq,&d.ddq,&d.raw,&d.sent})for(double x:*q)diagnostics<<','<<x;diagnostics<<'\n';}munlock(entry.data(),sizeof(entry));}
      if(tracking_count){
        std::ofstream diagnostics(std::filesystem::path(argv[1]).parent_path()/("tracking_"+std::to_string(entry_number)+".csv"));
        diagnostics<<std::setprecision(17)<<"robot_ms,phase,t,period_s,vy,monitor_vy,window_s,vz,x,y,z,width,release_requested,opening_complete,matched\n";
        double sum=0,minimum=1,maximum=-1,raw_minimum=1,raw_maximum=-1,start=-1,complete=-1,max_gap=0,max_vz=0;int samples=0,bad=0;
        for(size_t i=0;i<tracking_count;++i){const auto&d=tracking[i];
          diagnostics<<d.ms<<','<<d.phase<<','<<d.t<<','<<d.dt<<','<<d.vy<<','<<d.monitor_vy<<','<<d.window_s<<','<<d.vz<<','<<d.x<<','<<d.y<<','<<d.z<<','<<d.width<<','<<d.requested<<','<<d.opened<<','<<d.matched<<'\n';
          if(d.requested&&complete<0){if(start<0)start=d.t;sum+=d.monitor_vy;++samples;minimum=std::min(minimum,d.monitor_vy);maximum=std::max(maximum,d.monitor_vy);raw_minimum=std::min(raw_minimum,d.vy);raw_maximum=std::max(raw_maximum,d.vy);max_gap=std::max(max_gap,d.dt);max_vz=std::max(max_vz,std::abs(d.vz));if(!d.matched)++bad;if(d.opened)complete=d.t;}
        }
        std::lock_guard<std::mutex>lock(output_mutex);
        std::cout<<"{\"type\":\"release_metrics\",\"speed_measurement_window_s\":0.02,\"samples\":"<<samples<<",\"mean_vy_m_s\":"<<(samples?sum/samples:0)<<",\"min_vy_m_s\":"<<(samples?minimum:0)<<",\"max_vy_m_s\":"<<(samples?maximum:0)<<",\"raw_min_vy_m_s\":"<<(samples?raw_minimum:0)<<",\"raw_max_vy_m_s\":"<<(samples?raw_maximum:0)<<",\"bad_speed_samples\":"<<bad<<",\"maximum_sample_gap_s\":"<<max_gap<<",\"maximum_abs_vz_m_s\":"<<max_vz<<",\"release_start_t\":"<<start<<",\"opening_complete_t\":"<<complete<<",\"full_opening_speed_verified\":"<<(samples>0&&complete>=start&&start>=0&&bad==0&&max_gap<=.020001?"true":"false")<<"}\n"<<std::flush;
      }
      munlock(tracking.data(),sizeof(tracking));
      control.store(false);pending.store(nullptr);active_id.store(0);finish.store(false);
      auto end=robot.readOnce();tap.offer(end);
      if(end.current_errors||end.robot_mode!=franka::RobotMode::kIdle)faulted.store(true);
      event(reason.empty()?"cycle_finished":"cycle_stopped",last_done.load(),reason);
    }
    shutdown.store(true);halt.store(true);stop_worker_end.store(true);if(input.joinable()){if(!std::cin.eof())input.detach();else input.join();}stop_worker.join();hand_worker.join();event("closed");return 0;
  }catch(const std::exception&e){event("fatal",0,e.what());return 1;}
}
