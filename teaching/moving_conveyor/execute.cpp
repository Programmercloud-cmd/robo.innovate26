#include "model.hpp"
#include <franka/gripper.h>
#include "../../digital_twin/bridge/state_transport.hpp"
struct SplineSegment{double time;std::array<std::array<double,4>,7> coefficients;};
struct TrackingPlan{
  double ramp=0,hold=0,speed=0,height=0,total=0,gap=0,open=0,open_speed=0;int cpu=0,authorized=0;
  Pose conveyor{},tool{};Q q0{},lower{},upper{};V3 xyz_lower,xyz_upper;Model model;std::vector<SplineSegment> segments;
  TrackingPlan(const char*path){
    std::ifstream in(path);int version=0,n=0;in>>version>>n>>ramp>>hold>>speed>>height>>gap>>open>>open_speed>>cpu>>authorized;total=2*ramp+hold;
    auto read=[&](auto&array){for(auto&v:array)in>>v;};read(conveyor);read(tool);read(q0);read(lower);read(upper);
    for(int i=0;i<3;++i)in>>xyz_lower[i];for(int i=0;i<3;++i)in>>xyz_upper[i];in>>model.length;
    for(auto&t:model.origins)for(int i=0;i<16;++i)in>>t.data()[i];for(int i=0;i<16;++i)in>>model.ee.data()[i];
    if(version!=2||n<2||n>2001)throw std::runtime_error("Invalid timed plan header");
    segments.resize(n);for(auto&segment:segments){in>>segment.time;for(auto&c:segment.coefficients)read(c);}
    if(!in||ramp<.2||ramp>.6||hold<.05||hold>.6||!std::isfinite(speed)||speed<=0||speed>.12||height<=0||height>.01||gap<=0||gap>.020001||open!=.07||open_speed<=0||open_speed>.1||cpu<0||cpu>=CPU_SETSIZE||authorized<0||authorized>1||speed*(ramp+hold)>.068001)
      throw std::runtime_error("Invalid timed conveyor limits");
    for(size_t i=0;i<segments.size();++i){const auto&s=segments[i];if(!std::isfinite(s.time)||(i&&s.time<=segments[i-1].time)||s.time<0||s.time>=total)throw std::runtime_error("Invalid time knots");for(const auto&c:s.coefficients)for(double x:c)if(!std::isfinite(x))throw std::runtime_error("Nonfinite coefficient");}
  }
  Q joint(double t,int derivative=0)const{
    t=std::clamp(t,0.,total);auto it=std::upper_bound(segments.begin(),segments.end(),t,[](double v,const SplineSegment&s){return v<s.time;});if(it!=segments.begin())--it;
    const double u=t-it->time;Q result{};for(int j=0;j<7;++j){auto c=it->coefficients[j];result[j]=derivative==0?((c[0]*u+c[1])*u+c[2])*u+c[3]:derivative==1?(3*c[0]*u+2*c[1])*u+c[2]:6*c[0]*u+2*c[1];}return result;
  }
  V3 expected(double t)const{
    t=std::clamp(t,0.,total);double y=0,z=0;const auto integral=[](double u){return 2.5*std::pow(u,4)-3*std::pow(u,5)+std::pow(u,6);};
    if(t<ramp){double u=t/ramp;y=speed*ramp*integral(u);z=height*(1-blend(u));}
    else if(t<=ramp+hold)y=speed*ramp*.5+speed*(t-ramp);
    else{double u=(t-ramp-hold)/ramp;y=speed*ramp*.5+speed*hold+speed*ramp*(u-integral(u));}
    return xyz(conveyor)+V3(0,y,z);
  }
  void guard(const Q&q)const{
    for(int j=0;j<7;++j)if(!std::isfinite(q[j])||q[j]<lower[j]||q[j]>upper[j])throw std::runtime_error("Joint limit");
    const auto metric=model.metrics(q);if(metric.full<.1||metric.fixed<.08||metric.condition>50||metric.extension<.15)throw std::runtime_error("Jacobian guard");
  }
};
int main(int argc,char**argv){
  if(argc!=4||(std::string(argv[3])!="--check"&&std::string(argv[3])!="--execute"&&std::string(argv[3])!="--dry-motion"&&std::string(argv[3])!="--timing-test"))return 2;
  const bool dry=std::string(argv[3])=="--dry-motion";
  const bool timing_test=std::string(argv[3])=="--timing-test";
  std::cout<<std::unitbuf<<std::setprecision(17);std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
  try{
    TrackingPlan p(argv[1]);double maximum_error=0;
    for(int i=0;i<=4000;++i){double t=p.total*i/4000;auto q=p.joint(t);p.guard(q);auto actual=p.model.evaluate(q).first;maximum_error=std::max(maximum_error,(actual.block<3,1>(0,3)-p.expected(t)).norm());if(maximum_error>.0001)throw std::runtime_error("Frozen FK path mismatch");}
    if(std::string(argv[3])=="--check"){std::cout<<"PASS timed path model check; maximum error_m "<<maximum_error<<'\n';return 0;}
    if(!p.authorized&&!dry&&!timing_test)throw std::runtime_error("Hardware execution disabled: release timing has not been physically validated");
    if(timing_test&&p.conveyor[14]<.05)throw std::runtime_error("Empty hand timing test must stay at least50mm above robot-base Z0");
    cpu_set_t cpus;CPU_ZERO(&cpus);CPU_SET(p.cpu,&cpus);if(sched_setaffinity(0,sizeof(cpus),&cpus))throw std::runtime_error("Cannot pin controller");
    twin::SdkLease lease(true);
    franka::Robot robot("172.16.0.2",franka::RealtimeConfig::kEnforce);twin::StateTap telemetry;
    auto initial=robot.readOnce();telemetry.offer(initial);
    if(initial.current_errors||initial.robot_mode!=franka::RobotMode::kIdle)throw std::runtime_error("Arm not fault-free Idle");
    for(int j=0;j<7;++j)if(std::abs(initial.q[j]-p.q0[j])>.001||std::abs(initial.dq[j])>.01)throw std::runtime_error("Arm moved since preparation");
    for(int i=0;i<16;++i)if(std::abs(initial.F_T_EE[i]-p.tool[i])>1e-6)throw std::runtime_error("Tool changed");
    twin::HandLease hand_lease;
    franka::Gripper hand("172.16.0.2");const auto hand_before=hand.readOnce();
    if(dry?std::abs(hand_before.width-.07)>.001:hand_before.width<.0645||hand_before.width>.0655)throw std::runtime_error("Unexpected commissioning/loaded hand width");
    std::atomic<bool> trigger{false},cancel{false},released{false},release_failed{false};
    std::atomic<double> wall_elapsed{0},opening_time{0},width{hand_before.width};
    std::atomic<bool> opener_ready{false},opener_ordinary{false};
    std::thread opener([&]{sched_param ordinary{};opener_ordinary.store(pthread_setschedparam(pthread_self(),SCHED_OTHER,&ordinary)==0);opener_ready.store(true);
      if(!opener_ordinary.load()||dry)return;try{while(!trigger.load()&&!cancel.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));if(cancel.load())return;auto began=std::chrono::steady_clock::now();if(!hand.move(p.open,p.open_speed)){release_failed.store(true);return;}
      // One UDP reader, on this same worker, matches the measured bench path.
      // No concurrent observer contends with the blocking hand command.
      auto state=hand.readOnce();width.store(state.width);
      opening_time.store(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count());released.store(std::isfinite(state.width)&&std::abs(state.width-p.open)<=.001);
    }catch(...){release_failed.store(true);}});
    while(!opener_ready.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if(!opener_ordinary.load()){cancel.store(true);opener.join();throw std::runtime_error("Cannot demote hand worker");}
    struct Sample{double t;Pose pose;Q q;double vy;bool released;double width;};std::vector<Sample> samples(3000);for(auto&s:samples)s.t=0;
    if(mlock(samples.data(),samples.size()*sizeof(Sample))){cancel.store(true);opener.join();throw std::runtime_error("Cannot lock trace memory");}
    size_t count=0;double elapsed=0,startup=0,stable=0;bool warmed=false,first=true;Q reference{},offset{};std::exception_ptr failure;
    try{
      robot.control([&](const franka::RobotState&s,franka::Duration dt)->franka::JointPositions{
        telemetry.offer(s);
        if(interrupted||s.current_errors)throw std::runtime_error("Stop requested or robot fault");p.guard(s.q);p.guard(s.q_d);
        if(first){reference=s.q_d;for(int j=0;j<7;++j){offset[j]=reference[j]-p.q0[j];if(std::abs(offset[j])>.001)throw std::runtime_error("Changed command reference");}first=false;}
        if(!warmed){startup+=dt.toSec();stable=dt.toSec()<=p.gap?stable+dt.toSec():0;if(stable>=.5){warmed=true;elapsed=0;}else if(startup>2)throw std::runtime_error("Startup timing failed");}
        else{if(dt.toSec()>p.gap)throw std::runtime_error("Callback timing gap");elapsed+=dt.toSec();}
        wall_elapsed.store(elapsed);const V3 position=xyz(s.O_T_EE);const double relative_y=position[1]-p.conveyor[13];
        if((position.array()<p.xyz_lower.array()).any()||(position.array()>p.xyz_upper.array()).any()||relative_y<-.002||relative_y>.070||std::abs(position[0]-p.conveyor[12])>.002)throw std::runtime_error("Conveyor stroke/corridor bound");
        if((position-p.expected(elapsed)).norm()>.003)throw std::runtime_error("Tracking error");
        if(angle(s.O_T_EE,p.conveyor)>.03491)throw std::runtime_error("Tool orientation tracking error");
        const auto jacobian=p.model.evaluate(s.q).second;double measured_vy=0;for(int j=0;j<7;++j)measured_vy+=p.model.length*jacobian(1,j)*s.dq[j];
        if(elapsed>p.ramp+.025&&elapsed<p.ramp+p.hold-.01&&std::abs(measured_vy-p.speed)>std::min(.02,p.speed*.2))throw std::runtime_error("Measured speed does not match configured conveyor speed");
        for(int j=0;j<7;++j)if(std::abs(s.dq[j])>1||std::abs(s.q[j]-s.q_d[j])>.02)throw std::runtime_error("Joint speed/tracking bound");
        if(elapsed>=p.ramp&&!dry)trigger.store(true);
        if(!dry&&!timing_test&&(release_failed.load()||(elapsed>=p.ramp+p.hold&&!released.load())))throw std::runtime_error("Opening missed matching-speed window");
        Q command=p.joint(elapsed);for(int j=0;j<7;++j)command[j]+=offset[j]*(1-blend(elapsed/p.total));if(elapsed==0)command=reference;p.guard(command);
        if(count<samples.size())samples[count++]={elapsed,s.O_T_EE,s.q,measured_vy,released.load(),width.load()};
        franka::JointPositions result(command);if(elapsed>=p.total+.3)return franka::MotionFinished(result);return result;
      },franka::ControllerMode::kJointImpedance,true,1000);
    }catch(...){failure=std::current_exception();}
    cancel.store(true);opener.join();std::ofstream trace(argv[2]);trace<<std::setprecision(12)<<"t_s,x_m,y_m,z_m,vy_actual_m_s,released,width_m\n";for(size_t i=0;i<count;++i)trace<<samples[i].t<<','<<samples[i].pose[12]<<','<<samples[i].pose[13]<<','<<samples[i].pose[14]<<','<<samples[i].vy<<','<<samples[i].released<<','<<samples[i].width<<'\n';
    std::cout<<"OPENING_TIME_S "<<opening_time.load()<<" RELEASED "<<released.load()<<'\n';if(failure)std::rethrow_exception(failure);
    const auto final=robot.readOnce();telemetry.offer(final);if(final.current_errors||final.robot_mode!=franka::RobotMode::kIdle||(!dry&&!released.load()))throw std::runtime_error("Completion verification failed");
    if(timing_test&&(release_failed.load()||opening_time.load()>p.hold))throw std::runtime_error("Empty-hand timing window failed after smooth completion; no loaded placement enabled");
    std::cout<<"FINAL_STATE "<<final<<(dry?"\nPASS dry conveyor stroke; no release requested\n":timing_test?"\nPASS elevated empty hand timing stroke\n":"\nPASS timed conveyor stroke\n");
  }catch(const std::exception&e){std::cerr<<"STOP "<<e.what()<<'\n';return 1;}
}
