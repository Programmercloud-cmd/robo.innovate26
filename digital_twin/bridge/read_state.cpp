// Read-only telemetry. Calls Robot::readOnce only: no control, configuration,
// recovery, homing, force or hand position commands.
#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <franka/robot.h>
#include "state_transport.hpp"
static volatile std::sig_atomic_t stop_requested=0;
static void stop(int){stop_requested=1;}
int main(int argc,char**argv){
  if(argc<2||argc>4){std::cerr<<"Usage: read_state ROBOT_IP [HZ=50] [DURATION_S=0]\n";return 2;}
  try{
    const double hz=argc>2?std::stod(argv[2]):50;
    const double duration=argc>3?std::stod(argv[3]):0;
    if(hz<1||hz>100||duration<0)throw std::runtime_error("Invalid read-only sampling settings");
    std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
    twin::SdkLease lease(false);
    franka::Robot robot(argv[1],franka::RealtimeConfig::kIgnore);
    const auto began=std::chrono::steady_clock::now();auto next=began;
    std::cout<<std::setprecision(17);
    while(!stop_requested){
      if(lease.foregroundWaiting())throw twin::YieldToOwner();
      const auto state=robot.readOnce();const auto now=std::chrono::steady_clock::now();
      if(now>=next){
        const auto unix_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        std::cout<<"{\"unix_ns\":"<<unix_ns<<",\"state\":"<<state<<"}\n"<<std::flush;
        next=now+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1/hz));
      }
      if(duration>0&&std::chrono::duration<double>(now-began).count()>=duration)break;
    }
  }catch(const twin::YieldToOwner&e){std::cerr<<e.what()<<'\n';return 75;}
  catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
