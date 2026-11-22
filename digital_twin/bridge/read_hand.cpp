// Persistent receive-only hand session. No arm, move, grasp, home or stop API.
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <franka/gripper.h>
#include "state_transport.hpp"

namespace {volatile std::sig_atomic_t stopped=0;void stopReader(int){stopped=1;}}
int main(int argc,char**argv){
  if(argc!=2)return 2;
  std::signal(SIGINT,stopReader);std::signal(SIGTERM,stopReader);
  try{
    twin::BackgroundHandLease ownership;
    if(ownership.foregroundWaiting())return 75;
    franka::Gripper hand(argv[1]);
    std::cout<<std::setprecision(17)<<std::unitbuf;
    uint64_t previous=0;bool first=true;
    while(!stopped){
      if(ownership.foregroundWaiting())return 75;
      const auto state=hand.readOnce();
      if(!std::isfinite(state.width)||state.width<0||state.width>.09||(!first&&state.time.toMSec()<=previous))
        throw std::runtime_error("Invalid or non-advancing measured hand state");
      first=false;previous=state.time.toMSec();
      const auto stamp=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      std::cout<<"{\"unix_ns\":"<<stamp<<",\"state\":"<<state<<"}\n";
      for(int i=0;i<10&&!stopped;++i){
        if(ownership.foregroundWaiting())return 75;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    return 0;
  }catch(const twin::YieldToOwner&){return 75;}
  catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
