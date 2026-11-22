// Operator-authorized recovery of the current FCI Reflex; no motion/gripper API.
#include <cmath>
#include <iomanip>
#include <iostream>
#include <franka/robot.h>
#include "../../digital_twin/bridge/state_transport.hpp"

int main(){
  try{
    twin::SdkLease exclusive(true);
    franka::Robot robot("172.16.0.2",franka::RealtimeConfig::kIgnore);
    twin::StateTap telemetry;
    auto state=robot.readOnce();telemetry.offer(state);
    std::cout<<std::setprecision(17)<<"BEFORE "<<state<<std::endl;
    for(double v:state.dq)if(!std::isfinite(v)||std::abs(v)>.01)throw std::runtime_error("Recovery requires a stationary robot");
    if(state.robot_mode==franka::RobotMode::kReflex)robot.automaticErrorRecovery();
    else if(state.robot_mode!=franka::RobotMode::kIdle||state.current_errors)
      throw std::runtime_error("Current state is neither the authorized Reflex nor fault-free Idle");
    state=robot.readOnce();std::cout<<"RECOVERY_RESULT "<<state<<std::endl;
    const auto start=state.time.toMSec();
    do{
      state=robot.readOnce();telemetry.offer(state);
      if(state.robot_mode!=franka::RobotMode::kIdle||state.current_errors)
        throw std::runtime_error("Recovery did not produce fault-free Idle");
      for(double v:state.dq)if(!std::isfinite(v)||std::abs(v)>.01)throw std::runtime_error("Robot not stationary after recovery");
    }while(state.time.toMSec()-start<500);
    std::cout<<"AFTER "<<state<<"\nPASS recovered; stationary Idle and empty current_errors for500ms; no motion command\n";
    return 0;
  }catch(const std::exception&e){std::cerr<<"STOP "<<e.what()<<std::endl;return 1;}
}
