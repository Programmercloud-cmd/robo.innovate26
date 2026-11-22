// No robot connection, no samples published: test RT inheritance isolation only.
#include "state_transport.hpp"
#include <filesystem>
#include <iostream>
int main(){
  try {
    cpu_set_t cpus;CPU_ZERO(&cpus);CPU_SET(2,&cpus);
    if(sched_setaffinity(0,sizeof(cpus),&cpus))throw std::runtime_error("Cannot pin probe");
    sched_param realtime{};realtime.sched_priority=99;
    if(pthread_setschedparam(pthread_self(),SCHED_FIFO,&realtime))throw std::runtime_error("Cannot emulate SDK RT constructor");
    twin::StateTap tap;
    int workers=0;
    for(const auto& item:std::filesystem::directory_iterator("/proc/self/task")){
      const auto tid=std::stoi(item.path().filename());
      if(tid==getpid())continue;
      if(sched_getscheduler(tid)!=SCHED_OTHER)throw std::runtime_error("Auxiliary worker inherited real-time policy");
      ++workers;
    }
    if(workers!=1)throw std::runtime_error("Expected exactly one telemetry worker");
    std::cout<<"PASS: main FIFO99; telemetry worker SCHED_OTHER; no robot connection or samples\n";
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
