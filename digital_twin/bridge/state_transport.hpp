#pragma once
// Local SDK ownership + one-way measured-state transport. No robot commands.
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <functional>
#include <pthread.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <franka/robot_state.h>

namespace twin {
inline std::string directory() {
  const auto path="/tmp/fr3-sdk-"+std::to_string(getuid());
  if(mkdir(path.c_str(),0700) && errno!=EEXIST)throw std::runtime_error("Cannot create SDK IPC directory");
  struct stat info{};
  if(lstat(path.c_str(),&info)||!S_ISDIR(info.st_mode)||info.st_uid!=getuid()||(info.st_mode&0077))
    throw std::runtime_error("SDK IPC directory must be owned by this user, mode 0700");
  return path;
}
struct YieldToOwner:std::runtime_error {YieldToOwner():std::runtime_error("SDK reader yielding to foreground owner") {}};

class HandLease {
  int fd_=-1,demand_=-1;
 public:
  HandLease(){const auto path=directory();
    demand_=open((path+"/hand_foreground.lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    fd_=open((path+"/hand.lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    if(demand_<0||fd_<0||flock(demand_,LOCK_EX)||flock(fd_,LOCK_EX)){
      if(fd_>=0)close(fd_);if(demand_>=0)close(demand_);throw std::runtime_error("Cannot acquire hand reader ownership");}}
  ~HandLease(){if(fd_>=0)close(fd_);if(demand_>=0)close(demand_);}
  HandLease(const HandLease&)=delete;HandLease& operator=(const HandLease&)=delete;
};

class BackgroundHandLease {
  int fd_=-1,arm_demand_=-1,hand_demand_=-1;
  void release(){if(fd_>=0)close(fd_);if(arm_demand_>=0)close(arm_demand_);if(hand_demand_>=0)close(hand_demand_);fd_=arm_demand_=hand_demand_=-1;}
 public:
  BackgroundHandLease(){const auto path=directory();
    arm_demand_=open((path+"/foreground.lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    hand_demand_=open((path+"/hand_foreground.lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    fd_=open((path+"/hand.lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    if(fd_<0||arm_demand_<0||hand_demand_<0){release();throw std::runtime_error("Cannot open background hand ownership locks");}
    if(foregroundWaiting()||flock(fd_,LOCK_EX|LOCK_NB)){release();throw YieldToOwner();}}
  bool foregroundWaiting(){
    for(int fd:{arm_demand_,hand_demand_}){
      if(flock(fd,LOCK_SH|LOCK_NB))return true;
      flock(fd,LOCK_UN);
    }return false;
  }
  ~BackgroundHandLease(){release();}
  BackgroundHandLease(const BackgroundHandLease&)=delete;BackgroundHandLease&operator=(const BackgroundHandLease&)=delete;
};

class SdkLease {
  int demand_=-1,owner_=-1;
 public:
  explicit SdkLease(bool foreground) {
    const auto path=directory();
    demand_=open((path+"/foreground.lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    owner_=open((path+"/owner.lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    if(demand_<0||owner_<0){release();throw std::runtime_error("Cannot open SDK ownership locks");}
    if(foreground) {
      if(flock(demand_,LOCK_EX)||flock(owner_,LOCK_EX)){release();throw std::runtime_error("Cannot acquire SDK ownership");}
    } else {
      if(flock(demand_,LOCK_SH|LOCK_NB)||flock(owner_,LOCK_EX|LOCK_NB)){release();throw YieldToOwner();}
      flock(demand_,LOCK_UN);
    }
  }
  bool foregroundWaiting() {
    if(flock(demand_,LOCK_SH|LOCK_NB))return true;
    flock(demand_,LOCK_UN);return false;
  }
  void release(){if(owner_>=0){close(owner_);owner_=-1;}if(demand_>=0){close(demand_);demand_=-1;}}
  ~SdkLease(){release();}
  SdkLease(const SdkLease&)=delete;SdkLease& operator=(const SdkLease&)=delete;
};

// Single producer/single consumer. Callback copies into prefaulted locked slots;
// all formatting, allocation and socket work happens on an ordinary worker.
class StateTap {
  static_assert(std::atomic<size_t>::is_always_lock_free,"Telemetry indices must be lock-free");
 public:
  struct Item {franka::RobotState state{};int64_t unix_ns=0;
    std::array<double,7> requested{},sent{};int phase=-1;double phase_time=0;};
  using Observer=std::function<void(const Item&)>;
 private:
  std::array<Item,8> queue_{};
  alignas(64) std::atomic<size_t> head_{0};
  alignas(64) std::atomic<size_t> tail_{0};
  std::atomic<bool> stop_{false};
  std::atomic<bool> ready_{false},ordinary_{false};
  std::thread worker_;
  std::chrono::steady_clock::time_point next_{};
  std::string path_;
  Observer observer_;
  int worker_cpu_=-1;
  std::atomic<double> hand_width_{-1};
  std::atomic<int64_t> hand_time_{0};
 public:
  explicit StateTap(Observer observer={},int worker_cpu=-1):path_(directory()+"/state.sock"),observer_(std::move(observer)),worker_cpu_(worker_cpu) {
    if(mlock(queue_.data(),sizeof(queue_)))throw std::runtime_error("Cannot lock telemetry queue");
    try {worker_=std::thread([this]{
      sched_param ordinary{};
      bool valid=pthread_setschedparam(pthread_self(),SCHED_OTHER,&ordinary)==0;
      if(worker_cpu_>=0){cpu_set_t cpus;CPU_ZERO(&cpus);CPU_SET(worker_cpu_,&cpus);valid=valid&&pthread_setaffinity_np(pthread_self(),sizeof(cpus),&cpus)==0;}
      ordinary_.store(valid);
      ready_.store(true);
      if(!ordinary_.load())return;
      const int fd=socket(AF_UNIX,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
      sockaddr_un address{};address.sun_family=AF_UNIX;
      std::strncpy(address.sun_path,path_.c_str(),sizeof(address.sun_path)-1);
      while(!stop_.load(std::memory_order_relaxed)) {
        const auto tail=tail_.load(std::memory_order_relaxed);
        if(tail==head_.load(std::memory_order_acquire)){std::this_thread::sleep_for(std::chrono::milliseconds(2));continue;}
        const auto item=queue_[tail];tail_.store((tail+1)%queue_.size(),std::memory_order_release);
        try {
          std::ostringstream out;out.precision(17);
          out<<"{\"unix_ns\":"<<item.unix_ns<<",\"telemetry_origin\":\"foreground_sdk_owner\",\"state\":"<<item.state;
          const auto hand_stamp=hand_time_.load();
          if(hand_stamp>0&&item.unix_ns-hand_stamp>=0&&item.unix_ns-hand_stamp<500000000)
            out<<",\"width_m\":"<<hand_width_.load()<<",\"width_unix_ns\":"<<hand_stamp;
          out<<"}\n";
          const auto payload=out.str();
          if(fd>=0)sendto(fd,payload.data(),payload.size(),MSG_DONTWAIT|MSG_NOSIGNAL,
                         reinterpret_cast<sockaddr*>(&address),sizeof(address));
        } catch(...) {} // Telemetry failure must not terminate the controller.
        if(observer_){try{observer_(item);}catch(...){}}
      }
      if(fd>=0)close(fd);
    });} catch(...) {munlock(queue_.data(),sizeof(queue_));throw;}
    while(!ready_.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if(!ordinary_.load()){stop_.store(true);worker_.join();munlock(queue_.data(),sizeof(queue_));throw std::runtime_error("Cannot demote telemetry worker");}
  }
  void offer(const franka::RobotState& state,const std::array<double,7>*requested=nullptr,
             const std::array<double,7>*sent=nullptr,int phase=-1,double phase_time=0) noexcept {
    const auto now=std::chrono::steady_clock::now();if(now<next_)return;
    next_=now+std::chrono::milliseconds(20);
    const auto head=head_.load(std::memory_order_relaxed),next=(head+1)%queue_.size();
    if(next==tail_.load(std::memory_order_acquire))return; // Drop, never wait.
    queue_[head].state=state;
    queue_[head].requested=requested?*requested:state.q_d;
    queue_[head].sent=sent?*sent:state.q_d;
    queue_[head].phase=phase;queue_[head].phase_time=phase_time;
    queue_[head].unix_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    head_.store(next,std::memory_order_release);
  }
  void updateHand(double width,int64_t unix_ns)noexcept{hand_width_.store(width);hand_time_.store(unix_ns);}
  ~StateTap(){stop_.store(true);if(worker_.joinable())worker_.join();munlock(queue_.data(),sizeof(queue_));}
  StateTap(const StateTap&)=delete;StateTap& operator=(const StateTap&)=delete;
};
}
