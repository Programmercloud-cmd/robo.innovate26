#pragma once
// No Robot/Gripper object or SDK connection. Construct/finish outside control.
// One producer calls offer(); stop control if it returns false or stop_requested.
// Existing measured-state forwarding remains separate.
#include <franka/robot_state.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <fcntl.h>
#include <iomanip>
#include <limits>
#include <pthread.h>
#include <sched.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace lego_seating {
struct ConditionProfiles {
  std::string sdk_version, tool_profile_id, payload_profile_id;
  std::string impedance_profile_id, limiter_profile_id, filter_profile_id;
  std::string wrench_profile_id, fixture_grasp_id, clock_id;
  std::uint64_t control_period_ns = 0;
};
struct RecordingMetadata {
  std::string recording_id, trial_id, provenance;
  std::uint64_t episode_origin_monotonic_ns = 0;
  std::uint64_t profile_revision = 0;
  ConditionProfiles conditions;
  std::vector<std::pair<int, std::string>> phase_names;
};
struct GripperSnapshot {
  double width_m = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t monotonic_ns = 0, sequence = 0;
  bool valid = false;
};
struct SampleContext {
  std::uint64_t sequence = 0, monotonic_ns = 0, condition_profile_revision = 0;
  int phase_id = 0;
  bool before_command_application = true;
  bool estimated_wrench_valid = false, singularity_valid = false, grasp_valid = false;
  GripperSnapshot hand;
};
struct RecorderHealth {
  std::uint64_t offered = 0, enqueued = 0, written = 0, dropped = 0, rejected = 0, discarded = 0;
  std::uint64_t disk_errors = 0, queue_high_water = 0;
  std::uint32_t fault_flags = 0;
  int worker_policy = -1, worker_priority = -1;
  bool stop_requested = false, finalized = false, integrity_valid = false;
};

template<std::size_t Capacity = 64>
class OwnerRecorder {
  static_assert(Capacity >= 2, "SPSC ring needs an empty sentinel slot");
  static_assert(std::atomic<std::uint64_t>::is_always_lock_free, "Counters must be lock-free");
  static_assert(std::atomic<std::uint32_t>::is_always_lock_free, "Fault flags must be lock-free");
  static_assert(std::atomic<std::size_t>::is_always_lock_free, "Indices must be lock-free");
  static_assert(std::atomic<bool>::is_always_lock_free, "Flags must be lock-free");
  enum Fault : std::uint32_t {
    Overflow = 1, BadContext = 2, BadSequence = 4, BadMonotonicClock = 8,
    BadRobotClock = 16, DiskFailure = 32, NonfiniteState = 64, WorkerPolicy = 128
  };
  struct Packet {
    franka::RobotState state{};
    std::array<double, 7> requested{}, sent{};
    SampleContext context{};
  };
  // All callback-accessed data, including counters and metadata revision, lock.
  struct alignas(64) RealtimeStorage {
    std::array<Packet, Capacity> queue{};
    alignas(64) std::atomic<std::size_t> head{0};
    alignas(64) std::atomic<std::size_t> tail{0};
    std::atomic<std::uint64_t> offered{0}, enqueued{0}, written{0};
    std::atomic<std::uint64_t> dropped{0}, rejected{0}, discarded{0};
    std::atomic<std::uint64_t> disk_errors{0}, high_water{0}, in_offer{0};
    std::atomic<std::uint32_t> faults{0};
    std::atomic<bool> stop_request{false}, closing{false}, worker_stop{false};
    std::atomic<bool> ready{false}, startup_ok{false}, finalized{false};
    std::uint64_t profile_revision = 0, episode_origin_monotonic_ns = 0;
    std::uint64_t last_sequence = 0, last_monotonic_ns = 0, last_robot_time_ms = 0;
    bool have_first_sample = false;  // Producer-only; worker never reads these.
  } rt_;
  RecordingMetadata metadata_;
  std::filesystem::path directory_;
  pthread_t worker_{};
  bool started_ = false, locked_ = false;
  int trace_fd_ = -1;
  std::atomic<int> worker_policy_{-1}, worker_priority_{-1};
  std::uint64_t trace_bytes_ = 0;  // Worker-only.

  void fault(std::uint32_t flag) noexcept {
    rt_.faults.fetch_or(flag, std::memory_order_relaxed);
    rt_.stop_request.store(true, std::memory_order_release);
  }
  static std::string quoted(const std::string& text) {
    std::ostringstream out;
    out << '"';
    for (unsigned char ch : text) {
      if (ch == '"' || ch == '\\') out << '\\' << ch;
      else if (ch < 0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(ch) << std::dec;
      else out << ch;
    }
    out << '"';
    return out.str();
  }
  template<class T, std::size_t N>
  static void array(std::ostream& out, const std::array<T, N>& values) {
    out << '[';
    for (std::size_t i = 0; i < N; ++i) { if (i) out << ','; out << values[i]; }
    out << ']';
  }
  static bool finite(double x) noexcept { return std::isfinite(x); }
  template<class T, std::size_t N>
  static bool finite(const std::array<T, N>& values) noexcept {
    for (const auto& value : values) if (!finite(value)) return false;
    return true;
  }
  static bool finite_state(const franka::RobotState& s) noexcept {
    // Numeric fields serialized by installed libfranka 0.20.4's RobotState
    // ostream. Nonfinite packets are discarded+latched, never JSON nan.
#define LEGO_FINITE(member) if (!finite(s.member)) return false
    LEGO_FINITE(O_T_EE); LEGO_FINITE(O_T_EE_d); LEGO_FINITE(F_T_NE);
    LEGO_FINITE(NE_T_EE); LEGO_FINITE(F_T_EE); LEGO_FINITE(EE_T_K);
    LEGO_FINITE(m_ee); LEGO_FINITE(F_x_Cee); LEGO_FINITE(I_ee);
    LEGO_FINITE(m_load); LEGO_FINITE(F_x_Cload); LEGO_FINITE(I_load);
    LEGO_FINITE(m_total); LEGO_FINITE(F_x_Ctotal); LEGO_FINITE(I_total);
    LEGO_FINITE(elbow); LEGO_FINITE(elbow_d); LEGO_FINITE(elbow_c);
    LEGO_FINITE(delbow_c); LEGO_FINITE(ddelbow_c); LEGO_FINITE(tau_J);
    LEGO_FINITE(tau_J_d); LEGO_FINITE(dtau_J); LEGO_FINITE(q); LEGO_FINITE(dq);
    LEGO_FINITE(q_d); LEGO_FINITE(dq_d); LEGO_FINITE(ddq_d);
    LEGO_FINITE(joint_contact); LEGO_FINITE(cartesian_contact);
    LEGO_FINITE(joint_collision); LEGO_FINITE(cartesian_collision);
    LEGO_FINITE(tau_ext_hat_filtered); LEGO_FINITE(O_F_ext_hat_K);
    LEGO_FINITE(K_F_ext_hat_K); LEGO_FINITE(O_dP_EE_d); LEGO_FINITE(O_ddP_O);
    LEGO_FINITE(O_T_EE_c); LEGO_FINITE(O_dP_EE_c); LEGO_FINITE(O_ddP_EE_c);
    LEGO_FINITE(theta); LEGO_FINITE(dtheta); LEGO_FINITE(accelerometer_top);
    LEGO_FINITE(accelerometer_bottom); LEGO_FINITE(control_command_success_rate);
#undef LEGO_FINITE
    return true;
  }
  bool write_all(int fd, const std::string& text, bool trace) noexcept {
    std::size_t offset = 0;
    while (offset < text.size()) {
      const auto count = ::write(fd, text.data()+offset, text.size()-offset);
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0) {
        rt_.disk_errors.fetch_add(1, std::memory_order_relaxed);
        fault(DiskFailure); return false;
      }
      offset += std::size_t(count);
      if (trace) trace_bytes_ += std::uint64_t(count);
    }
    return true;
  }
  std::string header() const {
    const auto& p = metadata_.conditions;
    std::ostringstream out;
    out << std::setprecision(17)
        << "{\"type\":\"owner_recording_header\",\"schema\":\"lego_owner_raw_v1\",\"trial_id\":" << quoted(metadata_.trial_id)
        << ",\"recording_id\":" << quoted(metadata_.recording_id)
        << ",\"provenance\":" << quoted(metadata_.provenance)
        << ",\"episode_origin_monotonic_ns\":" << metadata_.episode_origin_monotonic_ns
        << ",\"profile_revision\":" << metadata_.profile_revision
        << ",\"conditions\":{\"sdk_version\":" << quoted(p.sdk_version)
        << ",\"tool_profile_id\":" << quoted(p.tool_profile_id)
        << ",\"payload_profile_id\":" << quoted(p.payload_profile_id)
        << ",\"impedance_profile_id\":" << quoted(p.impedance_profile_id)
        << ",\"limiter_profile_id\":" << quoted(p.limiter_profile_id)
        << ",\"filter_profile_id\":" << quoted(p.filter_profile_id)
        << ",\"wrench_profile_id\":" << quoted(p.wrench_profile_id)
        << ",\"fixture_grasp_id\":" << quoted(p.fixture_grasp_id)
        << ",\"clock_id\":" << quoted(p.clock_id)
        << ",\"control_period_s\":" << double(p.control_period_ns)/1e9
        << ",\"sample_phase\":\"before_command_application\"},\"phase_names\":{";
    bool first = true;
    for (const auto& phase : metadata_.phase_names) {
      if (!first) out << ',';
      first = false;
      out << quoted(std::to_string(phase.first)) << ':' << quoted(phase.second);
    }
    out << "},\"robot_time_resolution_ns\":1000000,\"wrench_calibration_status\":\"uncalibrated\","
           "\"hardware_deployment_enabled\":false,\"training_ready\":false}\n";
    return out.str();
  }
  std::string row(const Packet& item) const {
    const auto& s = item.state;
    const auto& c = item.context;
    std::ostringstream out;
    out << std::setprecision(17) << std::boolalpha
        << "{\"type\":\"owner_sample\",\"sequence\":" << c.sequence
        << ",\"monotonic_ns\":" << c.monotonic_ns << ",\"robot_time_ms\":" << s.time.toMSec()
        << ",\"condition_profile_revision\":" << c.condition_profile_revision
        << ",\"phase_id\":" << c.phase_id
        << ",\"sample_point\":\"before_command_application\",\"dropped_samples_since_last\":0"
        << ",\"requested_q\":"; array(out, item.requested);
    out << ",\"sent_q\":"; array(out, item.sent);
    out << ",\"desired_q\":"; array(out, s.q_d);
    out << ",\"desired_dq\":"; array(out, s.dq_d);
    out << ",\"desired_ddq\":"; array(out, s.ddq_d);
    out << ",\"measured_q\":"; array(out, s.q);
    out << ",\"measured_dq\":"; array(out, s.dq);
    out << ",\"F_T_EE\":"; array(out, s.F_T_EE);
    out << ",\"EE_T_K\":"; array(out, s.EE_T_K);
    out << ",\"O_T_EE\":"; array(out, s.O_T_EE);
    out << ",\"wrench_K_raw\":"; array(out, s.K_F_ext_hat_K);
    out << ",\"O_F_ext_hat_K_raw\":"; array(out, s.O_F_ext_hat_K);
    out << ",\"estimated_wrench_valid\":" << c.estimated_wrench_valid
        << ",\"wrench_valid\":false,\"wrench_calibration_status\":\"uncalibrated\""
        << ",\"singularity_valid\":" << c.singularity_valid
        << ",\"grasp_valid\":" << c.grasp_valid << ",\"robot_fault\":" << bool(s.current_errors)
        << ",\"gripper_width_m\":";
    if (std::isfinite(c.hand.width_m)) out << c.hand.width_m; else out << "null";
    out << ",\"hand_monotonic_ns\":" << c.hand.monotonic_ns
        << ",\"hand_sequence\":" << c.hand.sequence << ",\"hand_valid\":" << c.hand.valid
        << ",\"raw_state\":" << s << "}\n";
    return out.str();
  }
  static bool healthy(const RecorderHealth& h) noexcept {
    return !h.stop_requested && h.fault_flags == 0 && h.disk_errors == 0
        && h.offered > 0 && h.offered == h.enqueued+h.dropped+h.rejected
        && h.enqueued == h.written && h.dropped == 0 && h.rejected == 0 && h.discarded == 0
        && h.worker_policy == SCHED_OTHER && h.worker_priority == 0;
  }
  bool health_file(bool finalized) noexcept {
    try {
      auto h = snapshot();
      h.finalized = finalized; h.integrity_valid = finalized && healthy(h);
      std::ostringstream out;
      out << std::boolalpha << "{\"schema\":\"lego_owner_health_v1\",\"finalized\":" << finalized
          << ",\"recording_id\":" << quoted(metadata_.recording_id)
          << ",\"trial_id\":" << quoted(metadata_.trial_id)
          << ",\"profile_revision\":" << metadata_.profile_revision
          << ",\"integrity_valid\":" << h.integrity_valid
          << ",\"offered\":" << h.offered << ",\"enqueued\":" << h.enqueued
          << ",\"written\":" << h.written << ",\"dropped\":" << h.dropped
          << ",\"rejected\":" << h.rejected << ",\"discarded\":" << h.discarded
          << ",\"disk_errors\":" << h.disk_errors << ",\"fault_flags\":" << h.fault_flags
          << ",\"stop_requested\":" << h.stop_requested << ",\"queue_high_water\":" << h.queue_high_water
          << ",\"queue_capacity_samples\":" << Capacity-1 << ",\"bytes_written\":" << trace_bytes_
          << ",\"worker_policy\":" << h.worker_policy << ",\"worker_priority\":" << h.worker_priority
          << ",\"worker_explicit_sched_other\":" << (h.worker_policy == SCHED_OTHER && h.worker_priority == 0)
          << ",\"wrench_calibration_status\":\"uncalibrated\",\"physical_evidence_validated\":false,\"training_ready\":false}\n";
      const auto temporary = directory_/"health.json.next";
      const int fd = ::open(temporary.c_str(), O_CREAT|O_EXCL|O_WRONLY|O_CLOEXEC|O_NOFOLLOW, 0600);
      if (fd < 0) { rt_.disk_errors.fetch_add(1); fault(DiskFailure); return false; }
      bool ok = write_all(fd, out.str(), false);
      if (ok && ::fsync(fd)) { rt_.disk_errors.fetch_add(1); fault(DiskFailure); ok = false; }
      if (::close(fd)) { rt_.disk_errors.fetch_add(1); fault(DiskFailure); ok = false; }
      if (ok && ::rename(temporary.c_str(), (directory_/"health.json").c_str())) { rt_.disk_errors.fetch_add(1); fault(DiskFailure); ok = false; }
      if (!ok) ::unlink(temporary.c_str());
      return ok;
    } catch (...) { rt_.disk_errors.fetch_add(1); fault(DiskFailure); return false; }
  }
  static void* trampoline(void* self) noexcept {
    static_cast<OwnerRecorder*>(self)->worker_loop(); return nullptr;
  }
  void worker_loop() noexcept {
    sched_param scheduling{};
    int policy = -1;
    bool ok = pthread_getschedparam(pthread_self(), &policy, &scheduling) == 0
           && policy == SCHED_OTHER && scheduling.sched_priority == 0;
    worker_policy_.store(policy); worker_priority_.store(scheduling.sched_priority);
    if (!ok) fault(WorkerPolicy);
    try { if (ok) ok = write_all(trace_fd_, header(), true) && health_file(false); }
    catch (...) { fault(DiskFailure); rt_.disk_errors.fetch_add(1); ok = false; }
    rt_.startup_ok.store(ok, std::memory_order_release);
    rt_.ready.store(true, std::memory_order_release);
    bool writable = ok;
    for (;;) {
      const auto tail = rt_.tail.load(std::memory_order_relaxed), head = rt_.head.load(std::memory_order_acquire);
      if (tail == head) {
        // Stop publication follows the final producer commit. Recheck head
        // after acquiring stop; the first empty read may precede that commit.
        if (rt_.worker_stop.load(std::memory_order_acquire)
            && tail == rt_.head.load(std::memory_order_acquire)) break;
        std::this_thread::sleep_for(std::chrono::microseconds(200)); continue;
      }
      // In-place consume; producer cannot reuse this slot before tail release.
      const auto& item = rt_.queue[tail];
      if (writable && finite_state(item.state) && finite(item.requested) && finite(item.sent)) {
        try {
          if (write_all(trace_fd_, row(item), true)) rt_.written.fetch_add(1);
          else { writable = false; rt_.discarded.fetch_add(1); }
        } catch (...) { fault(DiskFailure); rt_.disk_errors.fetch_add(1); writable = false; rt_.discarded.fetch_add(1); }
      } else {
        if (writable) fault(NonfiniteState);
        rt_.discarded.fetch_add(1);
      }
      rt_.tail.store((tail+1)%Capacity, std::memory_order_release);
    }
    if (::fsync(trace_fd_)) { rt_.disk_errors.fetch_add(1); fault(DiskFailure); }
    if (::close(trace_fd_)) { rt_.disk_errors.fetch_add(1); fault(DiskFailure); }
    trace_fd_ = -1;
    if (health_file(true)) rt_.finalized.store(true, std::memory_order_release);
  }

 public:
  OwnerRecorder(const std::filesystem::path& new_directory, RecordingMetadata metadata)
      : metadata_(std::move(metadata)), directory_(new_directory) {
    const auto& p = metadata_.conditions;
    if (metadata_.recording_id.empty() || metadata_.trial_id.empty() || (metadata_.provenance != "physical" && metadata_.provenance != "synthetic")
        || metadata_.profile_revision == 0 || metadata_.episode_origin_monotonic_ns == 0 || p.control_period_ns == 0 || p.clock_id.empty()
        || p.sdk_version.empty() || p.tool_profile_id.empty() || p.payload_profile_id.empty()
        || p.impedance_profile_id.empty() || p.limiter_profile_id.empty() || p.filter_profile_id.empty()
        || p.wrench_profile_id.empty() || p.fixture_grasp_id.empty())
      throw std::invalid_argument("explicit immutable owner condition/clock profiles required");
    rt_.profile_revision = metadata_.profile_revision;
    rt_.episode_origin_monotonic_ns = metadata_.episode_origin_monotonic_ns;
    // Resolve the SDK copy/time symbols outside the callback as well as
    // constructing slots; first-use dynamic linking must not happen in offer.
    rt_.queue[0].state = franka::RobotState{};
    volatile auto prewarmed_robot_time = rt_.queue[0].state.time.toMSec();
    (void)prewarmed_robot_time;
    if (!std::filesystem::create_directory(directory_)) throw std::runtime_error("recording directory must be new");
    if (::mlock(&rt_, sizeof(rt_))) throw std::runtime_error("cannot lock recorder callback storage");
    locked_ = true;
    volatile unsigned char* bytes = reinterpret_cast<volatile unsigned char*>(&rt_);
    for (std::size_t i = 0; i < sizeof(rt_); i += std::size_t(::sysconf(_SC_PAGESIZE))) { const auto value = bytes[i]; bytes[i] = value; }
    const auto last = bytes[sizeof(rt_)-1]; bytes[sizeof(rt_)-1] = last;
    try {
      trace_fd_ = ::open((directory_/"trace.ndjson").c_str(), O_CREAT|O_EXCL|O_WRONLY|O_CLOEXEC|O_NOFOLLOW, 0600);
      if (trace_fd_ < 0) throw std::runtime_error("cannot open new recorder trace");
      pthread_attr_t attributes;
      if (pthread_attr_init(&attributes)) throw std::runtime_error("cannot initialize recorder thread attributes");
      sched_param scheduling{};
      const bool configured = pthread_attr_setinheritsched(&attributes, PTHREAD_EXPLICIT_SCHED) == 0
                           && pthread_attr_setschedpolicy(&attributes, SCHED_OTHER) == 0
                           && pthread_attr_setschedparam(&attributes, &scheduling) == 0;
      const int status = configured ? pthread_create(&worker_, &attributes, trampoline, this) : EINVAL;
      pthread_attr_destroy(&attributes);
      if (status) throw std::runtime_error("cannot start explicit ordinary-priority recorder worker");
      started_ = true;
      // A FIFO SDK owner can be pinned to the worker's inherited CPU. Sleeping
      // (not sched_yield) lets an ordinary worker initialize on that CPU.
      while (!rt_.ready.load(std::memory_order_acquire)) std::this_thread::sleep_for(std::chrono::microseconds(100));
      if (!rt_.startup_ok.load(std::memory_order_acquire)) throw std::runtime_error("recorder worker initialization/output failed");
    } catch (...) {
      finish();
      if (trace_fd_ >= 0) { ::close(trace_fd_); trace_fd_ = -1; }
      if (locked_) { ::munlock(&rt_, sizeof(rt_)); locked_ = false; }
      throw;
    }
  }
  // One SDK-owner callback is producer. Sequence counts unthrottled offers,
  // not viewer packets. No allocation, locks, syscalls, clocks or I/O here.
  bool offer(const franka::RobotState& state, const std::array<double, 7>& requested,
             const std::array<double, 7>& sent, const SampleContext& context) noexcept {
    if (rt_.closing.load(std::memory_order_seq_cst)) return false;
    rt_.in_offer.fetch_add(1, std::memory_order_seq_cst);
    if (rt_.closing.load(std::memory_order_seq_cst)) { rt_.in_offer.fetch_sub(1, std::memory_order_seq_cst); return false; }
    rt_.offered.fetch_add(1, std::memory_order_relaxed);
    bool accepted = false;
    if (rt_.stop_request.load(std::memory_order_acquire)) rt_.rejected.fetch_add(1);
    else if (!context.before_command_application || context.condition_profile_revision != rt_.profile_revision
             || context.monotonic_ns < rt_.episode_origin_monotonic_ns || (context.hand.valid &&
                 (!std::isfinite(context.hand.width_m) || context.hand.width_m < 0 || context.hand.monotonic_ns == 0 || context.hand.monotonic_ns > context.monotonic_ns))) {
      rt_.rejected.fetch_add(1); fault(BadContext);
    } else if (rt_.have_first_sample && context.sequence != rt_.last_sequence+1) {
      rt_.rejected.fetch_add(1); fault(BadSequence);
    } else if (rt_.have_first_sample && context.monotonic_ns <= rt_.last_monotonic_ns) {
      rt_.rejected.fetch_add(1); fault(BadMonotonicClock);
    } else if (rt_.have_first_sample && state.time.toMSec() <= rt_.last_robot_time_ms) {
      rt_.rejected.fetch_add(1); fault(BadRobotClock);
    } else {
      rt_.have_first_sample = true;
      rt_.last_sequence = context.sequence; rt_.last_monotonic_ns = context.monotonic_ns;
      rt_.last_robot_time_ms = state.time.toMSec();
      const auto head = rt_.head.load(std::memory_order_relaxed), next = (head+1)%Capacity;
      const auto tail = rt_.tail.load(std::memory_order_acquire);
      if (next == tail) { rt_.dropped.fetch_add(1); fault(Overflow); }
      else {
        auto& item = rt_.queue[head];
        item.state = state; item.requested = requested; item.sent = sent; item.context = context;
        rt_.enqueued.fetch_add(1, std::memory_order_relaxed);
        const auto occupancy = (next+Capacity-tail)%Capacity;
        if (occupancy > rt_.high_water.load(std::memory_order_relaxed)) rt_.high_water.store(occupancy, std::memory_order_relaxed);
        rt_.head.store(next, std::memory_order_release);
        accepted = true;
      }
    }
    rt_.in_offer.fetch_sub(1, std::memory_order_seq_cst);
    return accepted;
  }
  bool stop_requested() const noexcept { return rt_.stop_request.load(std::memory_order_acquire); }
  RecorderHealth snapshot() const noexcept {
    RecorderHealth h;
    h.offered = rt_.offered.load(); h.enqueued = rt_.enqueued.load(); h.written = rt_.written.load();
    h.dropped = rt_.dropped.load(); h.rejected = rt_.rejected.load(); h.discarded = rt_.discarded.load();
    h.disk_errors = rt_.disk_errors.load(); h.queue_high_water = rt_.high_water.load();
    h.fault_flags = rt_.faults.load(); h.stop_requested = rt_.stop_request.load(); h.finalized = rt_.finalized.load();
    h.worker_policy = worker_policy_.load(); h.worker_priority = worker_priority_.load();
    h.integrity_valid = h.finalized && healthy(h);
    return h;
  }
  // Off callback only. Calls after closure return false and are outside this
  // recording window. Producer must never outlive the recorder object.
  RecorderHealth finish() noexcept {
    rt_.closing.store(true, std::memory_order_seq_cst);
    while (rt_.in_offer.load(std::memory_order_seq_cst)) std::this_thread::sleep_for(std::chrono::microseconds(100));
    rt_.worker_stop.store(true, std::memory_order_release);
    if (started_) { pthread_join(worker_, nullptr); started_ = false; }
    return snapshot();
  }
  ~OwnerRecorder() { finish(); if (locked_) ::munlock(&rt_, sizeof(rt_)); }
  OwnerRecorder(const OwnerRecorder&) = delete;
  OwnerRecorder& operator=(const OwnerRecorder&) = delete;
};
}  // namespace lego_seating
