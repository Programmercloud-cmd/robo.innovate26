// Synthetic RobotState copying/file tests only. No SDK connection objects.
#include "owner_recorder.hpp"
#include <csignal>
#include <fstream>
#include <iostream>
#include <sys/resource.h>
#include <sys/wait.h>

using namespace lego_seating;

RecordingMetadata metadata(const std::string& trial) {
  RecordingMetadata m;
  m.recording_id = "synthetic:"+trial+":"+std::to_string(getpid())+":"
      +std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  m.trial_id = trial; m.provenance = "synthetic"; m.profile_revision = 1;
  m.episode_origin_monotonic_ns = 1000000000;
  m.conditions = {"synthetic-sdk", "synthetic-tool", "synthetic-payload", "synthetic-impedance",
                  "synthetic-limiter", "synthetic-filter", "synthetic-raw-wrench",
                  "synthetic-fixed-grasp", "synthetic-virtual-clock", 1000000};
  m.phase_names = {{0, "hold"}, {1, "insertion"}, {2, "withdrawal"}};
  return m;
}

franka::RobotState state(std::uint64_t sequence) {
  franka::RobotState s{};
  for (auto* pose : {&s.F_T_EE, &s.EE_T_K, &s.O_T_EE, &s.O_T_EE_d, &s.F_T_NE, &s.NE_T_EE, &s.O_T_EE_c}) {
    *pose = {}; for (int i : {0, 5, 10, 15}) (*pose)[i] = 1.;
  }
  s.time = franka::Duration(1000+sequence);
  s.robot_mode = franka::RobotMode::kMove;
  s.m_ee = s.m_total = .5;
  for (int i = 0; i < 7; ++i) {
    s.q[i] = .001*double(sequence)+.01*i;
    s.q_d[i] = s.q[i]+.0001; s.dq[i] = .1;
    s.dq_d[i] = .2; s.ddq_d[i] = .3;
    s.tau_J[i] = i+.5; s.tau_ext_hat_filtered[i] = i+.25;
  }
  s.K_F_ext_hat_K = {1.,2.,3.,.1,.2,.3};
  s.O_F_ext_hat_K = {4.,5.,6.,.4,.5,.6};
  s.control_command_success_rate = 1.;
  return s;
}

SampleContext context(std::uint64_t sequence) {
  SampleContext c;
  c.sequence = sequence; c.monotonic_ns = 1000000000+sequence*1000000;
  c.condition_profile_revision = 1; c.phase_id = 1;
  c.estimated_wrench_valid = c.singularity_valid = c.grasp_valid = true;
  c.hand = {.02, c.monotonic_ns, sequence, true};
  return c;
}

void require(bool condition, const char* why) { if (!condition) throw std::runtime_error(why); }
void conserved(const RecorderHealth& h) {
  require(h.offered == h.enqueued+h.dropped+h.rejected, "offer accounting");
  require(h.enqueued == h.written+h.discarded, "write accounting");
}

void normal(const std::filesystem::path& root) {
  OwnerRecorder<64> recorder(root/"normal", metadata("normal"));
  for (std::uint64_t i = 0; i < 20; ++i) {
    const auto s = state(i);
    auto requested = s.q_d, sent = s.q_d; requested[0] += .0002; sent[0] += .0001;
    require(recorder.offer(s, requested, sent, context(i)), "normal offer rejected");
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  const auto h = recorder.finish();
  conserved(h);
  require(h.finalized && h.integrity_valid && h.written == 20, "normal recording invalid");
  require(h.worker_policy == SCHED_OTHER && h.worker_priority == 0, "worker inherited RT policy");
}

void overflow(const std::filesystem::path& root) {
  OwnerRecorder<2> recorder(root/"overflow", metadata("overflow"));
  for (std::uint64_t i = 0; i < 100000 && !recorder.stop_requested(); ++i) {
    const auto s = state(i);
    recorder.offer(s, s.q_d, s.q_d, context(i));
  }
  const auto h = recorder.finish();
  conserved(h);
  require(h.dropped > 0 && h.stop_requested && !h.integrity_valid, "overflow was not explicit");
}

void declared_origin(const std::filesystem::path& root) {
  OwnerRecorder<64> recorder(root/"delayed-first", metadata("delayed-first"));
  for (std::uint64_t i = 0; i < 5; ++i) {
    const auto s = state(i);
    auto c = context(i);
    c.monotonic_ns += 5000000; c.hand.monotonic_ns += 5000000;
    require(recorder.offer(s,s.q_d,s.q_d,c), "delayed sample rejected");
  }
  const auto h = recorder.finish();
  conserved(h);
  require(h.integrity_valid && h.written == 5, "explicit origin recording failed");
}

void bad_sequence(const std::filesystem::path& root) {
  OwnerRecorder<64> recorder(root/"bad-sequence", metadata("bad-sequence"));
  const auto s = state(0);
  require(recorder.offer(s,s.q_d,s.q_d,context(0)), "first sequence rejected");
  require(!recorder.offer(state(1),s.q_d,s.q_d,context(0)), "replayed sequence accepted");
  const auto h = recorder.finish();
  conserved(h);
  require(h.rejected == 1 && h.stop_requested && !h.integrity_valid, "sequence failure was not explicit");
}

void nonfinite(const std::filesystem::path& root) {
  OwnerRecorder<64> recorder(root/"nonfinite", metadata("nonfinite"));
  auto s = state(0); s.K_F_ext_hat_K[0] = std::numeric_limits<double>::quiet_NaN();
  require(recorder.offer(s,s.q_d,s.q_d,context(0)), "nonfinite fixture not queued");
  const auto h = recorder.finish();
  conserved(h);
  require(h.discarded == 1 && h.stop_requested && !h.integrity_valid, "NaN packet was not explicitly rejected");
}

void disk_failure(const std::filesystem::path& root) {
  // Natural EFBIG in an isolated child; no production writer-test controls.
  const auto child = fork();
  if (child < 0) throw std::runtime_error("fork failed");
  if (child == 0) {
    try {
      std::signal(SIGXFSZ, SIG_IGN);
      rlimit limit{};
      if (getrlimit(RLIMIT_FSIZE,&limit)) _exit(3);
      limit.rlim_cur = 8192;
      if (setrlimit(RLIMIT_FSIZE,&limit)) _exit(3);
      OwnerRecorder<64> recorder(root/"disk-failure", metadata("disk-failure"));
      for (std::uint64_t i = 0; i < 100 && !recorder.stop_requested(); ++i) {
        const auto s = state(i);
        recorder.offer(s,s.q_d,s.q_d,context(i));
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      const auto h = recorder.finish();
      conserved(h);
      _exit(h.disk_errors > 0 && h.stop_requested && !h.integrity_valid ? 0 : 2);
    } catch (...) { _exit(4); }
  }
  int status = 0;
  if (waitpid(child,&status,0) != child || !WIFEXITED(status) || WEXITSTATUS(status) != 0)
    throw std::runtime_error("disk failure not correctly latched");
}

int main(int argc, char** argv) {
  if (argc != 2) { std::cerr << "Usage: owner_recorder_synthetic NEW_OUTPUT_DIRECTORY\n"; return 2; }
  try {
    const std::filesystem::path root(argv[1]);
    if (!std::filesystem::create_directory(root)) throw std::runtime_error("test output must be new");
    normal(root); declared_origin(root); overflow(root); bad_sequence(root); nonfinite(root); disk_failure(root);
    std::ofstream report(root/"synthetic-tests.json");
    report << "{\"provenance\":\"synthetic\",\"tests_passed\":6,\"hardware_connections\":0,"
              "\"training_ready\":false,\"test_cases\":[\"full_state\",\"declared_origin\",\"overflow\",\"sequence\",\"nonfinite\",\"disk_failure\"]}\n";
    require(bool(report), "cannot save synthetic test result");
    std::cout << "{\"synthetic_tests_passed\":6,\"hardware_connections\":0,\"training_ready\":false}\n";
    return 0;
  } catch (const std::exception& error) { std::cerr << "Recoverable stop: " << error.what() << '\n'; return 2; }
}
