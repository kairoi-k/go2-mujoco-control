#pragma once
// Verification-only sim-time lockstep coordinator (Order-103/105/106).
//
// Goal: never apply a controller command to a MuJoCo state the command was
// not computed from. Contract:
//   * startup stays identical to the wall-clock runner until the controller's
//     lifecycle boundary (first lowcmd arrival) completes the ready barrier;
//   * explicit handoff: on the first command the physics thread advances
//     exactly one mj_step (T0 -> T0+dt) and the barrier row is recorded at T0;
//   * frozen republish (Order-106): while a lockstep exchange is open the
//     bridge keeps publishing the same immutable physics state at 1000 Hz.
//     Every publish of one frozen state carries the SAME state_seq (its
//     monotonic tick), so a 500 Hz controller always has a fresh observation
//     and repeatedly sees the identical tick;
//   * causal exchange (Order-106): the physics thread advances exactly one
//     mj_step only when BOTH (a) a new LowCmd arrived after the FIRST
//     publish of the frozen state (the sim-local arrival counter proves it)
//     and (b) a matching ack{state_seq} for that state was received. The
//     ack carries ONLY the full-width state_seq (unitree Error_.source() is
//     uint32_t; LowState.tick is uint32_t; at 1 kHz that wraps at 2^32 ms,
//     i.e. ~49.7 days). The command sequence is NOT carried: the sim counts
//     LowCmd arrivals locally, so no cross-topic sequence matching and no
//     width/wrap proof for a command_seq side-channel is needed;
//   * duplicate acks are idempotent: the controller may write 1-2 LowCmds
//     (500 Hz) per frozen state and ack it more than once. Acks on the
//     rt/lockstep/ack topic from the single controller publisher are
//     delivered in DDS writer order, so a late ack for the just-consumed
//     state always arrives while the next exchange is still open and is
//     tolerated. TRUE stale (older than the last consumed state) and future
//     (newer than the published state) acks, plus missing acks/commands and
//     timeouts, fail closed with a trace row;
//   * the side-channel is sequence metadata only (no truth/geometry); with
//     the flag off this header has no effect on the wall-clock runner.
//
// The coordinator is DDS/MuJoCo-free so the ready/exchange/tick/ack
// invariants are unit-testable (see simulate/src/tests/test_lockstep.cpp).

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace lockstep
{

constexpr std::uint32_t kViolationBarrierTimeout = 1u << 0;
constexpr std::uint32_t kViolationExchangeTimeout = 1u << 1;
constexpr std::uint32_t kViolationStepTimeout = 1u << 2;
constexpr std::uint32_t kViolationTickGap = 1u << 3;
constexpr std::uint32_t kViolationStepTick = 1u << 4;
constexpr std::uint32_t kViolationAckStale = 1u << 5;
constexpr std::uint32_t kViolationAckFuture = 1u << 6;
// Order-106: duplicate acks are idempotent and no longer set this flag; the
// constants below are kept for trace/API value stability.
constexpr std::uint32_t kViolationAckDuplicate = 1u << 7;
constexpr std::uint32_t kViolationAckCmdMismatch = 1u << 8;
constexpr std::uint32_t kViolationAckMissing = 1u << 9;

enum class WaitOutcome : int
{
  kReady = 0,
  kAborted = 1,
  kTimeoutFailClosed = 2,
};

enum class ExchangeTrigger : int
{
  kBarrier = 0,
  kArrivalCount = 1,
  kTimeout = 2,
  kAckMatched = 3,
};

// Per-publish outcome returned by OnPublish (bridge role). kStepGranted
// means the frozen-state exchange just completed: the caller may apply the
// latest LowCmd and call NotifyCommandApplied() so physics steps exactly one
// mj_step. kIdle means the freeze continues (republish the same state).
enum class PublishOutcome : int
{
  kIdle = 0,
  kStepGranted = 1,
};

struct IntervalRecord
{
  std::uint64_t sim_tick_ms = 0;
  std::uint64_t step_index = 0;
  const char *phase = "";
  std::uint64_t cmd_seq_at_publish = 0;
  std::uint64_t cmd_seq_at_ready = 0;
  std::int64_t exchange_wait_us = 0;
  std::int64_t publish_wall_us = 0;
  ExchangeTrigger trigger = ExchangeTrigger::kBarrier;
  std::uint32_t violations = 0;
  std::uint64_t ack_state_seq = 0;
  std::uint64_t ack_cmd_seq = 0;
};

class Coordinator
{
public:
  struct Config
  {
    double barrier_timeout_s = 60.0;
    double exchange_timeout_s = 5.0;
    double step_wait_timeout_s = 5.0;
    std::uint64_t dt_ms = 2; // frozen physics interval (m->opt.timestep)
    std::string trace_path;
  };

  explicit Coordinator(Config cfg) : cfg_(std::move(cfg))
  {
    if (cfg_.barrier_timeout_s <= 0.0) cfg_.barrier_timeout_s = 60.0;
    if (cfg_.exchange_timeout_s <= 0.0) cfg_.exchange_timeout_s = 5.0;
    if (cfg_.step_wait_timeout_s <= 0.0) cfg_.step_wait_timeout_s = 5.0;
    if (cfg_.dt_ms == 0) cfg_.dt_ms = 2;
    if (!cfg_.trace_path.empty())
    {
      trace_.open(cfg_.trace_path, std::ios::out | std::ios::trunc);
      trace_ok_ = trace_.good();
      if (trace_ok_)
      {
        trace_ << "sim_tick_ms,step_index,phase,cmd_seq_at_publish,"
                  "cmd_seq_at_ready,exchange_wait_us,publish_wall_us,"
                  "exchange_trigger,violations,ack_state_seq,ack_cmd_seq\n";
      }
    }
  }

  ~Coordinator()
  {
    WriteSummary();
  }

  // ---- event feed (DDS subscriber callbacks) ----
  // Sim-local LowCmd arrival count. The exchange rule only needs "a newer
  // LowCmd arrived after the first publish of the frozen state"; the count
  // is never compared against a controller-side sequence, so no cross-topic
  // sequence matching or width/wrap proof is required.
  void OnCommandArrived()
  {
    cmd_seq_.fetch_add(1, std::memory_order_relaxed);
  }

  // Order-106 causal handshake: the controller adapter acks {state_seq}
  // (full-width Error_.source()) after each LowCmd write. Validation against
  // the frozen-state bookkeeping:
  //   * startup-phase acks (state_seq < the barrier handoff state) are
  //     ignored: they reference pre-handoff wall-clock states;
  //   * future (state_seq > published) and true-stale (state_seq < the last
  //     consumed state) acks fail closed;
  //   * a matching ack{state_seq == published} arms the exchange (duplicates
  //     are idempotent: the controller may write 1-2 LowCmds per frozen
  //     state and ack more than once);
  //   * a late ack{state_seq == last consumed} (the controller was still
  //     computing on the just-frozen state when the sim advanced) is
  //     tolerated; per-topic DDS writer order guarantees it arrives while
  //     the next exchange is open, so it can never complete a stale state.
  void OnAckReceived(std::uint64_t state_seq)
  {
    const char *reason = nullptr;
    std::uint32_t violation = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!ack_validation_active_) return; // startup wall-clock phase
      if (state_seq < barrier_state_seq_)
        return; // startup-phase ack (pre-handoff state): not an exchange ack
      if (state_seq > published_state_seq_)
      {
        violation = kViolationAckFuture;
        reason = "future ack for unpublished state";
      }
      else if (state_seq < last_consumed_state_seq_)
      {
        violation = kViolationAckStale;
        reason = "stale ack older than the last consumed state";
      }
      else if (state_seq == published_state_seq_ && exchange_open_)
      {
        matching_ack_seen_ = true; // duplicate acks for the frozen state are idempotent
      }
      // state_seq == last_consumed_state_seq_ (or a closed-exchange matching
      // state): late/duplicate ack for the just-consumed frozen state;
      // idempotent and unable to complete the current exchange.
    }
    if (reason != nullptr)
    {
      AddViolation(violation);
      FailClosed(AckReason(reason, state_seq));
    }
  }

  // ---- startup (bridge thread) ----
  // Call once per startup-phase state publish (wall-clock behavior until the
  // ready barrier). Returns true when the ready barrier completed on this
  // call: the first controller command (computed after the controller's
  // natural-settle + world-reference lifecycle boundary) arrived and the
  // first frozen step is permitted. `sim_tick_ms` is the tick of the state
  // just published; it becomes the barrier handoff state (startup-ack filter
  // and initial last-consumed state).
  bool OnStartupPublish(std::uint64_t sim_tick_ms)
  {
    if (barrier_complete_.load(std::memory_order_acquire)) return true;
    bool barrier_timeout = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (barrier_complete_.load(std::memory_order_relaxed)) return true;
      published_state_seq_ = sim_tick_ms;
      last_published_tick_ = sim_tick_ms;
      if (startup_begin_us_ == 0) startup_begin_us_ = NowUs();
      const std::uint64_t seq = cmd_seq_.load(std::memory_order_relaxed);
      if (seq < 1)
      {
        const std::int64_t elapsed_us = NowUs() - startup_begin_us_;
        if (static_cast<double>(elapsed_us) * 1e-6 > cfg_.barrier_timeout_s)
        {
          AddViolation(kViolationBarrierTimeout);
          barrier_timeout = true;
        }
        else
        {
          return false;
        }
      }
      else
      {
        barrier_complete_.store(true, std::memory_order_release);
        barrier_seq_ = seq;
        barrier_state_seq_ = sim_tick_ms;
        last_consumed_state_seq_ = sim_tick_ms;
      }
    }
    if (barrier_timeout)
    {
      FailClosed("ready barrier timeout waiting for first controller "
                 "command");
      return false;
    }
    step_permitted_.store(true, std::memory_order_release);
    return true;
  }

  bool BarrierComplete() const
  {
    return barrier_complete_.load(std::memory_order_acquire);
  }

  // ---- interval protocol (bridge thread, one call per 1000 Hz publish) ----
  // Registers the publish of the frozen physics state at `sim_tick_ms` and
  // evaluates the causal exchange:
  //   * if the physics thread completed a step since the last call, this is
  //     the FIRST publish of a new state: the exchange opens, the publish
  //     time and the local LowCmd count at first publish are recorded, and
  //     the tick must equal last_published + dt (gap/duplicate/reorder fail
  //     closed);
  //   * otherwise it is a 1000 Hz republish of the same frozen state: the
  //     exchange state is unchanged and the same state_seq is published;
  //   * the exchange completes (kStepGranted) when a matching ack was seen
  //     AND the local LowCmd count exceeds the first-publish count (a new
  //     command arrived after the controller first saw this state);
  //   * missing ack/command and timeouts fail closed.
  // The bridge serializes publishes with the physics step (sim mutex), and
  // NotifyStepCompleted runs inside the physics lock, so `sim_tick_ms` and
  // the consumed step_completed_ flag always refer to the same state.
  PublishOutcome OnPublish(std::uint64_t sim_tick_ms)
  {
    if (failed_closed_.load(std::memory_order_acquire))
      return PublishOutcome::kIdle;
    const char *fail_reason = nullptr;
    std::uint32_t fail_violation = 0;
    PublishOutcome outcome = PublishOutcome::kIdle;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ack_validation_active_ = true;
      if (step_completed_.exchange(false, std::memory_order_acq_rel))
      {
        if (sim_tick_ms != pending_step_tick_)
        {
          fail_violation = kViolationStepTick;
          fail_reason = "publish tick mismatch with physics step";
        }
        else if (sim_tick_ms != last_published_tick_ + cfg_.dt_ms)
        {
          fail_violation = kViolationTickGap;
          fail_reason = "tick sequence gap/duplicate/reorder";
        }
        else
        {
          published_state_seq_ = sim_tick_ms;
          last_published_tick_ = sim_tick_ms;
          first_publish_seq_ = cmd_seq_.load(std::memory_order_relaxed);
          first_publish_wall_us_ = NowUs();
          matching_ack_seen_ = false;
          exchange_open_ = true;
        }
      }
      else if (sim_tick_ms != last_published_tick_)
      {
        fail_violation = kViolationTickGap;
        fail_reason = "republish tick mismatch with frozen state";
      }
      if (fail_reason == nullptr && exchange_open_)
      {
        if (matching_ack_seen_ &&
            cmd_seq_.load(std::memory_order_relaxed) > first_publish_seq_)
        {
          exchange_open_ = false;
          last_consumed_state_seq_ = published_state_seq_;
          Record(IntervalRecord{
              published_state_seq_, interval_index_, "lockstep",
              first_publish_seq_,
              cmd_seq_.load(std::memory_order_relaxed),
              NowUs() - first_publish_wall_us_, first_publish_wall_us_,
              ExchangeTrigger::kAckMatched,
              violations_.load(std::memory_order_relaxed),
              published_state_seq_, 0});
          ++interval_index_;
          outcome = PublishOutcome::kStepGranted;
        }
        else if (NowUs() - first_publish_wall_us_ >
                 static_cast<std::int64_t>(cfg_.exchange_timeout_s * 1e6))
        {
          if (matching_ack_seen_)
          {
            fail_violation = kViolationExchangeTimeout;
            fail_reason = "exchange timeout waiting for new LowCmd after "
                          "state publish";
          }
          else
          {
            fail_violation = kViolationAckMissing;
            fail_reason = "exchange timeout waiting for controller ack";
          }
        }
      }
    }
    if (fail_reason != nullptr)
    {
      AddViolation(fail_violation);
      FailClosed(fail_reason);
    }
    return outcome;
  }

  // ---- interval protocol (physics thread) ----
  WaitOutcome WaitForStepPermission()
  {
    const bool waiting_for_barrier =
        !barrier_complete_.load(std::memory_order_acquire);
    const double timeout_s = waiting_for_barrier ? cfg_.barrier_timeout_s
                                                 : cfg_.step_wait_timeout_s;
    const auto deadline = Clock::now() + Seconds(timeout_s);
    for (;;)
    {
      if (AbortRequested()) return WaitOutcome::kAborted;
      if (failed_closed_.load(std::memory_order_acquire))
        return WaitOutcome::kTimeoutFailClosed;
      if (barrier_complete_.load(std::memory_order_acquire) &&
          step_permitted_.exchange(false, std::memory_order_acq_rel))
      {
        return WaitOutcome::kReady;
      }
      if (Clock::now() >= deadline)
      {
        if (waiting_for_barrier)
        {
          AddViolation(kViolationBarrierTimeout);
          FailClosed("ready barrier timeout waiting for first controller "
                     "command");
        }
        else
        {
          AddViolation(kViolationStepTimeout);
          FailClosed("step permission timeout");
        }
        return WaitOutcome::kTimeoutFailClosed;
      }
      SleepUs(200);
    }
  }

  // Call after exactly one mj_step; `sim_tick_ms` is the new sim tick. The
  // first call is the explicit lockstep handoff: it records the barrier row
  // at the pre-step tick (sim_tick_ms - dt) with the barrier command
  // sequence, so the trace is continuous with the wall-clock startup.
  // Must run under the sim mutex so the bridge's publish tick and this
  // notification always refer to the same frozen state.
  void NotifyStepCompleted(std::uint64_t sim_tick_ms)
  {
    bool step_tick_bad = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (interval_index_ == 0)
      {
        // Handoff: barrier row at the tick the first frozen step started
        // from (the wall-clock phase advanced exactly dt per mj_step).
        const std::uint64_t pre_tick =
            sim_tick_ms > cfg_.dt_ms ? sim_tick_ms - cfg_.dt_ms : 0;
        Record(IntervalRecord{
            pre_tick, 0, "barrier", 0, barrier_seq_, 0, NowUs(),
            ExchangeTrigger::kBarrier,
            violations_.load(std::memory_order_relaxed), 0, 0});
        interval_index_ = 1;
      }
      if (sim_tick_ms != last_published_tick_ + cfg_.dt_ms)
      {
        AddViolation(kViolationStepTick);
        step_tick_bad = true;
      }
    }
    if (step_tick_bad)
    {
      FailClosed("physics step tick mismatch");
      return;
    }
    pending_step_tick_ = sim_tick_ms;
    step_completed_.store(true, std::memory_order_release);
  }

  // ---- query / trace ----
  void NotifyCommandApplied()
  {
    step_permitted_.store(true, std::memory_order_release);
  }

  bool FailedClosed() const
  {
    return failed_closed_.load(std::memory_order_acquire);
  }

  std::uint32_t ViolationCount() const
  {
    return violations_.load(std::memory_order_relaxed);
  }

  std::uint64_t IntervalCount() const
  {
    return interval_count_.load(std::memory_order_relaxed);
  }

  void SetAbortCallback(std::function<bool()> cb)
  {
    abort_cb_ = std::move(cb);
  }

  // Tests replace the default process-exit behavior so timeouts can be
  // observed in-process.
  void SetFailClosedHandler(std::function<void()> handler)
  {
    fail_closed_handler_ = std::move(handler);
  }

  void SetDtMs(std::uint64_t dt_ms)
  {
    if (dt_ms == 0) return;
    cfg_.dt_ms = dt_ms;
  }

  std::uint64_t DtMs() const
  {
    return cfg_.dt_ms;
  }

  bool TraceOk() const
  {
    return trace_ok_;
  }

  void WriteSummary()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (trace_closed_) return;
    trace_closed_ = true;
    if (trace_ok_ && trace_.is_open())
    {
      trace_ << "#summary intervals=" << interval_count_.load(
                  std::memory_order_relaxed)
             << " violations=" << violations_.load(std::memory_order_relaxed)
             << " fail_closed="
             << (failed_closed_.load(std::memory_order_relaxed) ? 1 : 0)
             << " dt_ms=" << cfg_.dt_ms << "\n";
      trace_.flush();
      trace_.close();
    }
  }

private:
  using Clock = std::chrono::steady_clock;
  using Seconds = std::chrono::duration<double>;

  static std::int64_t NowUs()
  {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               Clock::now().time_since_epoch())
        .count();
  }

  static void SleepUs(int us)
  {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
  }

  bool AbortRequested() const
  {
    if (abort_cb_) return abort_cb_();
    return false;
  }

  void AddViolation(std::uint32_t flag)
  {
    violations_.fetch_or(flag, std::memory_order_relaxed);
  }

  // Formats a fail-closed reason with the offending ack's state_seq. The
  // ack callback is a single DDS thread, so thread-local storage suffices.
  static const char *AckReason(const char *reason, std::uint64_t state_seq)
  {
    thread_local char buf[160];
    std::snprintf(buf, sizeof(buf), "%s state_seq=%llu", reason,
                  static_cast<unsigned long long>(state_seq));
    return buf;
  }

  void Record(const IntervalRecord &record)
  {
    interval_count_.fetch_add(1, std::memory_order_relaxed);
    if (!trace_ok_ || !trace_.is_open()) return;
    trace_ << record.sim_tick_ms << "," << record.step_index << ","
           << record.phase << "," << record.cmd_seq_at_publish << ","
           << record.cmd_seq_at_ready << "," << record.exchange_wait_us << ","
           << record.publish_wall_us << ","
           << static_cast<int>(record.trigger) << "," << record.violations
           << "," << record.ack_state_seq << "," << record.ack_cmd_seq
           << "\n";
    trace_.flush();
  }

  // Marks the run failed and, unless a test handler replaced the exit
  // behavior, terminates the process. Callers must return
  // kTimeoutFailClosed after this returns (test path only).
  void FailClosed(const char *reason)
  {
    if (failed_closed_.exchange(true, std::memory_order_acq_rel)) return;
    std::fprintf(stderr, "SIM_LOCKSTEP_FAIL_CLOSED reason=%s violations=%u\n",
                 reason, violations_.load(std::memory_order_relaxed));
    std::fflush(stderr);
    WriteSummary();
    if (fail_closed_handler_)
    {
      fail_closed_handler_();
      return;
    }
    std::_Exit(EXIT_FAILURE);
  }

  Config cfg_;
  std::atomic<std::uint64_t> cmd_seq_{0};
  std::atomic<bool> barrier_complete_{false};
  std::atomic<bool> step_permitted_{false};
  std::atomic<bool> step_completed_{false};
  std::atomic<bool> failed_closed_{false};
  std::atomic<std::uint32_t> violations_{0};
  std::atomic<std::uint64_t> interval_count_{0};
  std::function<bool()> abort_cb_;
  std::function<void()> fail_closed_handler_;
  mutable std::mutex mutex_;
  std::ofstream trace_;
  bool trace_ok_ = false;
  bool trace_closed_ = false;
  std::uint64_t last_published_tick_ = 0;
  std::uint64_t interval_index_ = 0;
  std::uint64_t barrier_seq_ = 0;
  std::uint64_t barrier_state_seq_ = 0;
  std::int64_t startup_begin_us_ = 0;
  bool ack_validation_active_ = false;
  bool exchange_open_ = false;
  bool matching_ack_seen_ = false;
  std::uint64_t first_publish_seq_ = 0;
  std::int64_t first_publish_wall_us_ = 0;
  std::uint64_t published_state_seq_ = 0;
  std::uint64_t last_consumed_state_seq_ = 0;
  std::uint64_t pending_step_tick_ = 0;
};

} // namespace lockstep
