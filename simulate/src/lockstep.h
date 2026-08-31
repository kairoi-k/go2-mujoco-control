#pragma once
// Verification-only sim-time lockstep coordinator (Order-103/105).
//
// Goal: never apply a controller command to a MuJoCo state the command was
// not computed from. Contract:
//   * startup stays identical to the wall-clock runner until the controller's
//     lifecycle boundary (first lowcmd arrival) completes the ready barrier;
//   * explicit handoff: on the first command the physics thread advances
//     exactly one mj_step (T0 -> T0+dt) and the barrier row is recorded at T0;
//   * causal handshake (Order-105): every lockstep LowState carries a
//     monotonic tick side-channel. The controller's verification adapter
//     acks {state_seq, command_seq} after publishing each LowCmd. The
//     physics thread advances exactly one mj_step only when a newer LowCmd
//     arrived AND ack.state_seq equals the currently published state tick
//     AND the ack command_seq is consistent with an arrived command;
//   * stale/future/duplicate/reordered/missing acks, command mismatches and
//     timeouts fail closed with a trace row;
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
  void OnCommandArrived()
  {
    cmd_seq_.fetch_add(1, std::memory_order_relaxed);
  }

  // Order-105 causal handshake: the controller's verification adapter
  // publishes ack{state_seq, command_seq} after each LowCmd write. State
  // sequences are validated against the last published state tick; the ack
  // is held as the pending proof for the exchange of that state. Acks whose
  // command_seq belongs to the wall-clock startup phase (<= the barrier
  // command) reference pre-handoff states and are ignored.
  void OnAckReceived(std::uint64_t state_seq, std::uint64_t command_seq)
  {
    const char *reason = nullptr;
    std::uint32_t violation = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!ack_validation_active_) return; // startup wall-clock phase
      if (command_seq <= barrier_seq_)
        return; // barrier-command ack: startup phase, not an exchange ack
      if (pending_ack_valid_ &&
          pending_ack_state_seq_ == published_state_seq_)
      {
        violation = kViolationAckDuplicate;
        reason = "duplicate ack for published state";
      }
      else if (state_seq < published_state_seq_)
      {
        violation = kViolationAckStale;
        reason = "stale ack for older state";
      }
      else if (state_seq > published_state_seq_)
      {
        violation = kViolationAckFuture;
        reason = "future ack for unpublished state";
      }
      else
      {
        pending_ack_valid_ = true;
        pending_ack_state_seq_ = state_seq;
        pending_ack_cmd_seq_ = command_seq;
      }
    }
    if (reason != nullptr)
    {
      AddViolation(violation);
      FailClosed(AckReason(reason, state_seq, command_seq));
    }
  }

  // ---- startup (bridge thread) ----
  // Call once per startup-phase state publish (wall-clock behavior until the
  // ready barrier). Returns true when the ready barrier completed on this
  // call: the first controller command (computed after the controller's
  // natural-settle + world-reference lifecycle boundary) arrived and the
  // first frozen step is permitted. `sim_tick_ms` is the tick of the state
  // just published; it tracks the last published state for ack validation.
  bool OnStartupPublish(std::uint64_t sim_tick_ms)
  {
    if (barrier_complete_.load(std::memory_order_acquire)) return true;
    bool barrier_timeout = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (barrier_complete_.load(std::memory_order_relaxed)) return true;
      published_state_seq_ = sim_tick_ms;
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

  // Call before publishing each lockstep state with its tick. Fails closed
  // if a stale ack from the previous interval is still pending (the
  // controller wrote more than once from one state).
  void OnStatePublished(std::uint64_t sim_tick_ms)
  {
    bool stale_pending = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ack_validation_active_ = true;
      published_state_seq_ = sim_tick_ms;
      stale_pending = pending_ack_valid_ &&
                      pending_ack_state_seq_ != sim_tick_ms;
    }
    if (stale_pending)
    {
      AddViolation(kViolationAckStale);
      FailClosed("stale ack pending at lockstep state publish");
    }
  }

  // ---- interval protocol (bridge thread) ----
  // Blocks until the causal exchange for the just-published state is
  // complete: a newer controller command arrived AND an ack whose state_seq
  // equals `sim_tick_ms` (the published tick side-channel) and whose
  // command_seq references an arrived command newer than the publish.
  // kReady: the acked command may be applied and physics may step exactly
  // one mj_step. Timeouts and ack anomalies fail closed.
  WaitOutcome WaitForExchange(std::uint64_t sim_tick_ms,
                              ExchangeTrigger *out_trigger)
  {
    const std::int64_t publish_us = NowUs();
    std::uint64_t seq_at_publish = 0;
    std::uint64_t interval_index = 0;
    bool tick_gap = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      seq_at_publish = cmd_seq_.load(std::memory_order_relaxed);
      if (sim_tick_ms != next_expected_tick_)
      {
        AddViolation(kViolationTickGap);
        tick_gap = true;
      }
      next_expected_tick_ = sim_tick_ms + cfg_.dt_ms;
      last_published_tick_ = sim_tick_ms;
      interval_index = interval_index_++;
    }
    if (tick_gap)
    {
      FailClosed("tick sequence gap/duplicate/reorder");
    }
    exchange_active_.store(true, std::memory_order_release);
    struct ActiveGuard
    {
      std::atomic<bool> &flag;
      ~ActiveGuard() { flag.store(false, std::memory_order_release); }
    } active_guard{exchange_active_};

    const auto deadline = Clock::now() + Seconds(cfg_.exchange_timeout_s);
    ExchangeTrigger trigger = ExchangeTrigger::kTimeout;
    bool ack_seen = false;
    std::uint64_t ack_state_seq = 0;
    std::uint64_t ack_cmd_seq = 0;
    for (;;)
    {
      if (AbortRequested()) return WaitOutcome::kAborted;
      if (failed_closed_.load(std::memory_order_acquire))
        return WaitOutcome::kTimeoutFailClosed;
      const char *mismatch_reason = nullptr;
      std::uint32_t mismatch_violation = 0;
      bool complete = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (pending_ack_valid_)
        {
          ack_seen = true;
          ack_state_seq = pending_ack_state_seq_;
          ack_cmd_seq = pending_ack_cmd_seq_;
          if (pending_ack_state_seq_ != sim_tick_ms)
          {
            mismatch_reason =
                "ack state_seq mismatch (stale/future/reorder)";
            mismatch_violation = pending_ack_state_seq_ < sim_tick_ms
                                     ? kViolationAckStale
                                     : kViolationAckFuture;
          }
          else if (pending_ack_cmd_seq_ <= seq_at_publish)
          {
            mismatch_reason =
                "ack command_seq not newer than published state";
            mismatch_violation = kViolationAckCmdMismatch;
          }
          else if (cmd_seq_.load(std::memory_order_relaxed) >=
                   pending_ack_cmd_seq_)
          {
            pending_ack_valid_ = false;
            trigger = ExchangeTrigger::kAckMatched;
            complete = true;
          }
        }
      }
      if (mismatch_reason != nullptr)
      {
        AddViolation(mismatch_violation);
        FailClosed(AckReason(mismatch_reason, ack_state_seq, ack_cmd_seq));
        return WaitOutcome::kTimeoutFailClosed;
      }
      if (complete) break;
      if (Clock::now() >= deadline)
      {
        if (ack_seen)
        {
          AddViolation(kViolationExchangeTimeout);
          FailClosed("exchange timeout waiting for acked controller "
                     "command");
        }
        else
        {
          AddViolation(kViolationAckMissing);
          FailClosed("exchange timeout waiting for controller ack");
        }
        return WaitOutcome::kTimeoutFailClosed;
      }
      SleepUs(200);
    }
    if (out_trigger) *out_trigger = trigger;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      Record(IntervalRecord{
          sim_tick_ms, interval_index, "lockstep", seq_at_publish,
          cmd_seq_.load(std::memory_order_relaxed), NowUs() - publish_us,
          publish_us, trigger,
          violations_.load(std::memory_order_relaxed), ack_state_seq,
          ack_cmd_seq});
    }
    return WaitOutcome::kReady;
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
        last_published_tick_ = pre_tick;
        next_expected_tick_ = sim_tick_ms;
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
    }
    step_completed_.store(true, std::memory_order_release);
  }

  // Bridge side: blocks until the physics thread completed a step.
  WaitOutcome WaitForStepCompleted()
  {
    const auto deadline = Clock::now() + Seconds(cfg_.step_wait_timeout_s);
    for (;;)
    {
      if (AbortRequested()) return WaitOutcome::kAborted;
      if (failed_closed_.load(std::memory_order_acquire))
        return WaitOutcome::kTimeoutFailClosed;
      if (step_completed_.exchange(false, std::memory_order_acq_rel))
      {
        return WaitOutcome::kReady;
      }
      if (Clock::now() >= deadline)
      {
        AddViolation(kViolationStepTimeout);
        FailClosed("step completion timeout");
        return WaitOutcome::kTimeoutFailClosed;
      }
      SleepUs(200);
    }
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

  // True while WaitForExchange has captured the publish-time command count
  // and is waiting for the causal exchange (used by tests to inject events
  // deterministically).
  bool ExchangeActive() const
  {
    return exchange_active_.load(std::memory_order_acquire);
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

  // Formats a fail-closed reason with the offending ack's sequences. The
  // ack callback is a single DDS thread, so thread-local storage suffices.
  static const char *AckReason(const char *reason, std::uint64_t state_seq,
                               std::uint64_t command_seq)
  {
    thread_local char buf[160];
    std::snprintf(buf, sizeof(buf), "%s state_seq=%llu command_seq=%llu",
                  reason,
                  static_cast<unsigned long long>(state_seq),
                  static_cast<unsigned long long>(command_seq));
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
  std::atomic<bool> exchange_active_{false};
  std::function<bool()> abort_cb_;
  std::function<void()> fail_closed_handler_;
  mutable std::mutex mutex_;
  std::ofstream trace_;
  bool trace_ok_ = false;
  bool trace_closed_ = false;
  std::uint64_t next_expected_tick_ = 0;
  std::uint64_t last_published_tick_ = 0;
  std::uint64_t interval_index_ = 0;
  std::uint64_t barrier_seq_ = 0;
  std::int64_t startup_begin_us_ = 0;
  bool ack_validation_active_ = false;
  bool pending_ack_valid_ = false;
  std::uint64_t pending_ack_state_seq_ = 0;
  std::uint64_t pending_ack_cmd_seq_ = 0;
  std::uint64_t published_state_seq_ = 0;
};

} // namespace lockstep
