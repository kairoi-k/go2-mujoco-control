#pragma once
// Verification-only sim-time lockstep coordinator (Order-103, C-006c).
//
// The wall-clock runner lets the MuJoCo physics thread free-run while the
// DDS lowstate/lowcmd exchange races, so a controller command can be applied
// to a state it was not computed from (the Order-102 baseline roll). When
// enabled this coordinator makes the exchange deterministic:
//
//   * startup stays byte-identical to the wall-clock runner: physics
//     free-runs and the bridge publishes at 1000 Hz until the controller's
//     existing lifecycle boundary — natural settle + world reference
//     capture, observable as the first lowcmd arrival — is confirmed on both
//     sides. Startup safety/lifecycle failures (settle timeout, posture,
//     etc.) remain authoritative failures and are never skipped;
//   * explicit handoff: on the first controller command the ready barrier
//     completes; the physics thread then advances exactly one mj_step
//     (tick T0 -> T0+dt) and the coordinator records the barrier row at T0
//     with the command sequence. The first lockstep state publish is T0+dt,
//     so the tick sequence is continuous across the handoff (no duplicate /
//     missing tick);
//   * frozen intervals: the bridge publishes one lowstate per completed
//     physics step and then waits for the next controller command. The step
//     plus publish completes well inside one controller write period, so the
//     command that arrives after the publish was computed from the
//     just-published state (loopback DDS delivery is <1 ms, the controller
//     writes every 2 ms). Controller clock and sim clock stay 1:1;
//   * the physics thread advances exactly one mj_step only after the
//     exchange is complete; the next state is published only after that step;
//   * timeouts fail closed; every interval is recorded with sim_tick, step
//     index, command sequence numbers, wall timestamps, wait latency and
//     violation flags.
//
// The coordinator is DDS/MuJoCo-free so the ready/exchange/tick invariants
// are unit-testable (see simulate/src/tests/test_lockstep.cpp). The bridge
// and physics loop in the simulator feed it events; the controller is never
// modified and never reads simulator truth.

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
                  "exchange_trigger,violations\n";
      }
    }
  }

  ~Coordinator()
  {
    WriteSummary();
  }

  // ---- event feed (lowcmd DDS subscriber callback) ----
  void OnCommandArrived()
  {
    cmd_seq_.fetch_add(1, std::memory_order_relaxed);
  }

  // ---- startup (bridge thread) ----
  // Call once per startup-phase state publish (wall-clock behavior until the
  // ready barrier). Returns true when the ready barrier completed on this
  // call: the first controller command (computed after the controller's
  // natural-settle + world-reference lifecycle boundary) arrived and the
  // first frozen step is permitted. The barrier row's tick is recorded by
  // the first NotifyStepCompleted so it matches the actual pre-step tick.
  bool OnStartupPublish()
  {
    if (barrier_complete_.load(std::memory_order_acquire)) return true;
    bool barrier_timeout = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (barrier_complete_.load(std::memory_order_relaxed)) return true;
      if (startup_begin_us_ == 0) startup_begin_us_ = NowUs();
      const std::uint64_t seq = cmd_seq_.load(std::memory_order_relaxed);
      if (seq < 1)
      {
        // Startup watchdog: the controller must begin commanding (its
        // settle/world-reference lifecycle completes) within the barrier
        // timeout or the run fails closed. Startup safety/lifecycle
        // failures on the controller side are still the authoritative
        // harness failures; this bounds the sim-side wait.
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
      // FailClosed must run outside the mutex (WriteSummary re-locks).
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

  // ---- interval protocol (bridge thread) ----
  // Blocks until the exchange for the just-published state is complete.
  // Called immediately after the bridge published lowstate `sim_tick_ms`.
  // kReady: the latest received command may be applied and physics may
  // step. kAborted: external stop requested. Timeouts fail closed.
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

    const auto deadline = Clock::now() + Seconds(cfg_.exchange_timeout_s);
    ExchangeTrigger trigger = ExchangeTrigger::kTimeout;
    for (;;)
    {
      if (AbortRequested()) return WaitOutcome::kAborted;
      // One fresh command per frozen interval keeps the controller clock and
      // the sim clock 1:1 (both 2 ms). The step+publish completes within one
      // controller write period, so this arrival is guaranteed to have been
      // computed from the just-published state.
      if (cmd_seq_.load(std::memory_order_relaxed) >= seq_at_publish + 1)
      {
        trigger = ExchangeTrigger::kArrivalCount;
        break;
      }
      if (Clock::now() >= deadline)
      {
        AddViolation(kViolationExchangeTimeout);
        FailClosed("exchange timeout waiting for controller command");
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
          violations_.load(std::memory_order_relaxed)});
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
            violations_.load(std::memory_order_relaxed)});
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

  void Record(const IntervalRecord &record)
  {
    interval_count_.fetch_add(1, std::memory_order_relaxed);
    if (!trace_ok_ || !trace_.is_open()) return;
    trace_ << record.sim_tick_ms << "," << record.step_index << ","
           << record.phase << "," << record.cmd_seq_at_publish << ","
           << record.cmd_seq_at_ready << "," << record.exchange_wait_us << ","
           << record.publish_wall_us << ","
           << static_cast<int>(record.trigger) << "," << record.violations
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
  std::uint64_t next_expected_tick_ = 0;
  std::uint64_t last_published_tick_ = 0;
  std::uint64_t interval_index_ = 0;
  std::uint64_t barrier_seq_ = 0;
  std::int64_t startup_begin_us_ = 0;
};

} // namespace lockstep
