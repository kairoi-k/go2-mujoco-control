#pragma once
// Verification-only sim-time lockstep coordinator (Order-103/105/106/107).
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
//   * causal exchange (Order-107): the physics thread advances exactly one
//     mj_step only when the EXACT acked command has arrived AND the ack
//     state_seq equals the current immutable physics state. Both sides count
//     the SAME command stream (the controller increments one local command
//     sequence per LowCmd write and carries it as the ack command_seq; the
//     sim numbers LowCmd arrivals 1:1, so the acked command_seq equals the
//     arrival ordinal of the acked cycle's own LowCmd). The ack pair
//     {state_seq, lockstep_command_seq} is carried full-width in unitree
//     Error_ (source_/state_ are uint32_t; LowState.tick is uint32_t):
//     state_seq wraps at 2^32 ms (~49.7 days at 1 kHz) and command_seq wraps
//     after 2^32 writes. Both are interpreted with uint32 modular comparison
//     inside a bounded single-exchange window; half-space ambiguity fails
//     closed;
//   * independent latches: the sim latches the latest ack pair for the
//     current frozen state and counts LowCmd arrivals. The exchange completes
//     only when the acked command_seq resolves to an exact arrival that
//     happened after the exchange opened (an older post-publish LowCmd, a
//     duplicate, a reorder, a future/stale pair or a wrap ambiguity never
//     grants), so an ack that arrives before its own command can never be
//     unlocked by an older command. Ack-before-command and command-before-
//     ack both complete the exchange exactly once;
//   * ack validation: startup-phase acks (pre-handoff states) are ignored;
//     future (newer than the published state) and TRUE stale (older than the
//     last consumed state) acks fail closed. Duplicate acks are idempotent;
//     a late ack for the just-consumed state (the Order-106 canary race) is
//     tolerated; missing acks/commands and timeouts fail closed;
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

  // ---- uint32 modular resolution helpers (Order-107) ----
  // Resolve a uint32 wire state_seq to the uint64 sim-tick space nearest
  // `ref` (the current published tick). Returns 1 (resolved, *out valid),
  // 0 (exactly half the uint32 space away -> ambiguous -> fail closed), or
  // -1 (resolves below tick 0 -> pre-epoch startup state -> ignore).
  static int ResolveStateSeq(std::uint32_t wire, std::uint64_t ref,
                             std::uint64_t *out)
  {
    const std::uint64_t w = wire;
    const std::uint64_t r = ref & 0xFFFFFFFFu;
    const std::uint64_t d =
        (w >= r) ? (w - r) : ((1ULL << 32) - (r - w)); // (w - r) mod 2^32
    if (d == 0)
    {
      *out = ref;
      return 1;
    }
    if (d == (1ULL << 31)) return 0; // exact half-space: ambiguous
    if (d < (1ULL << 31))
    {
      *out = ref + d;
      return 1;
    }
    const std::uint64_t back = (1ULL << 32) - d;
    if (back > ref) return -1; // resolves below tick 0: pre-epoch state
    *out = ref - back;
    return 1;
  }

  // Resolve a uint32 acked command_seq against the current exchange window
  // of local arrival ordinals (window_open, arrivals]. Returns 1 (exact
  // arrival in the window, *out is its ordinal), 0 (the acked command has
  // not arrived yet: normal ack-before-own-command in-flight), or -1
  // (outside the bounded single-exchange window: an older command, a wrap
  // ambiguity or an unbounded window -> fail closed).
  static int ResolveCommandSeq(std::uint32_t wire, std::uint64_t window_open,
                               std::uint64_t arrivals, std::uint64_t *out)
  {
    const std::uint64_t n = arrivals - window_open; // window size
    if (n >= (1ULL << 31)) return -1;               // unbounded window
    const std::uint64_t m0 = (window_open + 1) & 0xFFFFFFFFu;
    const std::uint64_t w = wire;
    const std::uint64_t off =
        (w >= m0) ? (w - m0) : ((1ULL << 32) - (m0 - w)); // (w - m0) mod 2^32
    if (off < n)
    {
      *out = window_open + 1 + off;
      return 1;
    }
    if (off < (1ULL << 31)) return 0; // ahead of the window: not arrived yet
    return -1; // behind the window or half-space: stale/ambiguous
  }

  // ---- event feed (DDS subscriber callbacks) ----
  // Sim-local LowCmd arrival count. The sim numbers arrivals 1:1 with the
  // controller's ack command_seq (both count the same command stream from
  // their start), so the ordinal of an arrival IS the controller sequence of
  // that cycle's own LowCmd. The counter is uint64 internally; the wire
  // command_seq is uint32 and is interpreted modularly against the current
  // exchange window (see ResolveCommandSeq).
  void OnCommandArrived()
  {
    cmd_seq_.fetch_add(1, std::memory_order_relaxed);
  }

  // Order-107 causal handshake: the controller adapter acks the exact pair
  // {state_seq, command_seq} (Error_.source()/state(), both uint32_t) after
  // each LowCmd write. Validation against the frozen-state bookkeeping:
  //   * startup-phase acks (state resolves to a pre-handoff state) are
  //     ignored: they reference pre-barrier wall-clock states;
  //   * future (newer than the published state) and true-stale (older than
  //     the last consumed state) acks fail closed;
  //   * a matching ack{state_seq == published} is LATCHED for the open
  //     exchange (duplicates are idempotent; the latest pair wins); it can
  //     complete the exchange only when ResolveCommandSeq finds its exact
  //     arrival ordinal in the current exchange window;
  //   * a late ack for the just-consumed state (the controller was still
  //     computing on the just-frozen state when the sim advanced) is
  //     tolerated; per-topic DDS writer order guarantees it arrives while
  //     the next exchange is open, so it can never complete a stale state.
  void OnAckReceived(std::uint32_t state_seq, std::uint32_t cmd_seq)
  {
    const char *reason = nullptr;
    std::uint32_t violation = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!ack_validation_active_) return; // startup wall-clock phase
      std::uint64_t v = 0;
      const int r = ResolveStateSeq(state_seq, published_state_seq_, &v);
      if (r == 0)
      {
        violation = kViolationAckStale;
        reason = "ack state_seq half-space ambiguity";
      }
      else if (r < 0)
      {
        return; // resolves below tick 0: pre-epoch startup-phase state
      }
      else if (v < barrier_state_seq_)
      {
        return; // startup-phase ack (pre-handoff state): not an exchange ack
      }
      else if (v > published_state_seq_)
      {
        violation = kViolationAckFuture;
        reason = "future ack for unpublished state";
      }
      else if (v < last_consumed_state_seq_)
      {
        violation = kViolationAckStale;
        reason = "stale ack older than the last consumed state";
      }
      else if (v == published_state_seq_ && exchange_open_)
      {
        // Independent ack latch: the latest {state_seq, command_seq} pair
        // for the frozen state. Duplicates are idempotent; a command_seq
        // that already resolves outside the bounded exchange window (an
        // older command or a wrap ambiguity) fails closed immediately. The
        // exchange still needs ResolveCommandSeq to find the exact arrival.
        std::uint64_t ordinal = 0;
        const int m = ResolveCommandSeq(
            cmd_seq, first_publish_seq_,
            cmd_seq_.load(std::memory_order_relaxed), &ordinal);
        if (m < 0)
        {
          violation = kViolationAckCmdMismatch;
          reason = "ack command_seq outside the exchange window";
        }
        else
        {
          ack_latched_ = true;
          ack_latched_state_ = state_seq;
          ack_latched_cmd_ = cmd_seq;
        }
      }
      // v == last_consumed_state_seq_ (or a closed-exchange matching state):
      // late/duplicate ack for the just-consumed frozen state; idempotent
      // and unable to complete the current exchange.
    }
    if (reason != nullptr)
    {
      AddViolation(violation);
      FailClosed(AckReason(reason, state_seq, cmd_seq));
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
  //   * the exchange completes (kStepGranted) when a matching
  //     ack{state_seq == published} was latched AND its command_seq resolves
  //     to an exact arrival in the exchange window (a command that arrived
  //     after the first publish: only that exact acked command may be the
  //     causal proof); the acked command's ordinal is the arrival of the
  //     acked cycle's own LowCmd, so an older post-publish LowCmd, a
  //     duplicate, a reorder or a wrap ambiguity can never grant;
  //   * a latched ack whose command_seq is still ahead of the arrivals is
  //     ack-before-own-command (reverse order): the exchange waits for the
  //     exact arrival; a command_seq outside the bounded window and missing
  //     ack/command timeouts fail closed.
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
          ack_latched_ = false;
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
        const std::int64_t elapsed_us = NowUs() - first_publish_wall_us_;
        if (ack_latched_)
        {
          std::uint64_t ordinal = 0;
          const int m = ResolveCommandSeq(
              ack_latched_cmd_, first_publish_seq_,
              cmd_seq_.load(std::memory_order_relaxed), &ordinal);
          if (m == 1)
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
                ack_latched_state_, ack_latched_cmd_});
            ++interval_index_;
            outcome = PublishOutcome::kStepGranted;
          }
          else if (m < 0)
          {
            fail_violation = kViolationAckCmdMismatch;
            fail_reason = "ack command_seq outside the exchange window";
          }
          else if (elapsed_us >
                   static_cast<std::int64_t>(cfg_.exchange_timeout_s * 1e6))
          {
            fail_violation = kViolationExchangeTimeout;
            fail_reason = "exchange timeout waiting for the acked command "
                          "arrival";
          }
        }
        else if (elapsed_us >
                 static_cast<std::int64_t>(cfg_.exchange_timeout_s * 1e6))
        {
          fail_violation = kViolationAckMissing;
          fail_reason = "exchange timeout waiting for controller ack";
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

  // Test-support seam: seeds the local arrival counter so the uint32
  // command_seq wrap boundary (2^32 arrivals) can be exercised without
  // injecting four billion messages. Not used by the production paths.
  void SetCmdSeqBaseForTest(std::uint64_t base)
  {
    cmd_seq_.store(base, std::memory_order_relaxed);
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

  // Formats a fail-closed reason with the offending ack's pair. The ack
  // callback is a single DDS thread, so thread-local storage suffices.
  static const char *AckReason(const char *reason, std::uint32_t state_seq,
                               std::uint32_t cmd_seq)
  {
    thread_local char buf[192];
    std::snprintf(buf, sizeof(buf), "%s state_seq=%u command_seq=%u", reason,
                  state_seq, cmd_seq);
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
  bool ack_latched_ = false;
  std::uint32_t ack_latched_state_ = 0;
  std::uint32_t ack_latched_cmd_ = 0;
  std::uint64_t first_publish_seq_ = 0;
  std::int64_t first_publish_wall_us_ = 0;
  std::uint64_t published_state_seq_ = 0;
  std::uint64_t last_consumed_state_seq_ = 0;
  std::uint64_t pending_step_tick_ = 0;
};

} // namespace lockstep
