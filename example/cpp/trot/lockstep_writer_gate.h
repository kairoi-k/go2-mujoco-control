#pragma once
// Order-108 verification-only controller-side lockstep tick gate.
//
// The Order-103/105/106/107 sim coordinator freezes one physics state and
// republishes it at the 1000 Hz bridge rate until the controller acks the
// exact {state_seq, command_seq} pair; the controller's 500 Hz wall-clock
// writer therefore observes the SAME tick multiple times and runs several
// control updates per physics tick (Order-107 trace: command_seq deltas of
// 1/2/3/4, controller internal clock/gait phase ~2x sim time). This gate
// makes the lowcmd writer consume exactly ONE new tick per loop iteration
// once the controller handoff has completed:
//   * OnLowState(tick) (DDS LowState handler): strictly-new-tick detection.
//     A repeat publish of the same tick never signals; once engaged, a
//     backward tick (stale/reorder) and a forward tick that is not exactly
//     +dt fail closed with a stderr diagnostic;
//   * Engage(consumed_tick) (writer, once, at handoff): records the tick
//     the handoff control update consumed. A strictly-new tick that already
//     arrived (handler tick != consumed tick) stays pending, so the first
//     lockstep tick is never missed, while stale signals for already-
//     consumed ticks are naturally ignored (they equal the consumed tick);
//   * WaitForTick(): blocks until a strictly-new tick is pending; returns
//     kAborted when the abort predicate turns true (external stop; not a
//     failure), kTimeout when the gate failed closed (handler-side
//     stale/reorder/gap or no new tick within the configured wait);
//   * RecordConsumed(tick): the writer records the tick its control update
//     actually consumed (in TrotExperiment the exact snapshot tick carried
//     by the Order-107 ack).
//
// Before the handoff the gate only records ticks (original wall-clock
// lifecycle); with TROT_LOCKSTEP_ACK off it is never engaged and the
// wall-clock writer loop is unchanged. The gate is DDS-free so the
// invariants are unit-testable (example/cpp/tests/test_lockstep_writer_gate.cpp).

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <mutex>

namespace lockstep_writer
{

enum class WaitResult : int
{
  kTick = 0,    // strictly-new tick pending: run exactly one control update
  kAborted = 1, // abort predicate fired while waiting (external stop)
  kTimeout = 2, // fail-closed: stale/reorder/gap or wait timeout
};

enum ViolationFlag : std::uint32_t
{
  kViolationTickReorder = 1u << 0, // backward tick (stale/reorder)
  kViolationTickGap = 1u << 1,     // forward tick not exactly +dt
  kViolationTickTimeout = 1u << 2, // no new tick within the wait timeout
  kViolationSnapshotMismatch = 1u << 3, // writer saw a different state tick
};

class WriterGate
{
public:
  struct Config
  {
    std::uint64_t dt_ms;          // expected physics tick advance
    double tick_wait_timeout_s;   // fail closed if no new tick
  };

  explicit WriterGate(Config cfg = Config()) : cfg_(std::move(cfg))
  {
    if (cfg_.dt_ms == 0) cfg_.dt_ms = 2;
    if (cfg_.tick_wait_timeout_s <= 0.0) cfg_.tick_wait_timeout_s = 5.0;
  }

  void SetTickWaitTimeoutS(double timeout_s)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (timeout_s > 0.0) cfg_.tick_wait_timeout_s = timeout_s;
  }

  // Handler side (DDS LowState subscriber thread). Duplicate publishes of
  // the same tick never signal; while engaged, stale/reorder and gap fail
  // closed. Pre-handoff this only records the latest tick so the handoff
  // pending detection below stays exact.
  void OnLowState(std::uint32_t tick)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failed_closed_) return;
    if (!have_tick_)
    {
      have_tick_ = true;
      handler_tick_ = tick;
      return;
    }
    const std::uint32_t delta = tick - handler_tick_; // modulo 2^32
    if (delta == 0) return; // repeat publish of the same tick: ignore
    if (delta >= (1u << 31))
    {
      // Backward in time (stale/reorder; a 2^32 wrap forward is likewise
      // outside any bounded run and fails closed once engaged).
      if (!engaged_) return; // pre-handoff: keep the wall-clock lifecycle
      FailClosed(kViolationTickReorder, "LowState tick stale/reorder");
      return;
    }
    // Forward in time.
    if (!engaged_)
    {
      // Pre-handoff wall-clock phase: record only, keep startup lifecycle.
      handler_tick_ = tick;
      return;
    }
    if (delta == cfg_.dt_ms)
    {
      handler_tick_ = tick;
      cv_.notify_all();
    }
    else
    {
      FailClosed(kViolationTickGap, "LowState tick gap");
    }
  }

  // Writer side: handoff. Records the tick the handoff control update
  // consumed. Any strictly-new tick already observed stays pending (the
  // first lockstep tick is never missed); stale signals for already-
  // consumed ticks are ignored because they equal the consumed tick.
  void Engage(std::uint32_t consumed_tick)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    engaged_ = true;
    consumed_tick_ = consumed_tick;
  }

  // Blocks until a strictly-new tick is pending. *tick_out receives the
  // pending tick. kTimeout means the gate failed closed (handler-side
  // violation or wait timeout); the diagnostic was already emitted.
  template <typename AbortFn>
  WaitResult WaitForTick(AbortFn abort, std::uint32_t *tick_out = nullptr)
  {
    using Clock = std::chrono::steady_clock;
    using Seconds = std::chrono::duration<double>;
    const auto deadline = Clock::now() + Seconds(cfg_.tick_wait_timeout_s);
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;)
    {
      if (failed_closed_) return WaitResult::kTimeout;
      if (engaged_ && handler_tick_ != consumed_tick_)
      {
        if (tick_out != nullptr) *tick_out = handler_tick_;
        return WaitResult::kTick;
      }
      if (abort()) return WaitResult::kAborted;
      if (Clock::now() >= deadline)
      {
        FailClosed(kViolationTickTimeout,
                   "no new LowState tick within timeout");
        return WaitResult::kTimeout;
      }
      // Bounded poll so an abort predicate that fires while no tick is
      // arriving (external stop / shutdown) is noticed promptly; a new tick
      // still wakes us immediately via the condition variable.
      cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
  }

  void RecordConsumed(std::uint32_t tick)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    consumed_tick_ = tick;
  }

  // The writer must run on the exact state tick returned by WaitForTick.
  // Applying a command to any other snapshot breaks the causal exchange.
  void FailSnapshotMismatch()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    FailClosed(kViolationSnapshotMismatch, "state snapshot tick mismatch");
  }

  bool Engaged() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return engaged_;
  }

  bool FailedClosed() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return failed_closed_;
  }

  // Test/diagnostic seam: reports whether a strictly new tick is pending
  // without consuming it. It does not alter the production gate.
  bool HasPendingTick() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return engaged_ && !failed_closed_ && handler_tick_ != consumed_tick_;
  }

  std::uint32_t Violations() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return violations_;
  }

  const char *LastFailReason() const
  {
    return fail_reason_;
  }

  // Test seam: replaces the default stderr diagnostic.
  void SetFailClosedHandler(std::function<void(const char *reason)> handler)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_closed_handler_ = std::move(handler);
  }

private:
  void FailClosed(std::uint32_t flag, const char *reason)
  {
    if (failed_closed_) return;
    failed_closed_ = true;
    violations_ |= flag;
    fail_reason_ = reason;
    cv_.notify_all();
    if (fail_closed_handler_)
    {
      fail_closed_handler_(reason);
      return;
    }
    std::fprintf(stderr,
                 "TROT_LOCKSTEP_WRITER_FAIL_CLOSED reason=%s violations=%u\n",
                 reason, violations_);
    std::fflush(stderr);
  }

  Config cfg_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  bool have_tick_ = false;
  bool engaged_ = false;
  bool failed_closed_ = false;
  std::uint32_t violations_ = 0;
  std::uint32_t handler_tick_ = 0;
  std::uint32_t consumed_tick_ = 0;
  const char *fail_reason_ = "";
  std::function<void(const char *)> fail_closed_handler_;
};

} // namespace lockstep_writer
