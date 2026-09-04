// Order-108 controller-side lockstep writer tick gate tests.
//
// The gate (example/cpp/trot/lockstep_writer_gate.h) is DDS-free; these
// tests drive the LowState-handler side and the lowcmd-writer side directly
// and assert the invariants the TrotExperiment integration relies on:
//   * 1000 Hz frozen-state republish + 500 Hz writer: exactly ONE control
//     update per physics tick (command_seq delta == 1, controller_time
//     delta == 2 ms); duplicate same-tick publishes never trigger;
//   * handoff: pre-handoff (wall-clock) ticks are recorded without any
//     fail-closed; Engage() clears old events but never misses the first
//     lockstep tick;
//   * fail-closed: stale/reorder, gap, and no-new-tick timeout all fail
//     closed with the matching violation flag; abort stays a non-failure.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "lockstep_writer_gate.h"

namespace
{

int g_failures = 0;

void Check(bool condition, const char *what)
{
    if (!condition)
    {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++g_failures;
    }
}

// ------------------------------------------------------------------ tests

// Order-108 core trace gate: a 1000 Hz bridge that republishes each frozen
// physics tick several times, with a nominal-500 Hz gated writer. Each
// physics tick must produce exactly one control update: command_seq delta 1
// per tick and controller_time (running_time) delta 2 ms per tick. The
// duplicate publishes are present on purpose: the ungated 500 Hz writer of
// Order-107 consumed them as 1/2/3/4 command_seq deltas and ran the
// controller clock ~2x sim time.
void TestOneWritePerPhysicsTick()
{
    constexpr int kPhysicsTicks = 24;
    constexpr std::uint32_t kRepublishPerTick = 3; // 1000 Hz over 2 ms steps
    constexpr double kDtS = 0.002;

    lockstep_writer::WriterGate::Config cfg;
    cfg.dt_ms = 2;
    cfg.tick_wait_timeout_s = 10.0;
    lockstep_writer::WriterGate gate(cfg);
    gate.SetFailClosedHandler([](const char *) {});

    // Handoff already completed: the handoff control update consumed tick 0.
    gate.Engage(0);

    std::vector<std::uint32_t> consumed;
    std::vector<std::uint32_t> command_seqs;
    std::vector<double> controller_times;
    std::mutex m;
    std::condition_variable consumed_cv;
    std::atomic<bool> stop{false};

    std::thread writer([&]() {
        std::uint32_t cmd_seq = 0;
        double controller_time = 0.0;
        while (!stop.load())
        {
            std::uint32_t tick = 0;
            lockstep_writer::WaitResult r =
                gate.WaitForTick([&]() { return stop.load(); }, &tick);
            if (r == lockstep_writer::WaitResult::kAborted) return;
            if (r == lockstep_writer::WaitResult::kTimeout) return;
            // one full control update for this tick: advance the controller
            // clock by dt and count the command, then hand the consumed
            // tick back to the gate.
            {
                std::lock_guard<std::mutex> lk(m);
                consumed.push_back(tick);
                command_seqs.push_back(++cmd_seq);
                controller_time += kDtS;
                controller_times.push_back(controller_time);
            }
            consumed_cv.notify_all();
            gate.RecordConsumed(tick);
            // nominal 500 Hz cadence between writes (the Order-107 writer);
            // the gate is what keeps it one-write-per-tick regardless.
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    // 1000 Hz bridge: each physics tick (dt=2 ms) is republished
    // kRepublishPerTick times with the same tick; the sim advances to the
    // next tick only after the controller consumed the current one (the
    // causal exchange), so the publisher waits for that consumption.
    for (int i = 1; i <= kPhysicsTicks; ++i)
    {
        const std::uint32_t tick = static_cast<std::uint32_t>(i * 2);
        for (std::uint32_t r = 0; r < kRepublishPerTick; ++r)
        {
            gate.OnLowState(tick);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        {
            std::unique_lock<std::mutex> lk(m);
            consumed_cv.wait(lk, [&]() {
                return consumed.size() >= static_cast<std::size_t>(i) ||
                       stop.load();
            });
        }
    }

    // Extra duplicate publishes of the final tick: must not trigger a write.
    for (int r = 0; r < 5; ++r)
        gate.OnLowState(static_cast<std::uint32_t>(kPhysicsTicks * 2));
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    std::size_t count_before_stop = 0;
    {
        std::lock_guard<std::mutex> lk(m);
        count_before_stop = consumed.size();
    }
    stop.store(true);
    writer.join();

    Check(!gate.FailedClosed(), "1000Hz/500Hz gated run never fails closed");
    Check(count_before_stop == static_cast<std::size_t>(kPhysicsTicks),
          "exactly one write per physics tick (duplicates never trigger)");
    Check(consumed.size() == static_cast<std::size_t>(kPhysicsTicks),
          "consumed count equals physics tick count");
    for (std::size_t i = 0; i < consumed.size(); ++i)
    {
        Check(consumed[i] == static_cast<std::uint32_t>((i + 1) * 2),
              "consumed ticks are strictly new and in order");
        Check(command_seqs[i] == static_cast<std::uint32_t>(i + 1),
              "command_seq increments by exactly 1 per physics tick");
        if (i > 0)
        {
            Check(command_seqs[i] - command_seqs[i - 1] == 1,
                  "command_seq delta == 1 between physics ticks");
            Check(std::fabs(controller_times[i] - controller_times[i - 1] -
                            kDtS) < 1e-12,
                  "controller_time delta == 2 ms per physics tick");
        }
    }
    Check(consumed.size() >= 2 &&
              consumed.back() - consumed[consumed.size() - 2] == 2,
          "no duplicate/gapped ticks in the consumed sequence");
}

// The pathology the gate removes: an ungated writer sampling the latest
// published state observes the same physics tick multiple times.
void TestUngatedWriterSeesDuplicateTicks()
{
    std::atomic<std::uint32_t> latest_tick{0};
    std::vector<std::uint32_t> observed;
    std::atomic<bool> stop{false};

    std::thread writer([&]() {
        while (!stop.load())
        {
            observed.push_back(latest_tick.load());
            std::this_thread::yield();
        }
    });

    for (int i = 1; i <= 8; ++i)
    {
        const std::uint32_t tick = static_cast<std::uint32_t>(i * 2);
        for (int r = 0; r < 3; ++r)
        {
            latest_tick.store(tick);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    stop.store(true);
    writer.join();

    bool saw_duplicate = false;
    std::size_t start = 0;
    while (start < observed.size() && observed[start] == 0) ++start;
    for (std::size_t i = start + 1; i < observed.size(); ++i)
    {
        if (observed[i] == observed[i - 1])
        {
            saw_duplicate = true;
            break;
        }
    }
    Check(saw_duplicate,
          "ungated writer observes duplicate ticks (the Order-107 1/2/3/4 "
          "pathology the gate removes)");
}

// Duplicate same-tick publishes never signal; with no new tick the wait
// times out and fails closed with the timeout violation.
void TestDuplicateDoesNotTriggerAndTimeoutFailClosed()
{
    lockstep_writer::WriterGate::Config cfg;
    cfg.dt_ms = 2;
    cfg.tick_wait_timeout_s = 0.2;
    lockstep_writer::WriterGate gate(cfg);
    gate.SetFailClosedHandler([](const char *) {});

    gate.Engage(0);
    gate.OnLowState(2);
    std::uint32_t tick = 0;
    Check(gate.WaitForTick([]() { return false; }, &tick) ==
              lockstep_writer::WaitResult::kTick,
          "strictly-new tick wakes the writer");
    Check(tick == 2, "pending tick is the new tick");
    gate.RecordConsumed(2);

    for (int i = 0; i < 20; ++i)
        gate.OnLowState(2); // repeat publishes of the same frozen state
    const lockstep_writer::WaitResult r =
        gate.WaitForTick([]() { return false; }, &tick);
    Check(r == lockstep_writer::WaitResult::kTimeout,
          "duplicate same-tick publishes never trigger a write");
    Check(gate.FailedClosed(), "no-new-tick wait fails closed");
    Check((gate.Violations() & lockstep_writer::kViolationTickTimeout) != 0,
          "timeout violation flag set");
}

// Handoff: pre-handoff (wall-clock) ticks with gaps and backward publishes
// are recorded without fail-closed; Engage() clears those old events but
// never misses the first strictly-new lockstep tick.
void TestHandoffFirstLockstepTickNotMissed()
{
    lockstep_writer::WriterGate::Config cfg;
    cfg.dt_ms = 2;
    cfg.tick_wait_timeout_s = 1.0;
    lockstep_writer::WriterGate gate(cfg);
    gate.SetFailClosedHandler([](const char *) {});

    // Pre-handoff wall-clock phase: any tick pattern is tolerated.
    gate.OnLowState(100);
    gate.OnLowState(104); // gap
    gate.OnLowState(100); // backward publish
    gate.OnLowState(106);
    Check(!gate.FailedClosed(), "pre-handoff keeps wall-clock lifecycle");

    // Handoff: the control update just consumed tick 106.
    gate.Engage(106);
    Check(gate.Engaged(), "gate engaged at handoff");
    Check(!gate.FailedClosed(), "handoff itself never fails closed");

    // The first strictly-new tick after handoff (exactly +dt) is consumed
    // exactly once, even though stale pre-handoff observations existed.
    gate.OnLowState(108);
    std::uint32_t tick = 0;
    Check(gate.WaitForTick([]() { return false; }, &tick) ==
              lockstep_writer::WaitResult::kTick,
          "first lockstep tick after handoff is not missed");
    Check(tick == 108, "first lockstep tick is 108");
    gate.RecordConsumed(108);
    gate.OnLowState(108); // duplicate of the consumed tick: no trigger
    Check(!gate.FailedClosed(), "duplicate of consumed tick is benign");
}

// A backward tick once engaged fails closed as stale/reorder.
void TestReorderStaleFailClosed()
{
    lockstep_writer::WriterGate::Config cfg;
    cfg.tick_wait_timeout_s = 1.0;
    lockstep_writer::WriterGate gate(cfg);
    gate.SetFailClosedHandler([](const char *) {});

    gate.Engage(0);
    gate.OnLowState(2);
    std::uint32_t tick = 0;
    Check(gate.WaitForTick([]() { return false; }, &tick) ==
              lockstep_writer::WaitResult::kTick,
          "tick 2 pending");
    gate.RecordConsumed(2);

    gate.OnLowState(2); // duplicate: benign
    Check(!gate.FailedClosed(), "duplicate before the stale tick is benign");
    gate.OnLowState(0); // backward: stale/reorder
    Check(gate.FailedClosed(), "backward tick fails closed");
    Check((gate.Violations() & lockstep_writer::kViolationTickReorder) != 0,
          "stale/reorder violation flag set");
    Check(gate.LastFailReason() != nullptr &&
              gate.LastFailReason()[0] != '\0',
          "fail-closed diagnostic reason is set");
}

// A forward tick that is not exactly +dt fails closed as a gap.
void TestGapFailClosed()
{
    lockstep_writer::WriterGate::Config cfg;
    cfg.tick_wait_timeout_s = 1.0;
    lockstep_writer::WriterGate gate(cfg);
    gate.SetFailClosedHandler([](const char *) {});

    gate.Engage(0);
    gate.OnLowState(2);
    gate.RecordConsumed(2);
    gate.OnLowState(6); // delta 4 != dt 2
    Check(gate.FailedClosed(), "forward tick gap fails closed");
    Check((gate.Violations() & lockstep_writer::kViolationTickGap) != 0,
          "gap violation flag set");
}

void TestSnapshotMismatchFailClosed()
{
    lockstep_writer::WriterGate::Config cfg;
    cfg.tick_wait_timeout_s = 1.0;
    lockstep_writer::WriterGate gate(cfg);
    gate.SetFailClosedHandler([](const char *) {});

    gate.FailSnapshotMismatch();
    Check(gate.FailedClosed(), "snapshot mismatch fails closed");
    Check((gate.Violations() &
           lockstep_writer::kViolationSnapshotMismatch) != 0,
          "snapshot mismatch violation flag set");
}

// Abort predicate (external stop) returns kAborted without failing closed.
void TestAbortWhileWaiting()
{
    lockstep_writer::WriterGate::Config cfg;
    cfg.tick_wait_timeout_s = 5.0;
    lockstep_writer::WriterGate gate(cfg);
    gate.SetFailClosedHandler([](const char *) {});

    gate.Engage(0);
    std::atomic<bool> abort_flag{false};
    lockstep_writer::WaitResult r = lockstep_writer::WaitResult::kTick;
    std::thread t([&]() {
        r = gate.WaitForTick([&]() { return abort_flag.load(); });
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    abort_flag.store(true);
    t.join();
    Check(r == lockstep_writer::WaitResult::kAborted,
          "abort returns kAborted");
    Check(!gate.FailedClosed(), "abort does not fail closed");
    Check(gate.Violations() == 0, "abort records no violation");
}

// Modular tick arithmetic: a forward step across the uint32 wrap is exact.
void TestWrapTickArithmetic()
{
    lockstep_writer::WriterGate::Config cfg;
    cfg.dt_ms = 2;
    cfg.tick_wait_timeout_s = 1.0;
    lockstep_writer::WriterGate gate(cfg);
    gate.SetFailClosedHandler([](const char *) {});

    const std::uint32_t near_wrap = 0xFFFFFFFEu; // last even tick before wrap
    gate.Engage(near_wrap - 2);
    gate.OnLowState(near_wrap);
    std::uint32_t tick = 0;
    Check(gate.WaitForTick([]() { return false; }, &tick) ==
              lockstep_writer::WaitResult::kTick &&
              tick == near_wrap,
          "normal tick accepted");
    gate.RecordConsumed(near_wrap);

    gate.OnLowState(0); // 0x00000000 = (near_wrap + 2) mod 2^32
    Check(gate.WaitForTick([]() { return false; }, &tick) ==
              lockstep_writer::WaitResult::kTick &&
              tick == 0,
          "tick wrap across 2^32 resolves as an exact forward step");
    Check(!gate.FailedClosed(), "exact wrap step does not fail closed");
    gate.RecordConsumed(0);

    gate.OnLowState(2); // forward again: exact
    Check(gate.WaitForTick([]() { return false; }, &tick) ==
              lockstep_writer::WaitResult::kTick &&
              tick == 2,
          "tick after wrap resolves exactly");
}

} // namespace

int main()
{
    TestOneWritePerPhysicsTick();
    TestUngatedWriterSeesDuplicateTicks();
    TestDuplicateDoesNotTriggerAndTimeoutFailClosed();
    TestHandoffFirstLockstepTickNotMissed();
    TestReorderStaleFailClosed();
    TestGapFailClosed();
    TestSnapshotMismatchFailClosed();
    TestAbortWhileWaiting();
    TestWrapTickArithmetic();

    if (g_failures != 0)
    {
        std::fprintf(stderr, "lockstep_writer_gate: %d failure(s)\n",
                     g_failures);
        return 1;
    }
    std::printf("lockstep_writer_gate: all checks passed\n");
    return 0;
}
