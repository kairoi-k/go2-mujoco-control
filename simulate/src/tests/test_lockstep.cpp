// Unit/integration tests for the Order-103/105/106/107 sim-time lockstep
// coordinator (simulate/src/lockstep.h). The coordinator is DDS/MuJoCo-free;
// tests drive the ready/publish/exchange/tick/ack protocol directly and
// assert the invariants the simulator integration relies on:
//   * ready barrier completes only after the first controller command;
//   * one frozen interval per causal exchange; exact dt tick sequence
//     (no duplicate/missing/reordered tick);
//   * Order-106 frozen republish: repeated 1000 Hz publishes of one frozen
//     state carry the same state_seq and never fail closed;
//   * Order-107 exact causal binding: the exchange completes only when the
//     acked {state_seq, command_seq} pair's EXACT arrival is in the current
//     exchange window (the controller counts every LowCmd write 1:1 with the
//     sim's local arrival ordinals). An older post-publish LowCmd, a
//     duplicate, a reorder, a future/stale pair or a wrap ambiguity never
//     grants; ack-before-own-command and command-before-ack both complete
//     the exchange exactly once;
//   * Order-107 reviewer P2 reproduction: an older post-publish LowCmd is
//     already present, then the updated ack for a newer command arrives
//     before that command; physics must NOT advance until the exact newer
//     LowCmd callback arrives, and the older command never steps;
//   * ack validation: startup-phase (pre-handoff) acks ignored; TRUE stale
//     (older than the last consumed state) and future/reordered acks fail
//     closed; duplicate acks idempotent; late acks for the just-consumed
//     state tolerated; missing ack/command and timeouts fail closed;
//   * uint32 modular resolution: state_seq beyond 80 s, state_seq crossing
//     the 2^32 ms wrap and command_seq crossing the 2^32 write wrap are all
//     exact, and half-space ambiguity fails closed;
//   * external stop aborts waits without failing closed;
//   * repeated identical event scripts yield an identical deterministic
//     trace (stable columns), wall-timing columns excluded.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "lockstep.h"

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

std::uint32_t Or(std::initializer_list<std::uint32_t> flags)
{
    std::uint32_t out = 0;
    for (std::uint32_t f : flags) out |= f;
    return out;
}

// ---------------------------------------------------------------- helpers

// One LowCmd arrival; mirrors the coordinator's local arrival counter.
void FeedCommand(lockstep::Coordinator *coord)
{
    coord->OnCommandArrived();
}

// Ack for a frozen state with the exact controller command_seq (Order-107).
void FeedAck(lockstep::Coordinator *coord, std::uint32_t state_seq,
             std::uint32_t cmd_seq)
{
    coord->OnAckReceived(state_seq, cmd_seq);
}

// Test-side model of the controller ack adapter: every LowCmd write is the
// next absolute command_seq, so the ack for a write carries that write's
// seq (matching the sim's local arrival ordinals 1:1).
struct WriteClock
{
    std::uint32_t next;

    explicit WriteClock(std::uint32_t start_seq = 1) : next(start_seq) {}

    // Feed the write's LowCmd arrival; returns the command_seq its ack
    // must carry.
    std::uint32_t Write(lockstep::Coordinator *coord)
    {
        coord->OnCommandArrived();
        return next++;
    }
};

// Bridge + physics roles for the non-blocking interval protocol.
struct ProtocolDriver
{
    explicit ProtocolDriver(lockstep::Coordinator *c) : coord(c) {}

    // Bridge role: one 1000 Hz publish of the frozen state.
    lockstep::PublishOutcome Publish(std::uint64_t tick)
    {
        return coord->OnPublish(tick);
    }

    // Physics role: wait for one granted step and report it.
    lockstep::WaitOutcome Step(std::uint64_t next_tick)
    {
        lockstep::WaitOutcome w = coord->WaitForStepPermission();
        if (w != lockstep::WaitOutcome::kReady) return w;
        coord->NotifyStepCompleted(next_tick);
        return lockstep::WaitOutcome::kReady;
    }

    lockstep::Coordinator *coord;
};

// Runs the full protocol for `interval_count` intervals starting at
// `start_tick` with scripted 1:1 command/ack events (each interval feeds the
// ack before its command, covering the ack-before-command callback order on
// the open exchange). Returns the trace text; events are scripted so the
// run is reproducible.
std::string RunFullProtocol(int interval_count, const std::string &trace_path,
                            std::uint64_t start_tick = 0)
{
    lockstep::Coordinator::Config cfg;
    cfg.dt_ms = 2;
    cfg.trace_path = trace_path;
    cfg.barrier_timeout_s = 5.0;
    cfg.exchange_timeout_s = 2.0;
    cfg.step_wait_timeout_s = 2.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    ProtocolDriver driver(&coord);

    // Startup: no command yet -> barrier not complete.
    Check(!coord.OnStartupPublish(start_tick),
          "barrier not complete before first cmd");
    FeedCommand(&coord); // controller write #1 -> barrier
    Check(coord.OnStartupPublish(start_tick),
          "barrier completes on first command");
    Check(coord.BarrierComplete(), "BarrierComplete() true after barrier");

    Check(driver.Step(start_tick + 2) == lockstep::WaitOutcome::kReady,
          "first physics step granted");

    std::uint64_t tick = start_tick + 2;
    WriteClock wc(2); // the barrier command was write #1; next write is #2
    for (int i = 0; i < interval_count; ++i)
    {
        // First publish of the frozen state opens the exchange.
        Check(driver.Publish(tick) == lockstep::PublishOutcome::kIdle,
              "exchange opened on first publish");
        const std::uint32_t seq = wc.Write(&coord);
        FeedAck(&coord, static_cast<std::uint32_t>(tick), seq);
        // Next publish (republish of the same frozen state) evaluates the
        // exchange.
        Check(driver.Publish(tick) == lockstep::PublishOutcome::kStepGranted,
              "exchange completes on the exact ack pair + its command");
        coord.NotifyCommandApplied();
        Check(driver.Step(tick + 2) == lockstep::WaitOutcome::kReady,
              "physics step granted for every interval");
        tick += 2;
    }

    Check(!coord.FailedClosed(), "no fail-closed in the happy path");
    Check(coord.ViolationCount() == 0, "zero violations in the happy path");
    Check(coord.IntervalCount() ==
              static_cast<std::uint64_t>(interval_count + 1),
          "trace interval count matches");

    std::ifstream in(trace_path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// Prepares the coordinator for one lockstep exchange on `tick`: barrier,
// first step, first publish of the frozen state.
struct ExchangeScenario
{
    lockstep::Coordinator *coord;
    std::uint64_t tick = 0;
    std::uint32_t next_cmd_seq = 1; // controller write counter (write #1 was
                                    // the barrier command)

    explicit ExchangeScenario(lockstep::Coordinator *c,
                              std::uint64_t start_tick)
        : coord(c), tick(start_tick)
    {
        FeedCommand(coord); // first controller command -> barrier
        ++next_cmd_seq;     // the exchange command is write #2
        Check(coord->OnStartupPublish(tick - 2),
              "barrier completes in ExchangeScenario");
        Check(coord->WaitForStepPermission() == lockstep::WaitOutcome::kReady,
              "first step granted in ExchangeScenario");
        coord->NotifyStepCompleted(tick);
        Check(coord->OnPublish(tick) == lockstep::PublishOutcome::kIdle,
              "first publish opens the exchange in ExchangeScenario");
    }

    // Completes the exchange for the current frozen state with the exact
    // ack pair and its command arrival.
    bool CompleteExchange()
    {
        const std::uint32_t seq = next_cmd_seq++;
        FeedAck(coord, static_cast<std::uint32_t>(tick), seq);
        FeedCommand(coord);
        return coord->OnPublish(tick) == lockstep::PublishOutcome::kStepGranted;
    }

    // Re-arms the scenario for the next frozen state (physics already
    // stepped to `next_tick`).
    void NextExchange(std::uint64_t next_tick)
    {
        coord->NotifyCommandApplied();
        Check(coord->WaitForStepPermission() == lockstep::WaitOutcome::kReady,
              "next step granted in NextExchange");
        coord->NotifyStepCompleted(next_tick);
        tick = next_tick;
        Check(coord->OnPublish(tick) == lockstep::PublishOutcome::kIdle,
              "next first publish opens the exchange");
    }
};

// ------------------------------------------------------------------ tests

void TestBarrierTimeout()
{
    lockstep::Coordinator::Config cfg;
    cfg.barrier_timeout_s = 0.3;
    cfg.exchange_timeout_s = 0.3;
    cfg.step_wait_timeout_s = 0.3;
    lockstep::Coordinator coord(cfg);
    int failed_closed_calls = 0;
    coord.SetFailClosedHandler([&failed_closed_calls]() { ++failed_closed_calls; });

    lockstep::WaitOutcome outcome = lockstep::WaitOutcome::kReady;
    std::thread t([&]() { outcome = coord.WaitForStepPermission(); });
    t.join();

    Check(outcome == lockstep::WaitOutcome::kTimeoutFailClosed,
          "barrier timeout returns kTimeoutFailClosed");
    Check(coord.FailedClosed(), "FailedClosed set after barrier timeout");
    Check((coord.ViolationCount() & lockstep::kViolationBarrierTimeout) != 0,
          "barrier timeout violation flag set");
    Check(failed_closed_calls == 1, "fail-closed handler called once");
}

void TestExchangeTimeout()
{
    lockstep::Coordinator::Config cfg;
    cfg.barrier_timeout_s = 1.0;
    cfg.exchange_timeout_s = 0.3;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    FeedCommand(&coord);
    Check(coord.OnStartupPublish(0), "barrier complete");
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "step 1 granted");
    coord.NotifyStepCompleted(2);
    Check(coord.OnPublish(2) == lockstep::PublishOutcome::kIdle,
          "exchange opened");

    // No commands and no acks arrive: the exchange deadline fails closed.
    std::this_thread::sleep_for(std::chrono::milliseconds(350));
    Check(coord.OnPublish(2) == lockstep::PublishOutcome::kIdle,
          "no step without ack/command");
    Check(coord.FailedClosed(), "exchange timeout fails closed");
    Check((coord.ViolationCount() & lockstep::kViolationAckMissing) != 0,
          "missing ack violation flag set on exchange timeout");
}

void TestCommandArrivalTimeout()
{
    lockstep::Coordinator::Config cfg;
    cfg.barrier_timeout_s = 1.0;
    cfg.exchange_timeout_s = 0.3;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    FeedCommand(&coord);
    Check(coord.OnStartupPublish(0), "barrier complete");
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "step 1 granted");
    coord.NotifyStepCompleted(2);
    Check(coord.OnPublish(2) == lockstep::PublishOutcome::kIdle,
          "exchange opened");
    FeedAck(&coord, 2, 2); // the acked command (write #2) never arrives
    std::this_thread::sleep_for(std::chrono::milliseconds(350));
    Check(coord.OnPublish(2) == lockstep::PublishOutcome::kIdle,
          "ack without its exact command never grants a step");
    Check(coord.FailedClosed(),
          "ack-without-its-exact-command fails closed");
    Check((coord.ViolationCount() & lockstep::kViolationExchangeTimeout) != 0,
          "exchange timeout violation flag set");
}

void TestAckSeqMisalignmentTimesOut()
{
    // A controller command_seq far ahead of the local arrivals (epoch
    // misalignment / lost messages) stays pending and the exchange times out
    // fail-closed; it must never be unlocked by the arrivals that do exist.
    lockstep::Coordinator::Config cfg;
    cfg.barrier_timeout_s = 1.0;
    cfg.exchange_timeout_s = 0.3;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    FeedCommand(&coord);
    Check(coord.OnStartupPublish(0), "barrier complete");
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "step 1 granted");
    coord.NotifyStepCompleted(2);
    Check(coord.OnPublish(2) == lockstep::PublishOutcome::kIdle,
          "exchange opened");
    FeedAck(&coord, 2, 1000); // acked command is 999 arrivals ahead
    Check(!coord.FailedClosed(), "far-ahead ack is not an immediate violation");
    FeedCommand(&coord); // an arrival exists, but it is not the acked one
    std::this_thread::sleep_for(std::chrono::milliseconds(350));
    Check(coord.OnPublish(2) == lockstep::PublishOutcome::kIdle,
          "misaligned ack is never unlocked by an existing arrival");
    Check(coord.FailedClosed(), "misaligned epoch times out fail-closed");
    Check((coord.ViolationCount() & lockstep::kViolationExchangeTimeout) != 0,
          "exchange timeout violation flag set");
}

void TestStepTimeout()
{
    lockstep::Coordinator::Config cfg;
    cfg.barrier_timeout_s = 1.0;
    cfg.exchange_timeout_s = 1.0;
    cfg.step_wait_timeout_s = 0.3;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    FeedCommand(&coord);
    Check(coord.OnStartupPublish(0), "barrier complete");
    // First permission granted by the barrier; grant consumed, then never
    // re-granted -> the second step-permission wait must time out.
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "step 1 granted");
    lockstep::WaitOutcome outcome = lockstep::WaitOutcome::kReady;
    std::thread t([&]() { outcome = coord.WaitForStepPermission(); });
    t.join();

    Check(outcome == lockstep::WaitOutcome::kTimeoutFailClosed,
          "step permission timeout returns kTimeoutFailClosed");
    Check((coord.ViolationCount() & lockstep::kViolationStepTimeout) != 0,
          "step timeout violation flag set");
}

void TestPhysicsTickMismatchFailClosed()
{
    lockstep::Coordinator::Config cfg;
    cfg.barrier_timeout_s = 1.0;
    cfg.exchange_timeout_s = 1.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    FeedCommand(&coord);
    Check(coord.OnStartupPublish(0), "barrier complete");
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "step 1 granted");
    coord.NotifyStepCompleted(3); // expected 2 -> step tick mismatch
    Check(coord.FailedClosed(), "physics step tick mismatch fails closed");
    Check((coord.ViolationCount() & lockstep::kViolationStepTick) != 0,
          "step tick violation flag set");
}

void TestRepublishTickMismatchFailClosed()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 1.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    ExchangeScenario sc(&coord, 4);
    // Republish with a tick that is not the frozen state: invariant break.
    Check(coord.OnPublish(6) == lockstep::PublishOutcome::kIdle,
          "mismatched republish returns idle");
    Check(coord.FailedClosed(), "republish tick mismatch fails closed");
}

void TestStartupWatchdog()
{
    lockstep::Coordinator::Config cfg;
    cfg.barrier_timeout_s = 0.3; // no command ever arrives
    lockstep::Coordinator coord(cfg);
    int failed_closed_calls = 0;
    coord.SetFailClosedHandler([&failed_closed_calls]() { ++failed_closed_calls; });

    const auto t0 = std::chrono::steady_clock::now();
    bool completed = false;
    while (std::chrono::steady_clock::now() - t0 <
           std::chrono::seconds(5))
    {
        if (!coord.OnStartupPublish(0))
        {
            if (coord.FailedClosed())
            {
                completed = true;
                break;
            }
            // barrier not yet complete; keep publishing startup states
        }
        else
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    Check(completed, "startup watchdog fails closed without a command");
    Check(failed_closed_calls == 1, "startup watchdog handler called once");
    Check((coord.ViolationCount() & lockstep::kViolationBarrierTimeout) != 0,
          "startup watchdog records barrier timeout violation");
}

void TestAbort()
{
    lockstep::Coordinator::Config cfg;
    cfg.barrier_timeout_s = 5.0;
    cfg.exchange_timeout_s = 5.0;
    cfg.step_wait_timeout_s = 5.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});
    std::atomic<bool> abort{false};
    coord.SetAbortCallback([&abort]() { return abort.load(); });

    lockstep::WaitOutcome outcome = lockstep::WaitOutcome::kReady;
    std::thread t([&]() { outcome = coord.WaitForStepPermission(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    abort.store(true);
    t.join();

    Check(outcome == lockstep::WaitOutcome::kAborted,
          "abort returns kAborted without fail-closed");
    Check(!coord.FailedClosed(), "abort does not fail closed");
    Check(coord.ViolationCount() == 0, "abort records no violation");
}

// Order-106 canary reproduction: the Order-105 C-006e canary failed closed
// at 1b29974 with "stale ack for older state" (violations=32) when the
// controller's ack for the just-published/just-consumed state was delivered
// after the simulator advanced (docs/research/evidence/order105_c006e). The
// same event sequence must now be idempotent: the ack for the barrier state
// (and later for the just-consumed state) is tolerated, and the exchange
// completes only on the exact ack{state_seq == published, command_seq}
// pair plus its own LowCmd arrival.
void TestCanaryLateAckReproduction()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 2.0;
    cfg.step_wait_timeout_s = 2.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    FeedCommand(&coord);
    Check(coord.OnStartupPublish(2300), "barrier at 2300 (canary baseline)");
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "barrier step granted");
    coord.NotifyStepCompleted(2302);
    Check(coord.OnPublish(2302) == lockstep::PublishOutcome::kIdle,
          "first lockstep publish opens the exchange at 2302");

    // Canary race: the controller was still computing on the barrier state
    // 2300 when the sim advanced; its ack{2300, 3} arrives after 2302 was
    // published. Order-105 classified this as stale and failed closed.
    FeedAck(&coord, 2300, 3);
    Check(!coord.FailedClosed(),
          "late ack for the just-consumed state is tolerated");
    FeedAck(&coord, 2300, 3); // a second identical late ack is also idempotent
    Check(!coord.FailedClosed(), "duplicate late ack is idempotent");
    Check(coord.OnPublish(2302) == lockstep::PublishOutcome::kIdle,
          "late ack alone never grants a step");

    // The controller consumes 2302: exact ack pair + its own LowCmd (the
    // 2nd controller write, so its arrival ordinal is also 2).
    FeedAck(&coord, 2302, 2);
    FeedCommand(&coord);
    Check(coord.OnPublish(2302) == lockstep::PublishOutcome::kStepGranted,
          "exchange for 2302 completes on the exact pair + its command");
    coord.NotifyCommandApplied();
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "step to 2304 granted");
    coord.NotifyStepCompleted(2304);
    Check(coord.OnPublish(2304) == lockstep::PublishOutcome::kIdle,
          "exchange for 2304 opened");

    // Late ack for the just-consumed state 2302 after the sim advanced:
    // tolerated (== last consumed), never fails closed.
    FeedAck(&coord, 2302, 2);
    Check(!coord.FailedClosed(),
          "post-advance ack of the consumed state is tolerated");
    Check(coord.OnPublish(2304) == lockstep::PublishOutcome::kIdle,
          "consumed-state ack does not complete the new exchange");

    Check(coord.ViolationCount() == 0, "canary sequence has zero violations");
}

// Order-106: a 1000 Hz bridge republishes the same frozen state (same
// state_seq) until the exchange completes; republishes never fail closed and
// never grant a step early.
void Test1000HzRepublishSameSeq()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 5.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    ExchangeScenario sc(&coord, 4);
    for (int i = 0; i < 10; ++i) // ~10 ms of 1000 Hz republish
    {
        Check(coord.OnPublish(4) == lockstep::PublishOutcome::kIdle,
              "republish of the frozen state stays idle");
        Check(!coord.FailedClosed(), "republish never fails closed");
    }
    Check(coord.ViolationCount() == 0,
          "republish does not produce violations");
    Check(sc.CompleteExchange(), "exchange completes after the republishes");
    Check(!coord.FailedClosed(), "no fail-closed after republish exchange");
}

// Order-107: a 500 Hz controller may write 1-2 LowCmds per frozen state and
// ack the exact pair more than once; duplicate acks (while the state is
// current or after the sim advanced to the next state) are idempotent and
// the exchange still completes only on the exact pair + its own arrival.
void Test500HzControllerDoubleWriteDupIdempotent()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 5.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    ExchangeScenario sc(&coord, 4);
    // First 500 Hz write on the frozen state 4: exact ack pair + command.
    Check(sc.CompleteExchange(), "first write completes the exchange");
    coord.NotifyCommandApplied();
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "step to 6 granted");
    coord.NotifyStepCompleted(6);
    Check(coord.OnPublish(6) == lockstep::PublishOutcome::kIdle,
          "exchange for 6 opened");

    // Second 500 Hz write was still computed on state 4 (the controller had
    // not received state 6 yet): its ack{4, 3} duplicates the just-consumed
    // state and its command is a fresh arrival.
    FeedAck(&coord, 4, 3);
    FeedCommand(&coord);
    Check(!coord.FailedClosed(), "duplicate ack for consumed state tolerated");
    FeedAck(&coord, 6, 4);
    FeedCommand(&coord);
    Check(coord.OnPublish(6) == lockstep::PublishOutcome::kStepGranted,
          "exchange for 6 completes normally");
    Check(!coord.FailedClosed(), "no fail-closed on controller double-write");
    Check(coord.ViolationCount() == 0, "double-write has zero violations");
}

// Order-107: an ack older than the last consumed state is a TRUE stale ack
// and fails closed (it can only occur through reordering or corruption).
void TestTrueStaleAckFailClosed()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 2.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    ExchangeScenario sc(&coord, 2);
    Check(sc.CompleteExchange(), "state 2 consumed");
    coord.NotifyCommandApplied();
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "step to 4 granted");
    coord.NotifyStepCompleted(4);
    Check(coord.OnPublish(4) == lockstep::PublishOutcome::kIdle,
          "exchange for 4 opened");

    // ack{2} (the just-consumed state): tolerated.
    FeedAck(&coord, 2, 3);
    Check(!coord.FailedClosed(), "ack for just-consumed state tolerated");
    // ack{0} (== barrier state, older than the last consumed state): TRUE
    // stale -> fail closed.
    FeedAck(&coord, 0, 1);
    Check(coord.FailedClosed(), "true stale ack fails closed");
    Check((coord.ViolationCount() & lockstep::kViolationAckStale) != 0,
          "stale ack violation flag set");
}

// Order-107: an ack for a state the sim has not published (future, e.g. a
// reordered ack of the next state) fails closed.
void TestFutureAckFailClosed()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 2.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    ExchangeScenario sc(&coord, 4);
    FeedAck(&coord, 8, 5); // ack for a future state (tick 8)
    Check(coord.FailedClosed(), "future ack fails closed");
    Check((coord.ViolationCount() & lockstep::kViolationAckFuture) != 0,
          "future ack violation flag set");
}

// Order-107: reordered ack (the next state's ack arriving first) is a future
// ack and fails closed.
void TestReorderAckFailClosed()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 2.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    ExchangeScenario sc(&coord, 4);
    FeedAck(&coord, 6, 4); // next state's ack arrives first
    Check(coord.FailedClosed(), "reordered ack fails closed");
    Check((coord.ViolationCount() & lockstep::kViolationAckFuture) != 0,
          "reordered ack recorded as future violation");
}

// Order-107: startup-phase acks (before the ready barrier or referencing
// pre-handoff states) are ignored, not validated.
void TestStartupAckIgnored()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 1.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    // Acks before the barrier (ack_validation inactive): ignored.
    FeedAck(&coord, 999, 1);
    Check(!coord.FailedClosed(), "pre-barrier ack ignored");
    Check(coord.ViolationCount() == 0, "pre-barrier ack no violation");

    FeedCommand(&coord);
    Check(coord.OnStartupPublish(0), "barrier complete");
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "step 1 granted");
    coord.NotifyStepCompleted(2);
    // A pre-handoff state's ack (startup wall-clock state) is ignored.
    FeedAck(&coord, 0, 1);
    Check(!coord.FailedClosed(), "pre-handoff ack ignored");
    Check(coord.OnPublish(2) == lockstep::PublishOutcome::kIdle,
          "exchange for 2 opened");
    FeedAck(&coord, 2, 2);
    FeedCommand(&coord);
    Check(coord.OnPublish(2) == lockstep::PublishOutcome::kStepGranted,
          "exchange completes after the ignored ack");
}

// Order-107: both callback orders complete the exchange exactly once.
void TestCallbackOrderAckBeforeCommand()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 1.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    ExchangeScenario sc(&coord, 4);
    FeedAck(&coord, 4, 2); // ack first (cross-topic reorder)
    FeedAck(&coord, 4, 2); // duplicate ack while current: idempotent
    Check(!coord.FailedClosed(), "duplicate matching ack idempotent");
    FeedCommand(&coord);   // the exact acked command arrives after the acks
    Check(coord.OnPublish(4) == lockstep::PublishOutcome::kStepGranted,
          "ack-before-command completes the exchange");
    Check(!coord.FailedClosed(), "ack-before-command path does not fail");
}

void TestCallbackOrderCommandBeforeAck()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 1.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    ExchangeScenario sc(&coord, 4);
    FeedCommand(&coord);   // the exact acked command arrives first
    Check(coord.OnPublish(4) == lockstep::PublishOutcome::kIdle,
          "command alone does not grant a step");
    FeedAck(&coord, 4, 2); // then the matching ack pair
    Check(coord.OnPublish(4) == lockstep::PublishOutcome::kStepGranted,
          "command-before-ack completes the exchange");
    Check(!coord.FailedClosed(), "command-before-ack path does not fail");
}

// Order-107 reviewer P2 reproduction: an OLD post-publish LowCmd (computed
// from the previous frozen state, arriving after the current state's first
// publish) is already present in the exchange; the UPDATED ack for a NEWER
// command (computed from the current state) arrives BEFORE that command.
// The older command must never unlock the exchange; physics advances only
// after the exact newer LowCmd callback (Order 107 step 4).
void TestP2OldPostPublishNeverSteps()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 2.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    ExchangeScenario sc(&coord, 4); // barrier at 2; exchange for 4 open
    // The controller's 2nd write was computed from the previous frozen
    // state 2; its LowCmd arrives post-publish of state 4 (in-flight across
    // the handoff step): it is the first arrival of exchange 4.
    FeedCommand(&coord); // arrival ordinal 2 (old post-publish LowCmd)
    FeedAck(&coord, 2, 2); // its ack references the consumed state 2
    Check(!coord.FailedClosed(), "old post-publish command/ack is tolerated");
    Check(coord.OnPublish(4) == lockstep::PublishOutcome::kIdle,
          "old post-publish command alone never grants");

    // The UPDATED ack for the newer command (3rd write, computed from state
    // 4) arrives BEFORE its own LowCmd:
    FeedAck(&coord, 4, 3);
    Check(!coord.FailedClosed(),
          "ack-before-own-command is not an immediate violation");
    Check(coord.OnPublish(4) == lockstep::PublishOutcome::kIdle,
          "the older post-publish LowCmd (#2) cannot unlock ack{4,3}");
    Check(coord.OnPublish(4) == lockstep::PublishOutcome::kIdle,
          "repeated republish still never steps before the exact arrival");

    // The exact newer LowCmd (arrival ordinal 3) finally arrives:
    FeedCommand(&coord);
    Check(coord.OnPublish(4) == lockstep::PublishOutcome::kStepGranted,
          "physics advances only after the exact newer LowCmd arrives");
    Check(!coord.FailedClosed(), "P2 sequence does not fail closed");
    Check(coord.ViolationCount() == 0,
          "P2 exact-binding sequence has zero violations");
}

// Order-107 P2 reverse order: the newer command arrives first and the ack
// for it arrives later (command-before-ack); the old post-publish command
// still never grants by itself.
void TestP2ReverseOrderCommandBeforeAck()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 2.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    ExchangeScenario sc(&coord, 4);
    FeedCommand(&coord); // old post-publish command (arrival #2)
    FeedAck(&coord, 2, 2);
    Check(coord.OnPublish(4) == lockstep::PublishOutcome::kIdle,
          "old post-publish command never grants");
    FeedCommand(&coord); // newer command (arrival #3) arrives first
    Check(coord.OnPublish(4) == lockstep::PublishOutcome::kIdle,
          "newer command alone never grants");
    FeedAck(&coord, 4, 3); // its ack arrives after the command
    Check(coord.OnPublish(4) == lockstep::PublishOutcome::kStepGranted,
          "reverse order completes on the exact newer command");
    Check(coord.ViolationCount() == 0, "reverse P2 has zero violations");
}

// Order-107 epoch alignment + missed pre-barrier messages: commands and acks
// written before the ready barrier but delivered after it (the DDS in-flight
// window) occupy their absolute ordinals and can never unlock the exchange;
// the first post-barrier lockstep command completes it exactly.
void TestEpochAlignmentMissedPreBarrier()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 2.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    // Barrier completes on the first command (arrival #1).
    FeedCommand(&coord);
    Check(coord.OnStartupPublish(4), "barrier at 4");
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "handoff step granted");
    coord.NotifyStepCompleted(6);
    Check(coord.OnPublish(6) == lockstep::PublishOutcome::kIdle,
          "exchange for 6 opened");

    // Missed pre-barrier messages delivered post-barrier: an in-flight
    // startup command (arrival #2) and a pre-handoff ack for state 0. The
    // startup command's ack references the consumed state 4 (tolerated); the
    // pre-handoff ack is ignored. Neither can unlock exchange 6.
    FeedCommand(&coord); // arrival #2 (written before the barrier)
    FeedAck(&coord, 4, 2);
    FeedAck(&coord, 0, 1);
    Check(!coord.FailedClosed(),
          "missed pre-barrier command/acks are tolerated");
    Check(coord.OnPublish(6) == lockstep::PublishOutcome::kIdle,
          "pre-barrier arrivals never unlock the exchange");

    // The first post-barrier lockstep command (write #3, computed from
    // state 6) aligns with the epoch and completes the exchange.
    FeedAck(&coord, 6, 3);
    FeedCommand(&coord); // arrival #3
    Check(coord.OnPublish(6) == lockstep::PublishOutcome::kStepGranted,
          "epoch-aligned exact pair completes the exchange");
    Check(!coord.FailedClosed(), "epoch alignment does not fail closed");
    Check(coord.ViolationCount() == 0, "epoch alignment has zero violations");
}

// Order-107: duplicate acks (same exact pair) grant exactly one step; a
// duplicate of the consumed state arriving after the advance is tolerated.
void TestDuplicateAckOneStep()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 2.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    ExchangeScenario sc(&coord, 4);
    FeedAck(&coord, 4, 2);
    FeedAck(&coord, 4, 2); // duplicate while current
    FeedCommand(&coord);   // the exact command arrives
    Check(coord.OnPublish(4) == lockstep::PublishOutcome::kStepGranted,
          "duplicate acks grant exactly one step");
    Check(!coord.FailedClosed(), "duplicate acks do not fail closed");
    Check(coord.ViolationCount() == 0, "duplicate acks produce no violations");

    coord.NotifyCommandApplied();
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "step to 6 granted");
    coord.NotifyStepCompleted(6);
    Check(coord.OnPublish(6) == lockstep::PublishOutcome::kIdle,
          "exchange for 6 opened");
    FeedAck(&coord, 4, 2); // stale duplicate for the consumed state
    FeedCommand(&coord);   // an extra arrival in exchange 6
    Check(!coord.FailedClosed(),
          "duplicate of the consumed state is tolerated");
    Check(coord.OnPublish(6) == lockstep::PublishOutcome::kIdle,
          "consumed-state duplicate cannot complete the new exchange");
}

// Order-107: each exchange grants exactly one step; after completion, no
// further acks/commands on the frozen state grant a second step.
void TestOneStepPerExchange()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 2.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    ExchangeScenario sc(&coord, 4);
    Check(sc.CompleteExchange(), "exchange for 4 completes");
    // Extra acks/commands on the just-completed frozen state: tolerated but
    // never grant a second step before the physics advances.
    FeedAck(&coord, 4, 3);
    FeedCommand(&coord);
    Check(coord.OnPublish(4) == lockstep::PublishOutcome::kIdle,
          "no second step before the physics advances");
    Check(!coord.FailedClosed(), "post-completion acks are tolerated");

    coord.NotifyCommandApplied();
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "step to 6 granted");
    coord.NotifyStepCompleted(6);
    Check(coord.OnPublish(6) == lockstep::PublishOutcome::kIdle,
          "exchange for 6 opened");
    FeedAck(&coord, 6, 4);
    FeedCommand(&coord);
    Check(coord.OnPublish(6) == lockstep::PublishOutcome::kStepGranted,
          "next exchange steps exactly once");
    Check(coord.ViolationCount() == 0, "one-step discipline zero violations");
}

void VerifyTraceSequence(const std::string &text, std::uint64_t start_tick,
                         int interval_count)
{
    std::istringstream in(text);
    std::string line;
    std::getline(in, line); // header
    int rows = 0;
    std::uint64_t expected_tick = start_tick;
    std::uint64_t expected_index = 0;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#') continue;
        std::vector<std::string> cols;
        std::size_t pos = 0;
        while (pos <= line.size())
        {
            std::size_t comma = line.find(',', pos);
            if (comma == std::string::npos) comma = line.size();
            cols.push_back(line.substr(pos, comma - pos));
            pos = comma + 1;
        }
        Check(cols.size() == 11, "trace row has 11 columns");
        if (cols.size() != 11) continue;
        const std::uint64_t tick = std::stoull(cols[0]);
        const std::uint64_t step_index = std::stoull(cols[1]);
        const std::uint64_t seq_at_publish = std::stoull(cols[3]);
        const std::uint64_t seq_at_ready = std::stoull(cols[4]);
        const int trigger = std::stoi(cols[7]);
        const std::uint32_t violations = std::stoul(cols[8]);
        const std::uint64_t ack_state_seq = std::stoull(cols[9]);
        const std::uint64_t ack_cmd_seq = std::stoull(cols[10]);
        Check(tick == expected_tick,
              "tick sequence is exact (no gap/dup/reorder)");
        Check(step_index == expected_index, "step index is sequential");
        Check(trigger == (rows == 0 ? 0 : 3),
              "barrier row uses kBarrier, others kAckMatched");
        Check(violations == 0, "no violations in happy-path rows");
        if (rows == 0)
        {
            Check(ack_state_seq == 0 && ack_cmd_seq == 0,
                  "barrier row has empty ack fields");
            Check(seq_at_ready == 1,
                  "barrier row records the barrier command ordinal");
        }
        else
        {
            // The sim's arrival ordinal of the acked command equals the
            // ack command_seq (1:1 epoch alignment), and the trace records
            // the exact acked pair.
            Check(ack_state_seq == tick,
                  "ack state_seq equals the published tick");
            Check(ack_cmd_seq == seq_at_ready,
                  "ack command_seq equals its arrival ordinal");
            Check(seq_at_publish == static_cast<std::uint64_t>(rows),
                  "publish count is the arrival count at first publish");
            Check(seq_at_ready == static_cast<std::uint64_t>(rows + 1),
                  "ready count is the matched arrival ordinal");
        }
        expected_tick += 2;
        ++expected_index;
        ++rows;
    }
    Check(rows == interval_count + 1,
          "trace has 1 barrier + interval_count rows");
}

void TestFullProtocolTrace()
{
    const std::string trace_a = "/tmp/lockstep_test_a.csv";
    const std::string trace_b = "/tmp/lockstep_test_b.csv";
    const std::string trace_c = "/tmp/lockstep_test_c.csv";
    std::string a = RunFullProtocol(50, trace_a);
    std::string b = RunFullProtocol(50, trace_b);
    // Handoff at a late tick (wall-clock startup lasted 1000 ms): barrier
    // row must record tick 1000 and the sequence must continue from there.
    std::string c = RunFullProtocol(50, trace_c, 1000);

    // Deterministic columns: drop the wall-timing columns (5 and 6); the
    // remaining columns are scripted identically.
    auto stable = [](const std::string &text) {
        std::istringstream in(text);
        std::ostringstream out;
        std::string line;
        bool first = true;
        while (std::getline(in, line))
        {
            if (first) { first = false; continue; } // header
            if (line.empty() || line[0] == '#') continue;
            std::vector<std::string> cols;
            std::size_t pos = 0;
            while (pos <= line.size())
            {
                std::size_t comma = line.find(',', pos);
                if (comma == std::string::npos) comma = line.size();
                cols.push_back(line.substr(pos, comma - pos));
                pos = comma + 1;
            }
            if (cols.size() != 11) continue;
            // sim_tick_ms, step_index, phase, cmd_seq_at_publish,
            // cmd_seq_at_ready, [wait_us], [wall_us], trigger, violations,
            // ack_state_seq, ack_cmd_seq
            out << cols[0] << "," << cols[1] << "," << cols[2] << ","
                << cols[3] << "," << cols[4] << "," << cols[7] << ","
                << cols[8] << "," << cols[9] << "," << cols[10] << "\n";
        }
        return out.str();
    };

    // Wall-timing columns differ between runs by design; the deterministic
    // columns (tick, step, phase, command sequences, trigger, violations,
    // ack fields) must match exactly across repeated identical scripts.
    Check(a != b, "wall-timing columns vary between runs (recorded)");
    Check(stable(a) == stable(b), "deterministic trace columns match");

    VerifyTraceSequence(a, 0, 50);
    VerifyTraceSequence(c, 1000, 50);
}

// Order-106/107: tick space beyond 80 s (tick > 80000 at 2 ms dt) must not
// wrap or truncate. The wire carries state_seq as uint32_t (Error_.source(),
// LowState.tick), which wraps only at 2^32 ms (~49.7 days at 1 kHz); this
// fast-forward proves the protocol is exact across the 80 s boundary and at
// late-tick handoffs.
void TestWrapBeyond80s()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 60.0; // never hit: every exchange completes inline
    cfg.step_wait_timeout_s = 60.0;
    cfg.trace_path.clear(); // no trace file for the fast-forward
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    FeedCommand(&coord);
    Check(coord.OnStartupPublish(0), "barrier complete at tick 0");
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "first step granted");
    coord.NotifyStepCompleted(2);

    // 41001 intervals advance the sim from tick 2 to 82004 ms (> 80 s).
    const std::uint64_t interval_count = 41001;
    std::uint64_t tick = 2;
    WriteClock wc(2); // the barrier command was write #1
    for (std::uint64_t i = 0; i < interval_count; ++i, tick += 2)
    {
        Check(coord.OnPublish(tick) == lockstep::PublishOutcome::kIdle,
              "exchange opened (fast-forward)");
        const std::uint32_t seq = wc.Write(&coord);
        FeedAck(&coord, static_cast<std::uint32_t>(tick), seq);
        Check(coord.OnPublish(tick) == lockstep::PublishOutcome::kStepGranted,
              "exchange granted (fast-forward)");
        coord.NotifyCommandApplied();
        coord.NotifyStepCompleted(tick + 2);
        if (coord.FailedClosed()) break;
    }
    Check(tick - 2 >= 80000, "fast-forward crossed the 80 s boundary");
    Check(!coord.FailedClosed(), "no fail-closed past 80 s of ticks");
    Check(coord.ViolationCount() == 0, "zero violations past 80 s");
    Check(coord.IntervalCount() == interval_count + 1,
          "all intervals recorded past 80 s");

    // A late-tick handoff (barrier at 90000 ms > 80 s) must behave
    // identically.
    lockstep::Coordinator::Config cfg2;
    cfg2.exchange_timeout_s = 60.0;
    cfg2.step_wait_timeout_s = 60.0;
    cfg2.trace_path = "/tmp/lockstep_test_late.csv";
    lockstep::Coordinator coord2(cfg2);
    coord2.SetFailClosedHandler([]() {});
    FeedCommand(&coord2);
    Check(coord2.OnStartupPublish(90000), "barrier at 90000 ms");
    Check(coord2.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "late handoff step granted");
    coord2.NotifyStepCompleted(90002);
    Check(coord2.OnPublish(90002) == lockstep::PublishOutcome::kIdle,
          "late exchange opened");
    FeedAck(&coord2, 90002, 2);
    FeedCommand(&coord2);
    Check(coord2.OnPublish(90002) == lockstep::PublishOutcome::kStepGranted,
          "late exchange completes");
    Check(!coord2.FailedClosed(), "late-tick run does not fail closed");
    Check(coord2.ViolationCount() == 0, "late-tick run has zero violations");
}

// Order-107: state_seq (uint32 wire) crossing the 2^32 ms wrap must resolve
// exactly; a late ack for the just-consumed state across the wrap is
// tolerated and the protocol stays exact on both sides of the boundary.
void TestWrapStateSeqAt2To32()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 60.0;
    cfg.step_wait_timeout_s = 60.0;
    cfg.trace_path.clear();
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    const std::uint64_t base = (1ULL << 32) - 4; // 4294967292 ms
    FeedCommand(&coord);
    Check(coord.OnStartupPublish(base), "barrier just below 2^32 ms");
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "handoff step granted");
    coord.NotifyStepCompleted(base + 2);
    Check(coord.OnPublish(base + 2) == lockstep::PublishOutcome::kIdle,
          "exchange opened below the wrap");

    // The acked state_seq wire still fits uint32 here (4294967294).
    FeedAck(&coord, static_cast<std::uint32_t>(base + 2), 2);
    FeedCommand(&coord);
    Check(coord.OnPublish(base + 2) == lockstep::PublishOutcome::kStepGranted,
          "exchange completes below the wrap");
    coord.NotifyCommandApplied();
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "step granted across the wrap");
    coord.NotifyStepCompleted(base + 4); // 2^32 ms: wire state wraps to 0

    // Late ack for the just-consumed state across the wrap: tolerated.
    FeedAck(&coord, static_cast<std::uint32_t>(base + 2), 2);
    Check(!coord.FailedClosed(),
          "late ack for the consumed state across the wrap tolerated");

    Check(coord.OnPublish(base + 4) == lockstep::PublishOutcome::kIdle,
          "exchange opened past the wrap (wire state_seq = 0)");
    FeedAck(&coord, static_cast<std::uint32_t>(base + 4), 3); // wire = 0
    FeedCommand(&coord);
    Check(coord.OnPublish(base + 4) == lockstep::PublishOutcome::kStepGranted,
          "exchange completes past the 2^32 state wrap");
    Check(!coord.FailedClosed(), "state wrap run does not fail closed");
    Check(coord.ViolationCount() == 0, "state wrap has zero violations");
}

// Order-107: command_seq (uint32 wire) crossing the 2^32 write wrap must
// resolve exactly against the exchange window; a wrapped seq waits for its
// own arrival and matches the correct ordinal.
void TestWrapCommandSeq()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 60.0;
    cfg.step_wait_timeout_s = 60.0;
    cfg.trace_path.clear();
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    // Seed the local arrival counter so the next write lands exactly at the
    // uint32 wrap boundary (2^32 writes), without injecting 2^32 messages.
    coord.SetCmdSeqBaseForTest((1ULL << 32) - 2);
    FeedCommand(&coord); // arrival #(2^32 - 1) completes the barrier
    Check(coord.OnStartupPublish(0), "barrier complete");
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "handoff step granted");
    coord.NotifyStepCompleted(2);
    Check(coord.OnPublish(2) == lockstep::PublishOutcome::kIdle,
          "exchange opened (window base = 2^32 - 1)");

    // The controller's 2^32-th write has wire command_seq 0: ack first, its
    // command arrives later (ack-before-own-command across the wrap).
    FeedAck(&coord, 2, 0);
    Check(!coord.FailedClosed(), "wrapped ack is not an immediate violation");
    Check(coord.OnPublish(2) == lockstep::PublishOutcome::kIdle,
          "wrapped ack waits for its exact arrival");
    FeedCommand(&coord); // arrival #2^32: ordinal mod 2^32 == 0
    Check(coord.OnPublish(2) == lockstep::PublishOutcome::kStepGranted,
          "wrapped command_seq matches its exact arrival");
    coord.NotifyCommandApplied();
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "step granted");
    coord.NotifyStepCompleted(4);
    Check(coord.OnPublish(4) == lockstep::PublishOutcome::kIdle,
          "next exchange opened");

    // The controller's (2^32+1)-th write has wire command_seq 1.
    FeedAck(&coord, 4, 1);
    FeedCommand(&coord); // arrival #(2^32+1)
    Check(coord.OnPublish(4) == lockstep::PublishOutcome::kStepGranted,
          "command_seq continues past the wrap");
    Check(!coord.FailedClosed(), "command wrap run does not fail closed");
    Check(coord.ViolationCount() == 0, "command wrap has zero violations");
}

// Order-107: an ack whose command_seq resolves to an older ordinal (the
// barrier command itself, just before the window) across the wrap boundary
// is a stale/ambiguous pair and fails closed immediately.
void TestCommandSeqStaleAtWrapFailClosed()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 60.0;
    cfg.step_wait_timeout_s = 60.0;
    cfg.trace_path.clear();
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    coord.SetCmdSeqBaseForTest((1ULL << 32) - 2);
    FeedCommand(&coord); // arrival #(2^32 - 1) completes the barrier
    Check(coord.OnStartupPublish(0), "barrier complete");
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "handoff step granted");
    coord.NotifyStepCompleted(2);
    Check(coord.OnPublish(2) == lockstep::PublishOutcome::kIdle,
          "exchange opened");

    // ack{2, 2^32-1}: the wire seq 2^32-1 is the barrier command's ordinal,
    // exactly one behind the exchange window -> stale -> fail closed.
    FeedAck(&coord, 2, 0xFFFFFFFFu);
    Check(coord.FailedClosed(),
          "stale command_seq across the wrap fails closed");
    Check((coord.ViolationCount() & lockstep::kViolationAckCmdMismatch) != 0,
          "command mismatch violation flag set");
}

// Order-107: uint32 modular resolution helpers are exact at the wrap
// boundary and fail closed on half-space ambiguity.
void TestModularResolutionHelpers()
{
    std::uint64_t v = 0;
    Check(lockstep::Coordinator::ResolveStateSeq(4, 4, &v) == 1 && v == 4,
          "current state resolves exactly");
    Check(lockstep::Coordinator::ResolveStateSeq(6, 4, &v) == 1 && v == 6,
          "future state resolves forward");
    Check(lockstep::Coordinator::ResolveStateSeq(2, 4, &v) == 1 && v == 2,
          "stale state resolves backward");
    Check(lockstep::Coordinator::ResolveStateSeq(
              static_cast<std::uint32_t>(4 + (1u << 31)), 4, &v) == 0,
          "exact half-space state is ambiguous");
    Check(lockstep::Coordinator::ResolveStateSeq(0, 4294967298ULL, &v) == 1 &&
              v == 4294967296ULL,
          "wire state 0 near 2^32+2 resolves to the prior exchange");

    Check(lockstep::Coordinator::ResolveCommandSeq(3, 1, 3, &v) == 1 &&
              v == 3,
          "exact arrival resolves");
    Check(lockstep::Coordinator::ResolveCommandSeq(4, 1, 3, &v) == 0,
          "not-yet-arrived seq is pending");
    Check(lockstep::Coordinator::ResolveCommandSeq(1, 1, 3, &v) == -1,
          "pre-open ordinal is stale/ambiguous");
    Check(lockstep::Coordinator::ResolveCommandSeq(
              0, 4294967295ULL, 4294967296ULL, &v) == 1 &&
              v == 4294967296ULL,
          "wrapped command_seq matches its exact ordinal");
    Check(lockstep::Coordinator::ResolveCommandSeq(
              0xFFFFFFFFu, 4294967295ULL, 4294967296ULL, &v) == -1,
          "stale command_seq across the wrap is ambiguous");
}

void TestDtMsOverride()
{
    lockstep::Coordinator::Config cfg;
    lockstep::Coordinator coord(cfg);
    coord.SetDtMs(4);
    Check(coord.DtMs() == 4, "SetDtMs overrides the interval length");
    coord.SetDtMs(0);
    Check(coord.DtMs() == 4, "SetDtMs(0) keeps the previous interval length");
}

} // namespace

int main()
{
    TestBarrierTimeout();
    TestExchangeTimeout();
    TestCommandArrivalTimeout();
    TestAckSeqMisalignmentTimesOut();
    TestStepTimeout();
    TestPhysicsTickMismatchFailClosed();
    TestRepublishTickMismatchFailClosed();
    TestStartupWatchdog();
    TestAbort();
    TestCanaryLateAckReproduction();
    Test1000HzRepublishSameSeq();
    Test500HzControllerDoubleWriteDupIdempotent();
    TestTrueStaleAckFailClosed();
    TestFutureAckFailClosed();
    TestReorderAckFailClosed();
    TestStartupAckIgnored();
    TestCallbackOrderAckBeforeCommand();
    TestCallbackOrderCommandBeforeAck();
    TestP2OldPostPublishNeverSteps();
    TestP2ReverseOrderCommandBeforeAck();
    TestEpochAlignmentMissedPreBarrier();
    TestDuplicateAckOneStep();
    TestOneStepPerExchange();
    TestFullProtocolTrace();
    TestWrapBeyond80s();
    TestWrapStateSeqAt2To32();
    TestWrapCommandSeq();
    TestCommandSeqStaleAtWrapFailClosed();
    TestModularResolutionHelpers();
    TestDtMsOverride();

    if (g_failures == 0)
    {
        std::printf("test_lockstep: all tests passed\n");
        return 0;
    }
    std::fprintf(stderr, "test_lockstep: %d failure(s)\n", g_failures);
    return 1;
}
