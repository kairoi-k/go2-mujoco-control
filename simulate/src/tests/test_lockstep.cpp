// Unit/integration tests for the Order-103/105/106 sim-time lockstep
// coordinator (simulate/src/lockstep.h). The coordinator is DDS/MuJoCo-free;
// tests drive the ready/publish/exchange/tick/ack protocol directly and
// assert the invariants the simulator integration relies on:
//   * ready barrier completes only after the first controller command;
//   * one frozen interval per causal exchange; exact dt tick sequence
//     (no duplicate/missing/reordered tick);
//   * Order-106 frozen republish: repeated 1000 Hz publishes of one frozen
//     state carry the same state_seq and never fail closed; the exchange
//     completes only when a new LowCmd arrived after the FIRST publish AND a
//     matching ack{state_seq == published tick} was received;
//   * Order-106 canary reproduction (C-006e at 1b29974): a late ack for the
//     just-consumed state (delivered after the sim advanced) is idempotent
//     instead of failing closed as stale (violations=32 in the canary);
//   * duplicate acks idempotent; TRUE stale (older than the last consumed
//     state) and future/reordered acks fail closed with the expected
//     violation flags; startup-phase acks are ignored;
//   * both callback orders (ack-before-command and command-before-ack)
//     complete the exchange;
//   * tick space beyond 80 s (tick > 80000) has no wrap/truncation effects;
//   * external stop aborts waits without failing closed;
//   * repeated identical event scripts yield an identical deterministic
//     trace (stable columns), wall-timing columns excluded.

#include <atomic>
#include <chrono>
#include <cstdio>
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

// Ack for a frozen state (Order-106: state_seq only).
void FeedAck(lockstep::Coordinator *coord, std::uint64_t state_seq)
{
    coord->OnAckReceived(state_seq);
}

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
// `start_tick` with scripted 1:1 command/ack events (both the ack and the
// command are injected between the first publish and the completion check,
// covering the command-before-ack callback order on the open exchange).
// Returns the trace text; events are scripted so the run is reproducible.
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
    FeedCommand(&coord);
    Check(coord.OnStartupPublish(start_tick),
          "barrier completes on first command");
    Check(coord.BarrierComplete(), "BarrierComplete() true after barrier");

    Check(driver.Step(start_tick + 2) == lockstep::WaitOutcome::kReady,
          "first physics step granted");

    std::uint64_t tick = start_tick + 2;
    for (int i = 0; i < interval_count; ++i)
    {
        // First publish of the frozen state opens the exchange.
        Check(driver.Publish(tick) == lockstep::PublishOutcome::kIdle,
              "exchange opened on first publish");
        FeedCommand(&coord);
        FeedAck(&coord, tick);
        // Next publish (republish of the same frozen state) evaluates the
        // exchange.
        Check(driver.Publish(tick) == lockstep::PublishOutcome::kStepGranted,
              "exchange completes on the matching ack + new command");
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

    explicit ExchangeScenario(lockstep::Coordinator *c,
                              std::uint64_t start_tick)
        : coord(c), tick(start_tick)
    {
        FeedCommand(coord); // first controller command -> barrier
        Check(coord->OnStartupPublish(tick - 2),
              "barrier completes in ExchangeScenario");
        Check(coord->WaitForStepPermission() == lockstep::WaitOutcome::kReady,
              "first step granted in ExchangeScenario");
        coord->NotifyStepCompleted(tick);
        Check(coord->OnPublish(tick) == lockstep::PublishOutcome::kIdle,
              "first publish opens the exchange in ExchangeScenario");
    }

    // Completes the exchange for the current frozen state.
    bool CompleteExchange()
    {
        FeedAck(coord, tick);
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
    FeedAck(&coord, 2); // ack arrives but the acked command never does
    std::this_thread::sleep_for(std::chrono::milliseconds(350));
    Check(coord.OnPublish(2) == lockstep::PublishOutcome::kIdle,
          "ack without a new command never grants a step");
    Check(coord.FailedClosed(), "ack-without-command fails closed");
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
// completes only on ack{state_seq == published} plus a new LowCmd.
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
    // 2300 when the sim advanced; its ack{2300} arrives after 2302 was
    // published. Order-105 classified this as stale and failed closed.
    FeedAck(&coord, 2300);
    Check(!coord.FailedClosed(),
          "late ack for the just-consumed state is tolerated");
    FeedAck(&coord, 2300); // a second identical late ack is also idempotent
    Check(!coord.FailedClosed(), "duplicate late ack is idempotent");
    Check(coord.OnPublish(2302) == lockstep::PublishOutcome::kIdle,
          "late ack alone never grants a step");

    // The controller consumes 2302: matching ack + new LowCmd complete.
    FeedAck(&coord, 2302);
    FeedCommand(&coord);
    Check(coord.OnPublish(2302) == lockstep::PublishOutcome::kStepGranted,
          "exchange for 2302 completes on matching ack + new command");
    coord.NotifyCommandApplied();
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "step to 2304 granted");
    coord.NotifyStepCompleted(2304);
    Check(coord.OnPublish(2304) == lockstep::PublishOutcome::kIdle,
          "exchange for 2304 opened");

    // Late ack for the just-consumed state 2302 after the sim advanced:
    // tolerated (== last consumed), never fails closed.
    FeedAck(&coord, 2302);
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

// Order-106: a 500 Hz controller may write 1-2 LowCmds per frozen state and
// ack it more than once; duplicate acks (while the state is current or after
// the sim advanced to the next state) are idempotent.
void Test500HzControllerDoubleWriteDupIdempotent()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 5.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    ExchangeScenario sc(&coord, 4);
    // First 500 Hz write on the frozen state 4: matching ack + command.
    Check(sc.CompleteExchange(), "first write completes the exchange");
    coord.NotifyCommandApplied();
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "step to 6 granted");
    coord.NotifyStepCompleted(6);
    Check(coord.OnPublish(6) == lockstep::PublishOutcome::kIdle,
          "exchange for 6 opened");

    // Second 500 Hz write was still computed on state 4 (the controller had
    // not received state 6 yet): its ack{4} duplicates the just-consumed
    // state and its command is a fresh arrival.
    FeedAck(&coord, 4);
    FeedCommand(&coord);
    Check(!coord.FailedClosed(), "duplicate ack for consumed state tolerated");
    FeedAck(&coord, 6);
    FeedCommand(&coord);
    Check(coord.OnPublish(6) == lockstep::PublishOutcome::kStepGranted,
          "exchange for 6 completes normally");
    Check(!coord.FailedClosed(), "no fail-closed on controller double-write");
    Check(coord.ViolationCount() == 0, "double-write has zero violations");
}

// Order-106: an ack older than the last consumed state is a TRUE stale ack
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
    FeedAck(&coord, 2);
    Check(!coord.FailedClosed(), "ack for just-consumed state tolerated");
    // ack{0} (== barrier state, older than the last consumed state): TRUE
    // stale -> fail closed.
    FeedAck(&coord, 0);
    Check(coord.FailedClosed(), "true stale ack fails closed");
    Check((coord.ViolationCount() & lockstep::kViolationAckStale) != 0,
          "stale ack violation flag set");
}

// Order-106: an ack for a state the sim has not published (future, e.g. a
// reordered ack of the next state) fails closed.
void TestFutureAckFailClosed()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 2.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    ExchangeScenario sc(&coord, 4);
    FeedAck(&coord, 8); // ack for a future state (tick 8)
    Check(coord.FailedClosed(), "future ack fails closed");
    Check((coord.ViolationCount() & lockstep::kViolationAckFuture) != 0,
          "future ack violation flag set");
}

// Order-106: reordered ack (the next state's ack arriving first) is a future
// ack and fails closed.
void TestReorderAckFailClosed()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 2.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    ExchangeScenario sc(&coord, 4);
    FeedAck(&coord, 6); // next state's ack arrives first
    Check(coord.FailedClosed(), "reordered ack fails closed");
    Check((coord.ViolationCount() & lockstep::kViolationAckFuture) != 0,
          "reordered ack recorded as future violation");
}

// Order-106: startup-phase acks (before the ready barrier or referencing
// pre-handoff states) are ignored, not validated.
void TestStartupAckIgnored()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 1.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    // Acks before the barrier (ack_validation inactive): ignored.
    FeedAck(&coord, 999);
    Check(!coord.FailedClosed(), "pre-barrier ack ignored");
    Check(coord.ViolationCount() == 0, "pre-barrier ack no violation");

    FeedCommand(&coord);
    Check(coord.OnStartupPublish(0), "barrier complete");
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "step 1 granted");
    coord.NotifyStepCompleted(2);
    // A pre-handoff state's ack (startup wall-clock state) is ignored.
    FeedAck(&coord, 0);
    Check(!coord.FailedClosed(), "pre-handoff ack ignored");
    Check(coord.OnPublish(2) == lockstep::PublishOutcome::kIdle,
          "exchange for 2 opened");
    FeedAck(&coord, 2);
    FeedCommand(&coord);
    Check(coord.OnPublish(2) == lockstep::PublishOutcome::kStepGranted,
          "exchange completes after the ignored ack");
}

// Order-106: both callback orders complete the exchange exactly once.
void TestCallbackOrderAckBeforeCommand()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 1.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    ExchangeScenario sc(&coord, 4);
    FeedAck(&coord, 4);   // ack first (cross-topic reorder)
    FeedAck(&coord, 4);   // duplicate ack while current: idempotent
    Check(!coord.FailedClosed(), "duplicate matching ack idempotent");
    FeedCommand(&coord);  // command arrives after the acks
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
    FeedCommand(&coord);  // command first
    Check(coord.OnPublish(4) == lockstep::PublishOutcome::kIdle,
          "command alone does not grant a step");
    FeedAck(&coord, 4);   // then the matching ack
    Check(coord.OnPublish(4) == lockstep::PublishOutcome::kStepGranted,
          "command-before-ack completes the exchange");
    Check(!coord.FailedClosed(), "command-before-ack path does not fail");
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
        }
        else
        {
            Check(ack_state_seq == tick,
                  "ack state_seq equals the published tick");
            Check(ack_cmd_seq == 0,
                  "ack command_seq is no longer carried (always 0)");
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

// Order-106: tick space beyond 80 s (tick > 80000 at 2 ms dt) must not wrap
// or truncate. The wire carries state_seq as uint32_t (Error_.source(),
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
    for (std::uint64_t i = 0; i < interval_count; ++i, tick += 2)
    {
        Check(coord.OnPublish(tick) == lockstep::PublishOutcome::kIdle,
              "exchange opened (fast-forward)");
        FeedAck(&coord, tick);
        FeedCommand(&coord);
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
    FeedAck(&coord2, 90002);
    FeedCommand(&coord2);
    Check(coord2.OnPublish(90002) == lockstep::PublishOutcome::kStepGranted,
          "late exchange completes");
    Check(!coord2.FailedClosed(), "late-tick run does not fail closed");
    Check(coord2.ViolationCount() == 0, "late-tick run has zero violations");
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
    TestFullProtocolTrace();
    TestWrapBeyond80s();
    TestDtMsOverride();

    if (g_failures == 0)
    {
        std::printf("test_lockstep: all tests passed\n");
        return 0;
    }
    std::fprintf(stderr, "test_lockstep: %d failure(s)\n", g_failures);
    return 1;
}
