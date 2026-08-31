// Unit tests for the Order-103/105 sim-time lockstep coordinator
// (simulate/src/lockstep.h). The coordinator is DDS/MuJoCo-free; tests drive
// the ready/exchange/tick/ack protocol directly and assert the invariants
// that the simulator integration relies on:
//   * ready barrier completes only after the first controller command;
//   * one frozen interval per causal exchange; exact dt tick sequence
//     (no duplicate/missing/reordered tick);
//   * Order-105: a step is granted only for a matching newer LowCmd plus
//     ack{state_seq == published tick, command_seq consistent}; stale /
//     future / duplicate / reordered / missing acks, command mismatches and
//     timeouts fail closed with the expected violation flags;
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
std::atomic<std::uint64_t> g_cmd_count{0};

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

// One LowCmd arrival; mirrors the coordinator's internal command counter.
void FeedCommand(lockstep::Coordinator *coord)
{
    coord->OnCommandArrived();
    g_cmd_count.fetch_add(1, std::memory_order_relaxed);
}

// Blocks until WaitForExchange has captured the publish-time command count
// (so events injected afterwards are guaranteed to be observed as "newer"
// than the publish).
void WaitExchangeActive(lockstep::Coordinator *coord)
{
    while (!coord->ExchangeActive())
        std::this_thread::sleep_for(std::chrono::microseconds(100));
}

// Ack with the current arrival count (a real adapter counts every write the
// same way, so the sim-side arrival count and the ack command_seq align).
void FeedAck(lockstep::Coordinator *coord, std::uint64_t state_seq)
{
    coord->OnAckReceived(
        state_seq, g_cmd_count.load(std::memory_order_relaxed));
}

struct ProtocolDriver
{
    explicit ProtocolDriver(lockstep::Coordinator *c) : coord(c) {}

    // Bridge role for one interval: wait for the physics step, register the
    // published state tick, wait for the causal exchange (one fresh command
    // plus its ack), grant the next step.
    lockstep::WaitOutcome BridgeInterval(std::uint64_t tick)
    {
        lockstep::WaitOutcome w = coord->WaitForStepCompleted();
        if (w != lockstep::WaitOutcome::kReady) return w;
        coord->OnStatePublished(tick);
        lockstep::ExchangeTrigger trigger;
        w = coord->WaitForExchange(tick, &trigger);
        if (w != lockstep::WaitOutcome::kReady) return w;
        if (trigger != lockstep::ExchangeTrigger::kAckMatched)
        {
            Check(false, "exchange trigger must be kAckMatched");
        }
        coord->NotifyCommandApplied();
        return lockstep::WaitOutcome::kReady;
    }

    // Physics role: one step per granted interval.
    lockstep::WaitOutcome PhysicsStep(std::uint64_t next_tick)
    {
        lockstep::WaitOutcome w = coord->WaitForStepPermission();
        if (w != lockstep::WaitOutcome::kReady) return w;
        coord->NotifyStepCompleted(next_tick);
        return lockstep::WaitOutcome::kReady;
    }

    lockstep::Coordinator *coord;
};

// Runs the full protocol for `interval_count` intervals starting at
// `start_tick`. Returns the trace text; command/ack events are scripted so
// the run is reproducible.
std::string RunFullProtocol(int interval_count, const std::string &trace_path,
                            std::uint64_t start_tick = 0)
{
    g_cmd_count.store(0, std::memory_order_relaxed);
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

    std::thread physics([&]() {
        Check(driver.PhysicsStep(start_tick + 2) ==
                  lockstep::WaitOutcome::kReady,
              "first physics step granted");
        for (int i = 1; i < interval_count; ++i)
        {
            Check(driver.PhysicsStep(
                      start_tick +
                      2u * static_cast<std::uint64_t>(i + 1)) ==
                      lockstep::WaitOutcome::kReady,
                  "physics step granted for every interval");
        }
    });

    // Bridge role: every exchange needs one fresh command plus a matching
    // ack so WaitForExchange completes on kAckMatched (1:1 controller/sim).
    for (int i = 1; i <= interval_count; ++i)
    {
        const std::uint64_t tick =
            start_tick + 2u * static_cast<std::uint64_t>(i);
        std::thread feeder([&coord, tick]() {
            WaitExchangeActive(&coord);
            FeedCommand(&coord);
            FeedAck(&coord, tick);
        });
        Check(driver.BridgeInterval(tick) == lockstep::WaitOutcome::kReady,
              "exchange ready for every interval");
        feeder.join();
    }
    physics.join();

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
// first step, publish registration. The exchange wait itself runs on a
// background thread so the test can inject command/ack events while it
// blocks.
struct AckScenario
{
    lockstep::Coordinator *coord;
    std::uint64_t tick = 0;
    lockstep::WaitOutcome outcome = lockstep::WaitOutcome::kReady;
    lockstep::ExchangeTrigger trigger = lockstep::ExchangeTrigger::kBarrier;
    std::thread waiter;

    explicit AckScenario(lockstep::Coordinator *c,
                         std::uint64_t start_tick)
        : coord(c), tick(start_tick)
    {
        g_cmd_count.store(0, std::memory_order_relaxed);
        FeedCommand(coord); // first controller command -> barrier
        Check(coord->OnStartupPublish(tick - 2),
              "barrier completes in AckScenario");
        Check(coord->WaitForStepPermission() ==
                  lockstep::WaitOutcome::kReady,
              "first step granted in AckScenario");
        coord->NotifyStepCompleted(tick);
        coord->OnStatePublished(tick);
    }

    void StartWait()
    {
        waiter = std::thread([this]() {
            outcome = coord->WaitForExchange(tick, &trigger);
        });
        // Wait until the publish-time command count is captured before the
        // test injects command/ack events.
        WaitExchangeActive(coord);
    }

    // Re-arms the scenario for the next interval without the barrier.
    void NextExchange(std::uint64_t next_tick)
    {
        tick = next_tick;
        coord->OnStatePublished(tick);
        outcome = lockstep::WaitOutcome::kReady;
        trigger = lockstep::ExchangeTrigger::kBarrier;
        waiter = std::thread([this]() {
            outcome = coord->WaitForExchange(tick, &trigger);
        });
        WaitExchangeActive(coord);
    }

    void Join()
    {
        if (waiter.joinable()) waiter.join();
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
    coord.OnStatePublished(2);

    lockstep::WaitOutcome outcome = lockstep::WaitOutcome::kReady;
    std::thread t([&]() {
        lockstep::ExchangeTrigger trigger;
        outcome = coord.WaitForExchange(2, &trigger);
    });
    t.join(); // no commands and no acks arrive -> exchange timeout

    Check(outcome == lockstep::WaitOutcome::kTimeoutFailClosed,
          "exchange timeout returns kTimeoutFailClosed");
    Check((coord.ViolationCount() & lockstep::kViolationAckMissing) != 0,
          "missing ack violation flag set on exchange timeout");
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

void TestTickGapFailClosed()
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
    coord.NotifyStepCompleted(2);

    // Publish tick 3 instead of expected 2 -> tick gap -> fail closed.
    lockstep::WaitOutcome outcome = lockstep::WaitOutcome::kReady;
    std::thread t([&]() {
        lockstep::ExchangeTrigger trigger;
        outcome = coord.WaitForExchange(3, &trigger);
    });
    t.join();
    Check(outcome == lockstep::WaitOutcome::kTimeoutFailClosed,
          "tick gap returns kTimeoutFailClosed");
    Check((coord.ViolationCount() & lockstep::kViolationTickGap) != 0,
          "tick gap violation flag set");
    Check(coord.FailedClosed(), "FailedClosed set after tick gap");
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

// Order-105: an ack whose state_seq is older than the published state (an
// in-flight command computed from an older state) fails closed and no step
// is granted.
void TestAckStaleFailClosed()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 1.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    AckScenario sc(&coord, 4);
    sc.StartWait(); // waiting for the state published at tick 4
    FeedCommand(&coord);
    FeedAck(&coord, 2); // ack for the OLD state (tick 2)
    sc.Join();

    Check(sc.outcome == lockstep::WaitOutcome::kTimeoutFailClosed,
          "stale ack fails closed (no step)");
    Check((coord.ViolationCount() & lockstep::kViolationAckStale) != 0,
          "stale ack violation flag set");
}

// Order-105: an ack for a state the sim has not published yet fails closed.
void TestAckFutureFailClosed()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 1.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    AckScenario sc(&coord, 4);
    sc.StartWait();
    FeedCommand(&coord);
    coord.OnAckReceived(8, 2); // ack for a future state (tick 8)
    sc.Join();

    Check(sc.outcome == lockstep::WaitOutcome::kTimeoutFailClosed,
          "future ack fails closed (no step)");
    Check((coord.ViolationCount() & lockstep::kViolationAckFuture) != 0,
          "future ack violation flag set");
}

// Order-105: two acks for the same published state fail closed (a controller
// that acks twice for one state breaks the 1:1 write contract).
void TestAckDuplicateFailClosed()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 1.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    AckScenario sc(&coord, 4);
    sc.StartWait();
    // Both acks for the same state arrive before any command: the first is
    // held pending, the second must fail closed (no step, no completion).
    coord.OnAckReceived(4, 2);
    coord.OnAckReceived(4, 3);
    FeedCommand(&coord); // too late: the run already failed closed
    sc.Join();

    Check(sc.outcome == lockstep::WaitOutcome::kTimeoutFailClosed,
          "duplicate ack fails closed (no step)");
    Check((coord.ViolationCount() & lockstep::kViolationAckDuplicate) != 0,
          "duplicate ack violation flag set");
}

// Order-105: out-of-order acks (a later state's ack arriving first) are
// caught as future relative to the exchange currently waiting.
void TestAckReorderFailClosed()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 1.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    AckScenario sc(&coord, 4);
    sc.StartWait();
    FeedCommand(&coord);
    coord.OnAckReceived(6, 2); // next state's ack arrives first (reordered)
    sc.Join();

    Check(sc.outcome == lockstep::WaitOutcome::kTimeoutFailClosed,
          "reordered ack fails closed (no step)");
    Check((coord.ViolationCount() & lockstep::kViolationAckFuture) != 0,
          "reordered ack recorded as future violation");
}

// Order-105: ack for the previous state arriving after the sim moved to the
// next state (delayed ack) fails closed.
void TestAckDelayedFailClosed()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 1.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    AckScenario sc(&coord, 4);
    sc.StartWait();
    FeedCommand(&coord);
    FeedAck(&coord, 4); // valid exchange for tick 4 completes
    sc.Join();
    Check(sc.outcome == lockstep::WaitOutcome::kReady,
          "first exchange ready with matching ack");
    Check(sc.trigger == lockstep::ExchangeTrigger::kAckMatched,
          "first exchange trigger is kAckMatched");

    // Next state published (tick 6); the delayed ack for tick 4 arrives
    // during its exchange wait.
    sc.NextExchange(6);
    FeedAck(&coord, 4); // delayed ack for the previous state
    sc.Join();

    Check(sc.outcome == lockstep::WaitOutcome::kTimeoutFailClosed,
          "delayed ack for previous state fails closed (no step)");
    Check((coord.ViolationCount() & lockstep::kViolationAckStale) != 0,
          "delayed ack recorded as stale violation");
}

// Order-105: ack whose command_seq is not newer than the publish cannot be
// matched to a command computed from the published state.
void TestAckCmdMismatchFailClosed()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 1.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    AckScenario sc(&coord, 4);
    FeedCommand(&coord); // a second command arrives before the exchange wait
    sc.StartWait();      // publish-time count is now 2
    // The ack's command_seq 2 is not newer than the publish count 2, so it
    // cannot reference a command computed from the published state.
    coord.OnAckReceived(4, 2);
    sc.Join();

    Check(sc.outcome == lockstep::WaitOutcome::kTimeoutFailClosed,
          "command mismatch fails closed (no step)");
    Check((coord.ViolationCount() & lockstep::kViolationAckCmdMismatch) != 0,
          "command mismatch violation flag set");
}

// Order-105: an ack for the current state whose command never arrives fails
// closed on exchange timeout.
void TestAckCmdTimeoutFailClosed()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 0.3;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    AckScenario sc(&coord, 4);
    sc.StartWait();
    // Valid state_seq, but the ack references a command (count+3) that is
    // never published, so the exchange can never complete.
    coord.OnAckReceived(4, g_cmd_count.load(std::memory_order_relaxed) + 3);
    sc.Join();

    Check(sc.outcome == lockstep::WaitOutcome::kTimeoutFailClosed,
          "missing acked command fails closed on timeout");
    Check((coord.ViolationCount() & lockstep::kViolationExchangeTimeout) != 0,
          "exchange timeout violation flag set");
}

// Order-105 happy paths: a valid ack may arrive before its LowCmd (cross-
// topic reorder) and still complete the exchange exactly once.
void TestAckHappyPaths()
{
    lockstep::Coordinator::Config cfg;
    cfg.exchange_timeout_s = 1.0;
    cfg.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord(cfg);
    coord.SetFailClosedHandler([]() {});

    AckScenario sc(&coord, 4);
    sc.StartWait();
    // ack references command_seq 2 (newer than the publish count 1) while
    // its LowCmd is still in flight; the exchange waits for the arrival.
    coord.OnAckReceived(4, 2);
    FeedCommand(&coord);
    sc.Join();
    Check(sc.outcome == lockstep::WaitOutcome::kReady,
          "ack-before-command completes the exchange");
    Check(sc.trigger == lockstep::ExchangeTrigger::kAckMatched,
          "ack-before-command trigger is kAckMatched");
    Check(!coord.FailedClosed(), "happy ack path does not fail closed");

    // The barrier command's ack (command_seq <= barrier_seq_) references a
    // startup-phase state; it belongs to the wall-clock boundary, not to any
    // exchange, and must be ignored rather than fail closed.
    lockstep::Coordinator::Config cfg3;
    cfg3.exchange_timeout_s = 1.0;
    cfg3.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord3(cfg3);
    coord3.SetFailClosedHandler([]() {});
    AckScenario sc3(&coord3, 4);
    sc3.StartWait();
    coord3.OnAckReceived(2, 1); // delayed barrier-command ack (startup tick)
    FeedCommand(&coord3);
    coord3.OnAckReceived(4, 2); // valid exchange ack
    sc3.Join();
    Check(sc3.outcome == lockstep::WaitOutcome::kReady,
          "barrier-command ack is ignored, exchange still completes");
    Check(!coord3.FailedClosed(), "barrier-command ack does not fail closed");

    // A stale ack arriving while the first exchange is still pending must
    // not be overwritten by a later valid one: it fails closed immediately.
    lockstep::Coordinator::Config cfg2;
    cfg2.exchange_timeout_s = 1.0;
    cfg2.step_wait_timeout_s = 1.0;
    lockstep::Coordinator coord2(cfg2);
    coord2.SetFailClosedHandler([]() {});
    AckScenario sc2(&coord2, 4);
    sc2.StartWait();
    coord2.OnAckReceived(2, 2); // stale
    coord2.OnAckReceived(4, 2); // later valid ack must not mask the stale one
    FeedCommand(&coord2);
    sc2.Join();
    Check(sc2.outcome == lockstep::WaitOutcome::kTimeoutFailClosed,
          "stale ack is not masked by a later valid ack");
    Check((coord2.ViolationCount() & lockstep::kViolationAckStale) != 0,
          "stale violation flag set despite later valid ack");
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
            Check(ack_cmd_seq ==
                      static_cast<std::uint64_t>(rows + 1),
                  "ack command_seq matches the interval command");
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
    // Handoff at a later tick (wall-clock startup lasted 1000 ms): barrier
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
    TestStepTimeout();
    TestTickGapFailClosed();
    TestStartupWatchdog();
    TestAbort();
    TestAckStaleFailClosed();
    TestAckFutureFailClosed();
    TestAckDuplicateFailClosed();
    TestAckReorderFailClosed();
    TestAckDelayedFailClosed();
    TestAckCmdMismatchFailClosed();
    TestAckCmdTimeoutFailClosed();
    TestAckHappyPaths();
    TestFullProtocolTrace();
    TestDtMsOverride();

    if (g_failures == 0)
    {
        std::printf("test_lockstep: all tests passed\n");
        return 0;
    }
    std::fprintf(stderr, "test_lockstep: %d failure(s)\n", g_failures);
    return 1;
}
