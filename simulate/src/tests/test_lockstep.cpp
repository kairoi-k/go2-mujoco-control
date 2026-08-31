// Unit tests for the Order-103 sim-time lockstep coordinator
// (simulate/src/lockstep.h). The coordinator is DDS/MuJoCo-free; tests drive
// the ready/exchange/tick protocol directly and assert the invariants that
// the simulator integration relies on:
//   * ready barrier completes only after the first controller command;
//   * one frozen interval per exchange; exact dt tick sequence
//     (no duplicate/missing/reordered tick);
//   * timeouts fail closed with the expected violation flags;
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

// Feeds N command arrivals on a background thread, spaced apart.
void FeedCommands(lockstep::Coordinator *coord, int count, int spacing_us)
{
    for (int i = 0; i < count; ++i)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(spacing_us));
        coord->OnCommandArrived();
    }
}

struct ProtocolDriver
{
    explicit ProtocolDriver(lockstep::Coordinator *c) : coord(c) {}

    // Bridge role for one interval: wait for the physics step, publish
    // state `tick`, wait for the exchange, grant the next step.
    lockstep::WaitOutcome BridgeInterval(std::uint64_t tick)
    {
        lockstep::WaitOutcome w = coord->WaitForStepCompleted();
        if (w != lockstep::WaitOutcome::kReady) return w;
        lockstep::ExchangeTrigger trigger;
        w = coord->WaitForExchange(tick, &trigger);
        if (w != lockstep::WaitOutcome::kReady) return w;
        if (trigger != lockstep::ExchangeTrigger::kArrivalCount)
        {
            Check(false, "exchange trigger must be kArrivalCount");
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
// `start_tick` (the sim tick the first frozen step starts from, i.e. the
// handoff tick recorded by the barrier row). Returns the trace text;
// command arrivals are scripted so the run is reproducible.
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
    Check(!coord.OnStartupPublish(), "barrier not complete before first cmd");
    coord.OnCommandArrived();
    Check(coord.OnStartupPublish(), "barrier completes on first command");
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

    // Bridge role: every exchange needs one fresh-command feeder so
    // WaitForExchange completes on kArrivalCount (1:1 controller/sim clock).
    for (int i = 1; i <= interval_count; ++i)
    {
        std::thread feeder([&coord]() { FeedCommands(&coord, 1, 500); });
        Check(driver.BridgeInterval(
                  start_tick + 2u * static_cast<std::uint64_t>(i)) ==
                  lockstep::WaitOutcome::kReady,
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

    coord.OnCommandArrived();
    Check(coord.OnStartupPublish(), "barrier complete");
    Check(coord.WaitForStepPermission() == lockstep::WaitOutcome::kReady,
          "step 1 granted");
    coord.NotifyStepCompleted(2);

    lockstep::WaitOutcome outcome = lockstep::WaitOutcome::kReady;
    std::thread t([&]() {
        lockstep::ExchangeTrigger trigger;
        outcome = coord.WaitForExchange(2, &trigger);
    });
    t.join(); // no commands arrive -> exchange timeout

    Check(outcome == lockstep::WaitOutcome::kTimeoutFailClosed,
          "exchange timeout returns kTimeoutFailClosed");
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

    coord.OnCommandArrived();
    Check(coord.OnStartupPublish(), "barrier complete");
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

    coord.OnCommandArrived();
    Check(coord.OnStartupPublish(), "barrier complete");
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
        if (!coord.OnStartupPublish())
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
        Check(cols.size() == 9, "trace row has 9 columns");
        if (cols.size() != 9) continue;
        const std::uint64_t tick = std::stoull(cols[0]);
        const std::uint64_t step_index = std::stoull(cols[1]);
        const int trigger = std::stoi(cols[7]);
        const std::uint32_t violations = std::stoul(cols[8]);
        Check(tick == expected_tick,
              "tick sequence is exact (no gap/dup/reorder)");
        Check(step_index == expected_index, "step index is sequential");
        Check(trigger == (rows == 0 ? 0 : 1),
              "barrier row uses kBarrier, others kArrivalCount");
        Check(violations == 0, "no violations in happy-path rows");
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
            if (cols.size() != 9) continue;
            // sim_tick_ms, step_index, phase, cmd_seq_at_publish,
            // cmd_seq_at_ready, [wait_us], [wall_us], trigger, violations
            out << cols[0] << "," << cols[1] << "," << cols[2] << ","
                << cols[3] << "," << cols[4] << "," << cols[7] << ","
                << cols[8] << "\n";
        }
        return out.str();
    };

    // Wall-timing columns differ between runs by design; the deterministic
    // columns (tick, step, phase, command sequences, trigger, violations)
    // must match exactly across repeated identical scripts.
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
