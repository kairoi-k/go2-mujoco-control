// Order-109 state-synchronous motion clock tests.

#include <cmath>
#include <cstdio>
#include <cstdint>

#include "lockstep_motion_clock.h"

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

void TestPreHandoffAndFlagOffRemainInactive()
{
    lockstep_motion::StateSynchronousClock clock;
    double dt = -1.0;
    Check(!clock.Engaged(), "pre-handoff clock is inactive");
    Check(!clock.Step(2, dt), "pre-handoff does not override wall-clock path");
    Check(dt == 0.0, "inactive clock supplies no elapsed time");
    // The controller selects this seam only when both lockstep ack and the
    // writer gate are active; flag-off therefore follows its existing path.
    Check(!clock.Step(4, dt), "flag-off equivalent remains inactive");
}

void TestPostHandoffUsesExactStateDelta()
{
    lockstep_motion::StateSynchronousClock clock;
    clock.Engage(100);
    double dt = 0.0;
    Check(clock.Step(102, dt), "first post-handoff state is accepted");
    Check(std::fabs(dt - 0.002) < 1e-15,
          "post-handoff motion_dt equals the 2 ms state delta");
    Check(clock.Step(104, dt), "next consecutive state is accepted");
    Check(std::fabs(dt - 0.002) < 1e-15,
          "every consecutive state advances by 2 ms");
}

void TestCommandTimeSlopeAndHandoffContinuity()
{
    lockstep_motion::StateSynchronousClock clock;
    double cmd_time = 1.237; // wall-clock phase immediately before handoff
    const double before_handoff = cmd_time;
    clock.Engage(200); // rebase, without resetting command time
    double dt = 0.0;
    const bool first = clock.Step(202, dt);
    cmd_time += dt;
    Check(first && std::fabs(cmd_time - (before_handoff + 0.002)) < 1e-15,
          "handoff is continuous with one nonzero time advance");
    const double sim_time_1 = 0.202;
    const double cmd_time_1 = cmd_time;
    Check(clock.Step(204, dt), "second post-handoff state accepted");
    cmd_time += dt;
    const double sim_time_2 = 0.204;
    Check(std::fabs((cmd_time - cmd_time_1) /
                        (sim_time_2 - sim_time_1) - 1.0) < 1e-12,
          "cmd_time slope versus sim time is exactly 1");
}

void TestNonConsecutiveTickDoesNotAdvance()
{
    lockstep_motion::StateSynchronousClock clock;
    clock.Engage(300);
    double dt = 0.0;
    Check(!clock.Step(306, dt), "gapped state is rejected");
    Check(dt == 0.0, "gapped state does not advance motion time");
    Check(clock.Step(302, dt), "clock remains anchored after rejected state");
    Check(std::fabs(dt - 0.002) < 1e-15,
          "recovery uses the exact anchored consecutive delta");
}

void TestTickWrap()
{
    lockstep_motion::StateSynchronousClock clock;
    clock.Engage(0xFFFFFFFEu);
    double dt = 0.0;
    Check(clock.Step(0, dt), "exact 2 ms state delta across uint32 wrap");
    Check(std::fabs(dt - 0.002) < 1e-15,
          "wrapped state delta remains 2 ms");
}
} // namespace

int main()
{
    TestPreHandoffAndFlagOffRemainInactive();
    TestPostHandoffUsesExactStateDelta();
    TestCommandTimeSlopeAndHandoffContinuity();
    TestNonConsecutiveTickDoesNotAdvance();
    TestTickWrap();
    if (g_failures != 0)
    {
        std::fprintf(stderr, "lockstep_motion_clock: %d failure(s)\n",
                     g_failures);
        return 1;
    }
    std::printf("lockstep_motion_clock: all checks passed\n");
    return 0;
}
