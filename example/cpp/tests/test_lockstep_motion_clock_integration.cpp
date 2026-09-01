// Order-109b: production TrotExperiment call-chain integration test.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include "trot_experiment.h"

namespace
{
int failures = 0;
void Check(bool ok, const char *what)
{
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", what); ++failures; }
}

unitree_go::msg::dds_::LowState_ State(std::uint32_t tick)
{
    unitree_go::msg::dds_::LowState_ state;
    state.tick(tick);
    // Match the production stand target so the real hard-limit path does not
    // intentionally stop this timing-only call-chain probe.
    const double q[12] = {
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763};
    for (int i = 0; i < 12; ++i) {
        state.motor_state()[i].q(q[i]);
        state.motor_state()[i].dq(0.0);
    }
    return state;
}

void TestProductionChain()
{
    go2_trot::TrotParams params;
    params.wall_clock_motion = true;
    // Primary mode bypasses the position-error safety witness in this
    // synthetic sensor-only probe; no WBC model is loaded or solved.
    params.wbc_full = true;
    params.wbc_primary = true;
    TrotExperiment experiment(
        10.0, "/tmp/order109b_controller.csv", params,
        1000, false, "", false);

    // Before handoff the real LowCmdWrite path retains wall-clock motion.
    Check(experiment.TestRunWallClockTick(State(100)),
          "pre-handoff LowCmdWrite executes");
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    Check(experiment.TestRunWallClockTick(State(100)),
          "pre-handoff duplicate state still uses wall-clock path");
    const auto pre = experiment.TestLastMotionClockSample();
    Check(pre.motion_dt_s > 0.001,
          "pre-handoff wall-clock motion_dt remains nonzero");

    // The production gate and LowCmdWrite are driven together.  The helper
    // only suppresses DDS output; MotionClockStep and all timer consumers
    // are the production implementations executed by LowCmdWrite.
    experiment.TestPrepareMotionClock(100);
    Check(experiment.TestRunLockstepTick(State(102)),
          "first accepted post-handoff LowCmdWrite executes");
    const auto first = experiment.TestLastMotionClockSample();
    Check(std::fabs(first.motion_dt_s - 0.002) < 1e-12,
          "production motion_dt is exactly 2 ms");
    Check(std::fabs(first.cmd_time_s - pre.cmd_time_s - 0.002) < 1e-12,
          "handoff cmd_time is continuous");
    Check(std::fabs(first.gait_time_s - pre.gait_time_s - 0.002) < 1e-12,
          "production gait timer advances once");

    // A repeated DDS publication is consumed by the handler but cannot run
    // the production writer, so no timer receives a second update.
    Check(!experiment.TestRunLockstepTick(State(102)),
          "duplicate tick does not invoke LowCmdWrite");
    const auto duplicate = experiment.TestLastMotionClockSample();
    Check(std::fabs(duplicate.cmd_time_s - first.cmd_time_s) < 1e-12,
          "duplicate does not advance cmd_time");
    Check(std::fabs(duplicate.gait_time_s - first.gait_time_s) < 1e-12,
          "duplicate does not advance gait timer");

    Check(experiment.TestRunLockstepTick(State(104)),
          "second accepted post-handoff LowCmdWrite executes");
    const auto second = experiment.TestLastMotionClockSample();
    Check(std::fabs(second.motion_dt_s - 0.002) < 1e-12,
          "each accepted state produces one 2 ms motion_dt");
    for (double delta : {second.cmd_time_s - first.cmd_time_s,
                         second.gait_time_s - first.gait_time_s,
                         second.ramp_time_s - first.ramp_time_s,
                         second.governor_time_s - first.governor_time_s,
                         second.stop_time_s - first.stop_time_s})
        Check(std::fabs(delta - 0.002) < 1e-12,
              "production timer consumer advances exactly once");

    // Lockstep is verification-only; with no lockstep preparation a fresh
    // controller instance continues to select the existing wall-clock path.
    go2_trot::TrotParams flag_off_params;
    flag_off_params.wall_clock_motion = true;
    flag_off_params.wbc_full = true;
    flag_off_params.wbc_primary = true;
    TrotExperiment flag_off(
        10.0, "/tmp/order109b_flag_off.csv", flag_off_params,
        1000, false, "", false);
    Check(flag_off.TestRunWallClockTick(State(200)),
          "flag-off LowCmdWrite executes through legacy path");
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    Check(flag_off.TestRunWallClockTick(State(202)),
          "flag-off remains wall-clock driven");
    Check(flag_off.TestLastMotionClockSample().motion_dt_s > 0.001,
          "flag-off wall-clock motion_dt remains nonzero");
}
} // namespace

int main()
{
    TestProductionChain();
    if (failures != 0)
    {
        std::fprintf(stderr, "lockstep_motion_clock_integration: %d failure(s)\n", failures);
        return 1;
    }
    std::printf("lockstep_motion_clock_integration: all checks passed\n");
    return 0;
}
