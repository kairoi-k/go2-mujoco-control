#include "velocity_command.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>

namespace {

constexpr double kMinDtS = 1.0e-4;
constexpr double kMaxDtS = 0.050;
constexpr double kDiscreteShaperStopBoundM = 0.078001;

struct StopResult
{
    double distance_m = 0.0;
    std::size_t steps = 0;
};

StopResult StopFrom03(double dt_s)
{
    go2_trot::VelocityCommandShaperParams params;
    params.max_accel_mps2 = 0.8;
    params.max_decel_mps2 = 1.2;
    params.max_jerk_mps3 = 4.0;
    params.max_speed_mps = 0.3;

    go2_trot::VelocityCommandShaper shaper(params);
    shaper.Reset(0.3);

    StopResult result;
    for (std::size_t i = 0; i < 10000; ++i)
    {
        const auto state = shaper.Step(0.0, dt_s);
        result.distance_m += state.shaped_mps * dt_s;
        ++result.steps;
        assert(state.shaped_mps >= -1.0e-12 &&
               state.shaped_mps <= 0.3 + 1.0e-12);
        assert(state.accel_mps2 <= 0.8 + 1.0e-12 &&
               state.accel_mps2 >= -1.2 - 1.0e-12);
        assert(std::abs(state.jerk_mps3) <= 4.0 + 1.0e-9);
        if (state.shaped_mps == 0.0)
            return result;
    }
    assert(false && "velocity shaper failed to stop");
    return result;
}

} // namespace

int main()
{
    // Order-116 P1 correction: exercise the actual discrete Euler shaper,
    // including jerk limiting, saturation and overshoot correction.  The
    // prior 0.183334 m number was only a loose continuous bound.
    double worst_distance_m = 0.0;
    for (std::size_t i = 0; i <= 1000; ++i)
    {
        const double alpha = static_cast<double>(i) / 1000.0;
        const double dt_s = kMinDtS + alpha * (kMaxDtS - kMinDtS);
        const auto result = StopFrom03(dt_s);
        worst_distance_m = std::max(worst_distance_m, result.distance_m);
    }
    assert(worst_distance_m <= kDiscreteShaperStopBoundM);

    // C-006 lockstep advances the controller on one 2 ms state tick.  Pin the
    // production-tick discrete value separately from the sensor-to-halt latency
    // budget, which remains an end-to-end runtime proof obligation.
    const auto lockstep = StopFrom03(0.002);
    assert(std::abs(lockstep.distance_m - 0.0774008) < 1.0e-9);
    assert(lockstep.steps == 200);
    return 0;
}
