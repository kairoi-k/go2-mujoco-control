#include <cmath>
#include <iostream>
#include <limits>

#include "velocity_filter.h"

namespace
{

bool Near(double actual, double expected, double tolerance = 1e-12)
{
    return std::abs(actual - expected) <= tolerance;
}

bool CheckInitializationAndStepResponse()
{
    go2_control::FirstOrderVelocityFilter filter(
        go2_control::VelocityFilterParams{4.0});
    go2_control::Vector3 output{};
    if (!filter.Update({0.0, 0.0, 0.0}, 0.002, output) ||
        !filter.initialized() ||
        !Near(output[0], 0.0) ||
        !Near(filter.last_alpha(), 1.0))
    {
        return false;
    }

    const double expected_alpha =
        1.0 - std::exp(
            -2.0 * 3.14159265358979323846 * 4.0 * 0.002);
    if (!filter.Update({1.0, -0.5, 0.25}, 0.002, output) ||
        !Near(filter.last_alpha(), expected_alpha) ||
        !(output[0] > 0.0 && output[0] < 1.0) ||
        !(output[1] < 0.0 && output[1] > -0.5) ||
        !(output[2] > 0.0 && output[2] < 0.25))
    {
        return false;
    }

    const go2_control::Vector3 held = output;
    return filter.Update({1.0, -0.5, 0.25}, 0.0, output) &&
           Near(output[0], held[0]) && Near(output[1], held[1]) &&
           Near(output[2], held[2]);
}

bool CheckDisabledAndInvalidInput()
{
    go2_control::FirstOrderVelocityFilter filter(
        go2_control::VelocityFilterParams{0.0});
    go2_control::Vector3 output{};
    if (!filter.Update({0.2, 0.3, -0.4}, 0.002, output) ||
        !filter.Update({-0.6, 0.7, 0.8}, 0.002, output) ||
        !Near(output[0], -0.6) ||
        !Near(output[1], 0.7) ||
        !Near(output[2], 0.8) ||
        !Near(filter.last_alpha(), 1.0))
    {
        return false;
    }

    const go2_control::Vector3 nan_vector{
        std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
    return !filter.Update(nan_vector, 0.002, output) &&
           !filter.Update({0.0, 0.0, 0.0}, -0.001, output);
}

} // namespace

int main()
{
    if (!CheckInitializationAndStepResponse() ||
        !CheckDisabledAndInvalidInput())
    {
        std::cerr << "Velocity filter checks failed\n";
        return 1;
    }
    std::cout << "Velocity filter checks passed.\n";
    return 0;
}
