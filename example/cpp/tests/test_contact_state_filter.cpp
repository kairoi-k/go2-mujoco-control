#include <cmath>
#include <iostream>
#include <limits>

#include "contact_state_filter.h"

namespace
{

bool CheckHysteresisBand()
{
    const go2_control::HystereticContactParams params{5.0, 3.0};
    bool current = false;
    if (!go2_control::UpdateHystereticContact(
            false, 4.0, params, current) ||
        current)
    {
        return false;
    }
    if (!go2_control::UpdateHystereticContact(
            false, 5.0, params, current) ||
        !current)
    {
        return false;
    }
    if (!go2_control::UpdateHystereticContact(
            true, 4.0, params, current) ||
        !current)
    {
        return false;
    }
    return go2_control::UpdateHystereticContact(
               true, 3.0, params, current) &&
           !current;
}

bool CheckInvalidInput()
{
    const go2_control::HystereticContactParams params{5.0, 3.0};
    bool current = true;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (go2_control::UpdateHystereticContact(
            true, nan, params, current))
    {
        return false;
    }

    const go2_control::HystereticContactParams invalid_params{3.0, 5.0};
    return !go2_control::UpdateHystereticContact(
        false, 4.0, invalid_params, current);
}

} // namespace

int main()
{
    if (!CheckHysteresisBand() || !CheckInvalidInput())
    {
        std::cerr << "Hysteretic contact filter checks failed\n";
        return 1;
    }
    std::cout << "Hysteretic contact filter checks passed.\n";
    return 0;
}
