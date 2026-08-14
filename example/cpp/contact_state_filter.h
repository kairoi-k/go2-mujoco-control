#pragma once

#include <cmath>

namespace go2_control
{

struct HystereticContactParams
{
    double engage_force_n = 5.0;
    double release_force_n = 3.0;
};

inline bool UpdateHystereticContact(
    bool previous_contact,
    double force_n,
    const HystereticContactParams &params,
    bool &current_contact)
{
    if (!std::isfinite(force_n) ||
        !std::isfinite(params.engage_force_n) ||
        !std::isfinite(params.release_force_n) ||
        params.release_force_n > params.engage_force_n)
    {
        return false;
    }

    current_contact = previous_contact;
    if (previous_contact)
    {
        if (force_n <= params.release_force_n)
            current_contact = false;
    }
    else if (force_n >= params.engage_force_n)
    {
        current_contact = true;
    }
    return true;
}

} // namespace go2_control
