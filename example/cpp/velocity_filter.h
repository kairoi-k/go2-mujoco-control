#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "motion_frame_utils.h"

namespace go2_control
{

struct VelocityFilterParams
{
    double cutoff_hz = 4.0;
};

class FirstOrderVelocityFilter
{
public:
    explicit FirstOrderVelocityFilter(VelocityFilterParams params = {})
        : params_(params)
    {
    }

    void Reset() noexcept
    {
        state_ = {};
        initialized_ = false;
        last_alpha_ = 0.0;
    }

    bool Update(
        const Vector3 &measurement,
        double dt_s,
        Vector3 &filtered)
    {
        if (!ValidVector(measurement) ||
            !std::isfinite(dt_s) ||
            dt_s < 0.0 ||
            !std::isfinite(params_.cutoff_hz) ||
            params_.cutoff_hz < 0.0)
        {
            return false;
        }

        if (!initialized_)
        {
            state_ = measurement;
            initialized_ = true;
            last_alpha_ = 1.0;
            filtered = state_;
            return true;
        }

        if (params_.cutoff_hz == 0.0 || dt_s == 0.0)
        {
            last_alpha_ = params_.cutoff_hz == 0.0 ? 1.0 : 0.0;
        }
        else
        {
            last_alpha_ = 1.0 - std::exp(
                -2.0 * kPi * params_.cutoff_hz * dt_s);
            if (!std::isfinite(last_alpha_))
                return false;
            last_alpha_ = std::max(0.0, std::min(1.0, last_alpha_));
        }

        for (std::size_t axis = 0; axis < state_.size(); ++axis)
        {
            state_[axis] += last_alpha_ * (measurement[axis] - state_[axis]);
        }
        filtered = state_;
        return ValidVector(filtered);
    }

    bool initialized() const noexcept
    {
        return initialized_;
    }

    double last_alpha() const noexcept
    {
        return last_alpha_;
    }

private:
    static constexpr double kPi = 3.14159265358979323846;

    static bool ValidVector(const Vector3 &value)
    {
        for (double axis : value)
        {
            if (!std::isfinite(axis))
                return false;
        }
        return true;
    }

    VelocityFilterParams params_;
    Vector3 state_{};
    bool initialized_ = false;
    double last_alpha_ = 0.0;
};

} // namespace go2_control
