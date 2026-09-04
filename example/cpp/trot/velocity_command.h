#pragma once
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
namespace go2_trot {
struct VelocityCommandPoint
{
    double time_s = 0.0;
    double velocity_mps = 0.0;
};
struct VelocityCommandProfile
{
    std::vector<VelocityCommandPoint> points;
    bool Empty() const noexcept { return points.empty(); }
    bool Validate(std::string *error = nullptr) const
    {
        if (points.size() < 2)
        {
            if (error) *error = "velocity command profile needs at least two points";
            return false;
        }
        double previous_time = -std::numeric_limits<double>::infinity();
        for (const auto &point : points)
        {
            if (!std::isfinite(point.time_s) ||
                !std::isfinite(point.velocity_mps) ||
                point.time_s < 0.0 || point.time_s <= previous_time ||
                point.velocity_mps < 0.0)
            {
                if (error)
                    *error = "velocity command profile must have strictly increasing "
                             "nonnegative times and finite nonnegative velocities";
                return false;
            }
            previous_time = point.time_s;
        }
        return true;
    }
    double Sample(double time_s) const noexcept
    {
        if (points.empty() || !std::isfinite(time_s))
            return 0.0;
        if (time_s <= points.front().time_s)
            return points.front().velocity_mps;
        if (time_s >= points.back().time_s)
            return points.back().velocity_mps;
        const auto upper = std::upper_bound(
            points.begin(), points.end(), time_s,
            [](double value, const VelocityCommandPoint &point) {
                return value < point.time_s;
            });
        const auto &right = *upper;
        const auto &left = *(upper - 1);
        const double alpha = (time_s - left.time_s) /
            std::max(1.0e-9, right.time_s - left.time_s);
        return left.velocity_mps +
            std::clamp(alpha, 0.0, 1.0) *
                (right.velocity_mps - left.velocity_mps);
    }
};
inline bool LoadVelocityCommandProfile(
    const std::string &path,
    VelocityCommandProfile &profile,
    std::string *error = nullptr)
{
    std::ifstream input(path);
    if (!input)
    {
        if (error) *error = "cannot open velocity command profile: " + path;
        return false;
    }
    profile.points.clear();
    std::string line;
    int line_number = 0;
    while (std::getline(input, line))
    {
        ++line_number;
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos)
            line.erase(comment);
        for (char &character : line)
        {
            if (character == ',')
                character = ' ';
        }
        std::istringstream stream(line);
        double time_s = 0.0;
        double velocity_mps = 0.0;
        if (line.empty() || !(stream >> time_s >> velocity_mps))
        {
            if (line.find_first_not_of(" \t\r\n") != std::string::npos)
            {
                if (error)
                    *error = "invalid velocity profile line " +
                        std::to_string(line_number);
                return false;
            }
            continue;
        }
        profile.points.push_back({time_s, velocity_mps});
    }
    return profile.Validate(error);
}
struct VelocityCommandShaperParams
{
    double max_accel_mps2 = 1.50;
    double max_decel_mps2 = 2.00;
    double max_jerk_mps3 = 8.00;
    double max_speed_mps = 3.20;
    double max_tracking_lead_mps = 0.38;
};
struct VelocityCommandState
{
    double requested_mps = 0.0;
    double shaped_mps = 0.0;
    double accel_mps2 = 0.0;
    double jerk_mps3 = 0.0;
    bool active = false;
    double applied_mps = 0.0;
};
class VelocityCommandShaper
{
public:
    explicit VelocityCommandShaper(VelocityCommandShaperParams params = {})
        : params_(params)
    {
    }
    void Reset(double velocity_mps = 0.0) noexcept
    {
        shaped_mps_ = std::clamp(velocity_mps, 0.0, params_.max_speed_mps);
        accel_mps2_ = 0.0;
        previous_accel_mps2_ = 0.0;
        initialized_ = true;
    }
    VelocityCommandState Step(double requested_mps, double dt_s) noexcept
    {
        if (!initialized_)
            Reset();
        requested_mps = std::clamp(
            std::isfinite(requested_mps) ? requested_mps : 0.0,
            0.0, params_.max_speed_mps);
        const double dt = std::clamp(
            std::isfinite(dt_s) ? dt_s : 0.002, 1.0e-4, 0.050);
        const double delta = requested_mps - shaped_mps_;
        const double acceleration_limit = delta >= 0.0
            ? params_.max_accel_mps2 : params_.max_decel_mps2;
        const double desired_accel = delta == 0.0
            ? 0.0
            : std::copysign(std::max(0.0, acceleration_limit), delta);
        const double max_accel_change =
            std::max(0.0, params_.max_jerk_mps3) * dt;
        accel_mps2_ += std::clamp(
            desired_accel - accel_mps2_,
            -max_accel_change, max_accel_change);
        shaped_mps_ = std::clamp(
            shaped_mps_ + accel_mps2_ * dt,
            0.0, params_.max_speed_mps);
        if ((delta >= 0.0 && shaped_mps_ > requested_mps) ||
            (delta < 0.0 && shaped_mps_ < requested_mps))
        {
            shaped_mps_ = requested_mps;
        }
        const double jerk = (accel_mps2_ - previous_accel_mps2_) / dt;
        previous_accel_mps2_ = accel_mps2_;
        return {requested_mps, shaped_mps_, accel_mps2_, jerk, true};
    }
private:
    VelocityCommandShaperParams params_;
    double shaped_mps_ = 0.0;
    double accel_mps2_ = 0.0;
    double previous_accel_mps2_ = 0.0;
    bool initialized_ = false;
};
struct ContinuousVelocityGaitSchedule
{
    double period_s = 0.44;
    double duty_factor = 0.70;
    double step_length_m = 0.0;
    double foot_lift_m = 0.035;
    const char *regime = "continuous-trot";
};
// The running-trot timing is validated at the high-speed end, but it is
// not a viable sustained low-speed support schedule: at 0.30 m/s it gives
// a 5 mm swing lift and only 44% duty.  The support-rich probe schedule
// (0.50 s period, 0.75 duty) is therefore used only for a *sustained*
// low-speed regime such as a terrain approach.  Keying the schedule to
// the instantaneous applied speed proved unstable: during an aggressive
// ramp the tracking-lead cap holds the applied speed low, the schedule
// then stays in a long-period blend for the whole ramp, and measured
// tracking collapses (B0 brake_3_to_0 regression, 2026-08-28).
// Qualification is time-based so ramping *through* the low band never
// engages it, while a profile that holds <0.40 m/s still receives the
// support-rich timing.  This changes timing parameters only; topology
// and the acceleration/jerk shaper remain untouched.
inline ContinuousVelocityGaitSchedule ScheduleContinuousVelocityGait(
    double velocity_mps,
    bool low_speed_support) noexcept
{
    const double speed = std::clamp(
        std::isfinite(velocity_mps) ? velocity_mps : 0.0, 0.0, 3.20);
    constexpr double kLowSpeedPeriodS = 0.50;
    constexpr double kLowSpeedDuty = 0.75;
    constexpr double kLowSpeedFootLiftM = 0.035;
    constexpr double kHighSpeedPeriodS = 0.14;
    constexpr double kHighSpeedDuty = 0.44;
    constexpr double kHighSpeedFootLiftM = 0.200;
    ContinuousVelocityGaitSchedule schedule;
    if (low_speed_support)
    {
        schedule.period_s = kLowSpeedPeriodS;
        schedule.duty_factor = kLowSpeedDuty;
        schedule.foot_lift_m = kLowSpeedFootLiftM;
    }
    else
    {
        const double normalized = std::clamp(speed / 3.0, 0.0, 1.0);
        const double blend = normalized * normalized *
            (3.0 - 2.0 * normalized);
        schedule.period_s = kHighSpeedPeriodS;
        schedule.duty_factor = kHighSpeedDuty;
        schedule.foot_lift_m = kHighSpeedFootLiftM * blend;
    }
    schedule.step_length_m = speed * schedule.period_s /
        std::max(0.20, 2.0 * schedule.duty_factor);
    return schedule;
}
class ContinuousVelocityGaitScheduler
{
public:
    void Reset() noexcept { low_speed_time_s_ = 0.0; }
    ContinuousVelocityGaitSchedule Step(
        double velocity_mps,
        double dt_s) noexcept
    {
        constexpr double kLowSpeedEnterMps = 0.40;
        constexpr double kLowSpeedQualificationS = 1.0;
        const double speed = std::clamp(
            std::isfinite(velocity_mps) ? velocity_mps : 0.0, 0.0, 3.20);
        const double dt = std::clamp(
            std::isfinite(dt_s) ? dt_s : 0.0, 0.0, 0.050);
        if (speed < kLowSpeedEnterMps)
            low_speed_time_s_ += dt;
        else
            low_speed_time_s_ = 0.0;
        return ScheduleContinuousVelocityGait(
            speed, low_speed_time_s_ >= kLowSpeedQualificationS);
    }
private:
    double low_speed_time_s_ = 0.0;
};
}  // namespace go2_trot
