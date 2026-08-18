#include "trot_task.h"

#include <algorithm>
#include <iostream>

namespace
{

double Smoothstep(double x)
{
    if (x <= 0.0)
        return 0.0;
    if (x >= 1.0)
        return 1.0;
    return x * x * (3.0 - 2.0 * x);
}

double WrapAngle(double angle)
{
    return std::remainder(angle, 2.0 * TrotTask::kPi);
}

double Clamp(double value, double low, double high)
{
    return std::max(low, std::min(high, value));
}

}  // namespace

void TrotTask::Configure(bool task_mode, const TrotGoalConfig &goal)
{
    task_mode_ = task_mode;
    if (goal.enabled)
        SetWorldGoal(goal.x, goal.y, goal.tol);
}

void TrotTask::SetWorldGoal(double x, double y, double tol_m)
{
    goal_x_ = x;
    goal_y_ = y;
    goal_tol_ = std::max(0.02, tol_m);
    goal_enabled_ = true;
    reached_goal_ = false;
}

bool TrotTask::PhaseStandUp(
    double running_time_s,
    std::array<double, kMotorCount> &joint_targets)
{
    if (!(running_time_s < kStandUpDuration))
        return false;

    motion_stage_ = 0;
    const double phase = Smoothstep(running_time_s / kStandUpDuration);
    for (int i = 0; i < kMotorCount; ++i)
    {
        joint_targets[static_cast<std::size_t>(i)] =
            phase * stand_up_joint_pos_[static_cast<std::size_t>(i)] +
            (1.0 - phase) * start_joint_pos_[static_cast<std::size_t>(i)];
    }
    return true;
}

bool TrotTask::PhaseStandSettle(double running_time_s)
{
    if (!(running_time_s >= kStandUpDuration &&
          running_time_s < kStandUpDuration + kStandSettleDuration))
        return false;
    motion_stage_ = 1;
    return true;
}

bool TrotTask::PhaseLieDown(
    double running_time_s,
    std::array<double, kMotorCount> &joint_targets)
{
    if (!(task_mode_ && lie_down_started_))
        return false;

    const double lie_elapsed = running_time_s - lie_down_start_time_s_;
    const double lie_phase = Smoothstep(lie_elapsed / kStandDownDuration);
    motion_stage_ = lie_elapsed < kStandDownDuration ? 4 : 5;
    for (int i = 0; i < kMotorCount; ++i)
    {
        joint_targets[static_cast<std::size_t>(i)] =
            (1.0 - lie_phase) *
                stand_up_joint_pos_[static_cast<std::size_t>(i)] +
            lie_phase * stand_down_joint_pos_[static_cast<std::size_t>(i)];
    }
    if (lie_elapsed >= kStandDownDuration + kLieDownHoldDuration)
    {
        sequence_finished_ = true;
        std::cout << "Task completed: stand-walk-lie\n";
    }
    return true;
}

bool TrotTask::PhaseStopToStand(
    double running_time_s,
    std::array<double, kMotorCount> &joint_targets)
{
    if (!stop_requested_)
        return false;

    if (stop_start_time_s_ == 0.0)
    {
        stop_start_time_s_ = running_time_s;
        stop_origin_joint_targets_ = joint_targets;
        have_stop_origin_joint_targets_ = true;
        std::cout << "Trot stopping; returning to stand\n";
    }
    motion_stage_ = 3;
    const double stop_blend = Smoothstep(
        (running_time_s - stop_start_time_s_) / kStopTransitionDuration);
    for (int i = 0; i < kMotorCount; ++i)
    {
        joint_targets[static_cast<std::size_t>(i)] =
            (1.0 - stop_blend) *
                stop_origin_joint_targets_[static_cast<std::size_t>(i)] +
            stop_blend * stand_up_joint_pos_[static_cast<std::size_t>(i)];
    }
    if (running_time_s - stop_start_time_s_ >=
        kStopTransitionDuration + kFinalHoldDuration)
    {
        if (task_mode_ && task_completion_requested_ && !lie_down_started_)
        {
            lie_down_started_ = true;
            stop_requested_ = false;
            gait_started_ = false;
            task_completion_requested_ = false;
            lie_down_start_time_s_ = running_time_s;
            std::cout << "Task state: RETURN_TO_STAND -> LIE_DOWN\n";
        }
        else
        {
            sequence_finished_ = true;
        }
    }
    return true;
}

bool TrotTask::BeginGait(double running_time_s)
{
    if (gait_started_ || stop_requested_ || lie_down_started_)
        return false;
    if (running_time_s < kStandUpDuration + kStandSettleDuration)
        return false;
    gait_started_ = true;
    gait_start_time_s_ = running_time_s;
    motion_stage_ = 2;
    return true;
}

bool TrotTask::MaybeReachWorldGoal(double x, double y)
{
    if (!goal_enabled_ || reached_goal_)
        return false;
    if (!InLocomotion())
        return false;
    if (RemainingXy(x, y) > goal_tol_)
        return false;
    reached_goal_ = true;
    stop_requested_ = true;
    task_completion_requested_ = true;
    commanded_turn_rate_radps_ = 0.0;
    return true;
}

double TrotTask::RemainingXy(double x, double y) const
{
    return std::hypot(goal_x_ - x, goal_y_ - y);
}

double TrotTask::DesiredHeading(double x, double y) const
{
    const double dx = goal_x_ - x;
    const double dy = goal_y_ - y;
    if (std::hypot(dx, dy) < 1e-6)
        return heading_hold_rad_;
    return std::atan2(dy, dx);
}

double TrotTask::CommandedStepScale(double x, double y) const
{
    if (!goal_enabled_ || reached_goal_)
        return reached_goal_ ? 0.0 : 1.0;
    const double rem = RemainingXy(x, y);
    const double slow_r = kGoalSlowRadiusM;
    if (rem >= slow_r)
        return 1.0;
    return std::max(kMinStepScale, rem / slow_r);
}

double TrotTask::UpdateTurnFromPose(double x, double y, double yaw_rad)
{
    if (!goal_enabled_ || reached_goal_ || !InLocomotion())
    {
        commanded_turn_rate_radps_ = 0.0;
        return 0.0;
    }
    const double rem = RemainingXy(x, y);
    if (rem >= 2.0 * goal_tol_)
    {
        heading_hold_rad_ = DesiredHeading(x, y);
        have_heading_hold_ = true;
    }
    else if (!have_heading_hold_)
    {
        heading_hold_rad_ = yaw_rad;
        have_heading_hold_ = true;
    }
    const double heading_error = WrapAngle(heading_hold_rad_ - yaw_rad);
    if (std::abs(heading_error) < 0.25)
    {
        commanded_turn_rate_radps_ = 0.0;
        return 0.0;
    }
    commanded_turn_rate_radps_ = Clamp(
        kHeadingTurnGain * heading_error,
        -kHeadingTurnLimit,
        kHeadingTurnLimit);
    return commanded_turn_rate_radps_;
}

double TrotTask::TurnEnable(double running_time_s) const
{
    if (!gait_started_)
        return 0.0;
    return Clamp(
        (running_time_s - gait_start_time_s_ - 2.0 * kGaitBlendDuration) /
            kTurnRampS,
        0.0, 1.0);
}
