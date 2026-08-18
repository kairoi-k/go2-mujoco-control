#include <array>
#include <cmath>
#include <iostream>

#include "trot_task.h"

namespace
{

bool Near(double actual, double expected, double tolerance = 1e-9)
{
    return std::abs(actual - expected) <= tolerance;
}

bool CheckStandSettleGaitTiming()
{
    TrotTask task;
    task.Configure(true, {});
    std::array<double, TrotTask::kMotorCount> joints{};
    task.start_joint_pos_ = task.stand_down_joint_pos_;

    if (!task.PhaseStandUp(0.0, joints) || task.motion_stage_ != 0)
        return false;
    if (task.PhaseStandSettle(0.0) || task.BeginGait(0.0))
        return false;
    if (!task.PhaseStandSettle(TrotTask::kStandUpDuration + 0.1) ||
        task.motion_stage_ != 1)
        return false;
    if (!task.BeginGait(TrotTask::kStandUpDuration +
                        TrotTask::kStandSettleDuration) ||
        task.motion_stage_ != 2 || !task.gait_started_)
        return false;
    if (task.BeginGait(10.0))
        return false;
    return true;
}

bool CheckWorldGoalStopAndHeading()
{
    TrotTask task;
    TrotGoalConfig goal;
    goal.enabled = true;
    goal.x = 2.0;
    goal.y = 0.5;
    goal.tol = 0.12;
    task.Configure(true, goal);
    task.BeginGait(4.0);

    if (!Near(task.RemainingXy(0.0, 0.0), std::hypot(2.0, 0.5)))
        return false;
    if (!Near(task.DesiredHeading(0.0, 0.0), std::atan2(0.5, 2.0)))
        return false;
    if (!Near(task.CommandedStepScale(0.0, 0.0), 1.0))
        return false;
    if (!(task.CommandedStepScale(1.85, 0.46) < 1.0 &&
          task.CommandedStepScale(1.85, 0.46) >= TrotTask::kMinStepScale))
        return false;

    if (task.MaybeReachWorldGoal(0.0, 0.0) || task.reached_goal_)
        return false;

    const double yaw0 = 0.0;
    if (!Near(task.UpdateTurnFromPose(0.0, 0.0, yaw0), 0.0, 1e-12))
        return false;
    task.SetWorldGoal(0.2, 2.0, 0.12);
    const double turn = task.UpdateTurnFromPose(0.0, 0.0, yaw0);
    if (!(turn > 0.0) || !task.InLocomotion())
        return false;
    task.SetWorldGoal(2.0, 0.5, 0.12);

    if (!task.MaybeReachWorldGoal(2.0, 0.5) || !task.reached_goal_ ||
        !task.stop_requested_ || !task.task_completion_requested_)
        return false;
    if (task.MaybeReachWorldGoal(2.0, 0.5))
        return false;
    return true;
}

bool CheckStopThenLie()
{
    TrotTask task;
    task.Configure(true, {});
    task.BeginGait(4.0);
    task.stop_requested_ = true;
    task.task_completion_requested_ = true;
    std::array<double, TrotTask::kMotorCount> joints = task.stand_up_joint_pos_;
    if (!task.PhaseStopToStand(4.0, joints) || task.motion_stage_ != 3)
        return false;
    const double done =
        4.0 + TrotTask::kStopTransitionDuration + TrotTask::kFinalHoldDuration;
    if (!task.PhaseStopToStand(done, joints) || !task.lie_down_started_ ||
        task.sequence_finished_)
        return false;
    const double lie_done =
        done + TrotTask::kStandDownDuration + TrotTask::kLieDownHoldDuration;
    if (!task.PhaseLieDown(lie_done, joints) || !task.sequence_finished_)
        return false;
    return true;
}

}  // namespace

int main()
{
    if (!CheckStandSettleGaitTiming())
    {
        std::cerr << "test_trot_task: stand/settle/gait timing failed\n";
        return 1;
    }
    if (!CheckWorldGoalStopAndHeading())
    {
        std::cerr << "test_trot_task: world goal/heading failed\n";
        return 1;
    }
    if (!CheckStopThenLie())
    {
        std::cerr << "test_trot_task: stop-then-lie failed\n";
        return 1;
    }
    return 0;
}
