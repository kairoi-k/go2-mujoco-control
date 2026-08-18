#pragma once
// Stand / walk / stand / lie sequencer plus optional world-frame A→B goal.
// Owns phase timing and stand/lie joint targets so TrotExperiment is not
// also the task state machine.

#include <array>
#include <cmath>
#include <cstddef>

struct TrotGoalConfig
{
    bool enabled = false;
    double x = 0.0;
    double y = 0.0;
    double tol = 0.12;
};

class TrotTask
{
public:
    static constexpr int kMotorCount = 12;
    static constexpr double kPi = 3.14159265358979323846;
    // Keep identical to go2_trot timing constants in trot_types.h.
    static constexpr double kStandUpDuration = 3.0;
    static constexpr double kStandSettleDuration = 0.5;
    static constexpr double kStandDownDuration = 3.0;
    static constexpr double kStopTransitionDuration = 2.0;
    static constexpr double kFinalHoldDuration = 0.8;
    static constexpr double kLieDownHoldDuration = 0.8;
    static constexpr double kGaitBlendDuration = 0.8;
    static constexpr double kHeadingTurnGain = 0.8;
    static constexpr double kHeadingTurnLimit = 0.22;
    static constexpr double kGoalSlowRadiusM = 0.35;
    static constexpr double kMinStepScale = 0.35;
    static constexpr double kTurnRampS = 0.8;

    const std::array<double, kMotorCount> stand_up_joint_pos_{
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763};
    const std::array<double, kMotorCount> stand_down_joint_pos_{
        0.0473455, 1.22187, -2.44375,
        -0.0473455, 1.22187, -2.44375,
        0.0473455, 1.22187, -2.44375,
        -0.0473455, 1.22187, -2.44375};

    std::array<double, kMotorCount> start_joint_pos_{};
    bool have_start_joint_pos_ = false;

    int motion_stage_ = 0;
    double gait_start_time_s_ = 0.0;
    bool gait_started_ = false;
    bool stop_requested_ = false;
    bool task_completion_requested_ = false;
    bool lie_down_started_ = false;
    double lie_down_start_time_s_ = 0.0;
    double stop_start_time_s_ = 0.0;
    std::array<double, kMotorCount> stop_origin_joint_targets_{};
    bool have_stop_origin_joint_targets_ = false;
    bool sequence_finished_ = false;
    bool task_mode_ = false;

    bool goal_enabled_ = false;
    bool reached_goal_ = false;
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    double goal_tol_ = 0.12;
    double heading_hold_rad_ = 0.0;
    bool have_heading_hold_ = false;
    double commanded_turn_rate_radps_ = 0.0;

    void Configure(bool task_mode, const TrotGoalConfig &goal);
    void SetWorldGoal(double x, double y, double tol_m);

    bool PhaseStandUp(
        double running_time_s,
        std::array<double, kMotorCount> &joint_targets);
    bool PhaseStandSettle(double running_time_s);
    bool PhaseLieDown(
        double running_time_s,
        std::array<double, kMotorCount> &joint_targets);
    bool PhaseStopToStand(
        double running_time_s,
        std::array<double, kMotorCount> &joint_targets);
    bool BeginGait(double running_time_s);

    bool MaybeReachWorldGoal(double x, double y);
    double RemainingXy(double x, double y) const;
    double DesiredHeading(double x, double y) const;
    double CommandedStepScale(double x, double y) const;
    double UpdateTurnFromPose(double x, double y, double yaw_rad);
    double TurnEnable(double running_time_s) const;

    bool InLocomotion() const
    {
        return gait_started_ && !stop_requested_ && !lie_down_started_;
    }
};
