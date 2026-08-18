#include "leg_lift_experiment.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "go2_forward_kinematics.h"
#include "go2_inverse_kinematics.h"

using namespace unitree::common;
using namespace unitree::robot;
using namespace go2_leg;

// --- TrackingExperiment::ReadWorldPose ---
bool TrackingExperiment::ReadWorldPose(WorldPose &pose)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!have_low_state_ || !have_high_state_)
    {
        return false;
    }
    pose = ComputeWorldPose(low_state_, high_state_);
    return true;
}

// --- TrackingExperiment::UpdateWorldFeedbackForNextStep ---
bool TrackingExperiment::UpdateWorldFeedbackForNextStep()
{
    if (!world_feedback_enabled_)
    {
        return true;
    }
    if (!have_world_reference_)
    {
        std::cerr << "World feedback reference is not available\n";
        return false;
    }
    WorldPose pose;
    if (!ReadWorldPose(pose))
    {
        std::cerr << "World feedback state is not available\n";
        return false;
    }
    const double actual_x = pose.base.x - world_reference_x_m_;
    const double actual_y = pose.base.y - world_reference_y_m_;
    world_yaw_error_rad_ = std::remainder(
        pose.yaw_rad - world_reference_yaw_rad_, 2.0 * kPi);
    const double reference_cos = std::cos(world_reference_yaw_rad_);
    const double reference_sin = std::sin(world_reference_yaw_rad_);
    const double target_world_x =
        reference_cos * body_advance_x_m_ -
        reference_sin * body_advance_y_m_;
    const double target_world_y =
        reference_sin * body_advance_x_m_ +
        reference_cos * body_advance_y_m_;
    const double position_error_world_x = actual_x - target_world_x;
    const double position_error_world_y = actual_y - target_world_y;
    world_position_error_x_m_ = position_error_world_x;
    world_position_error_y_m_ = position_error_world_y;
    const double correction_world_x = std::max(
        -kWorldFeedbackMaxCorrectionM,
        std::min(kWorldFeedbackMaxCorrectionM,
                 -kWorldFeedbackGain * position_error_world_x));
    const double correction_world_y = std::max(
        -kWorldFeedbackMaxCorrectionM,
        std::min(kWorldFeedbackMaxCorrectionM,
                 -kWorldFeedbackGain * position_error_world_y));
    const double requested_feedback_x_m =
        reference_cos * correction_world_x +
        reference_sin * correction_world_y;
    const double requested_feedback_y_m =
        -reference_sin * correction_world_x +
        reference_cos * correction_world_y;
    yaw_feedback_y_m_ = 0.0;
    if (yaw_feedback_enabled_)
    {
        yaw_feedback_y_m_ = std::max(
            -kWorldYawFeedbackMaxCorrectionM,
            std::min(
                kWorldYawFeedbackMaxCorrectionM,
                -kWorldYawFeedbackGainMPerRad * world_yaw_error_rad_));
    }
    const auto slew_feedback = [](double current, double requested)
    {
        const double delta = std::max(
            -kWorldFeedbackMaxCorrectionStepM,
            std::min(
                kWorldFeedbackMaxCorrectionStepM,
                requested - current));
        return current + delta;
    };
    const double requested_total_feedback_x_m =
        requested_feedback_x_m;
    const double requested_total_feedback_y_m =
        requested_feedback_y_m + yaw_feedback_y_m_;
    world_feedback_x_m_ = slew_feedback(
        world_feedback_x_m_, requested_total_feedback_x_m);
    world_feedback_y_m_ = slew_feedback(
        world_feedback_y_m_, requested_total_feedback_y_m);
    ++world_feedback_update_count_;
    std::cout << "World feedback #" << world_feedback_update_count_
              << ": actual=(" << actual_x << ", " << actual_y
              << ") m, target=(" << target_world_x << ", "
              << target_world_y << ") m, yaw_error="
              << world_yaw_error_rad_ * 180.0 / kPi
              << " deg, correction=("
              << world_feedback_x_m_ << ", " << world_feedback_y_m_
              << ") m, requested=(" << requested_total_feedback_x_m
              << ", " << requested_total_feedback_y_m
              << ") m, yaw_correction_y=" << yaw_feedback_y_m_
              << " m\n";
    return true;
}

// --- TrackingExperiment::BeginTerminalCorrection ---
void TrackingExperiment::BeginTerminalCorrection()
{
    if (terminal_correction_started_)
        return;

    terminal_correction_started_ = true;
    terminal_correction_start_time_s_ = running_time_;
    terminal_correction_x_m_ = 0.0;
    terminal_correction_y_m_ = 0.0;
    if (!world_feedback_enabled_ || !have_world_reference_)
        return;

    WorldPose pose;
    if (!ReadWorldPose(pose))
    {
        std::cerr << "Terminal correction state is not available"
                  << std::endl;
        return;
    }
    const double actual_x = pose.base.x - world_reference_x_m_;
    const double actual_y = pose.base.y - world_reference_y_m_;
    world_yaw_error_rad_ = std::remainder(
        pose.yaw_rad - world_reference_yaw_rad_, 2.0 * kPi);
    const double reference_cos = std::cos(world_reference_yaw_rad_);
    const double reference_sin = std::sin(world_reference_yaw_rad_);
    const double target_world_x =
        reference_cos * body_advance_x_m_ -
        reference_sin * body_advance_y_m_;
    const double target_world_y =
        reference_sin * body_advance_x_m_ +
        reference_cos * body_advance_y_m_;
    const double error_world_x = actual_x - target_world_x;
    const double error_world_y = actual_y - target_world_y;
    const double correction_world_x = std::max(
        -kTerminalCorrectionMaxM,
        std::min(kTerminalCorrectionMaxM,
                 -kTerminalCorrectionGain * error_world_x));
    const double correction_world_y = std::max(
        -kTerminalCorrectionMaxM,
        std::min(kTerminalCorrectionMaxM,
                 -kTerminalCorrectionGain * error_world_y));
    terminal_correction_x_m_ =
        reference_cos * correction_world_x +
        reference_sin * correction_world_y;
    terminal_correction_y_m_ =
        -reference_sin * correction_world_x +
        reference_cos * correction_world_y;
    std::cout << "Terminal correction: error=(" << error_world_x << ", "
              << error_world_y << ") m, target correction=("
              << terminal_correction_x_m_ << ", "
              << terminal_correction_y_m_ << ") m over "
              << kTerminalCorrectionDuration << " s" << std::endl;
}

// --- TrackingExperiment::LogTerminalWorldError ---
void TrackingExperiment::LogTerminalWorldError()
{
    if (!world_feedback_enabled_ || !have_world_reference_)
        return;
    WorldPose pose;
    if (!ReadWorldPose(pose))
    {
        std::cerr << "Terminal world pose is not available" << std::endl;
        return;
    }
    const double actual_x = pose.base.x - world_reference_x_m_;
    const double actual_y = pose.base.y - world_reference_y_m_;
    world_yaw_error_rad_ = std::remainder(
        pose.yaw_rad - world_reference_yaw_rad_, 2.0 * kPi);
    const double reference_cos = std::cos(world_reference_yaw_rad_);
    const double reference_sin = std::sin(world_reference_yaw_rad_);
    const double target_world_x =
        reference_cos * body_advance_x_m_ -
        reference_sin * body_advance_y_m_;
    const double target_world_y =
        reference_sin * body_advance_x_m_ +
        reference_cos * body_advance_y_m_;
    terminal_world_position_error_x_m_ = actual_x - target_world_x;
    terminal_world_position_error_y_m_ = actual_y - target_world_y;
    const double terminal_error_m = std::hypot(
        terminal_world_position_error_x_m_,
        terminal_world_position_error_y_m_);
    std::cout << "Terminal convergence: "
              << (terminal_error_m <= kTerminalPositionToleranceM
                      ? "accepted"
                      : "not accepted")
              << " (error=" << terminal_error_m * 1000.0
              << " mm, tolerance="
              << kTerminalPositionToleranceM * 1000.0 << " mm)"
              << std::endl;
    std::cout << "Terminal world error: dx="
              << terminal_world_position_error_x_m_
              << " m, dy=" << terminal_world_position_error_y_m_
              << " m, actual=(" << actual_x << ", " << actual_y
              << ") m, target=(" << target_world_x << ", "
              << target_world_y << ") m, yaw_error="
              << world_yaw_error_rad_ * 180.0 / kPi
              << " deg" << std::endl;
}
