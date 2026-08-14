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

// --- TrackingExperiment::ApplyStepConfig ---
void TrackingExperiment::ApplyStepConfig(const StepConfig &step)
{
    const double sequence_offset_x =
        repeat_sequence_ ? sequence_offset_x_m_ : 0.0;
    const double sequence_offset_y =
        repeat_sequence_ ? sequence_offset_y_m_ : 0.0;
    body_shift_x_m_ = step.body_shift_x_m + sequence_offset_x;
    body_shift_y_m_ = step.body_shift_y_m + sequence_offset_y;
    foot_lift_height_m_ = step.foot_lift_height_m;
    lift_leg_ = step.lift_leg;
    lift_leg_name_ = kLegNames[static_cast<std::size_t>(step.lift_leg)];
    swing_x_m_ = step.swing_x_m;
    swing_y_m_ = step.swing_y_m;
    yaw_feedback_swing_x_m_ = 0.0;
    yaw_feedback_body_rotation_rad_ = 0.0;
    if (yaw_feedback_enabled_ && have_world_reference_)
    {
        // Apply a bounded differential foot-placement correction. In repeat
        // mode this is still safe: the correction follows the measured yaw
        // error, is clipped per foot, and decays as the heading converges.
        // The body-level y feedback below handles translation; this term
        // handles the long-horizon heading drift that body translation alone
        // cannot remove.
        const bool left_side =
            step.lift_leg == go2::Leg::FL ||
            step.lift_leg == go2::Leg::RL;
        const double side_sign = left_side ? 1.0 : -1.0;
        // A positive yaw error is corrected with right-side forward and
        // left-side backward swing. Keep this sign explicit: EXP-20
        // established it by an A/B probe rather than intuition.
        yaw_feedback_swing_x_m_ = std::max(
            -kWorldYawSwingFeedbackMaxCorrectionM,
            std::min(kWorldYawSwingFeedbackMaxCorrectionM,
                     -side_sign * kWorldYawSwingFeedbackGainMPerRad *
                         world_yaw_error_rad_));
        yaw_feedback_body_rotation_rad_ = std::max(
            -kWorldYawBodyRotationMaxRad,
            std::min(
                kWorldYawBodyRotationMaxRad,
                kWorldYawBodyRotationGain * world_yaw_error_rad_));
    }
    body_advance_x_m_ = step.body_advance_x_m + sequence_offset_x;
    body_advance_y_m_ = step.body_advance_y_m + sequence_offset_y;
    if (world_feedback_enabled_ && have_world_reference_)
    {
        if (!repeat_sequence_)
        {
            body_shift_x_m_ += world_feedback_x_m_;
            body_shift_y_m_ += world_feedback_y_m_;
        }
        // In repeat mode, keep local support geometry periodic. World
        // feedback belongs to the global advance endpoint; mixing it into
        // every local body shift makes repeated RL transitions harder.
        body_advance_x_m_ += world_feedback_x_m_;
        body_advance_y_m_ += world_feedback_y_m_;
    }
}

// --- TrackingExperiment::Init ---
bool TrackingExperiment::Init()
{
    if (!CheckMotionTargets())
    {
        return false;
    }

    csv_.open(csv_path_);
    if (!csv_)
    {
        std::cerr << "Failed to open CSV file: " << csv_path_ << std::endl;
        return false;
    }
    csv_ << std::fixed << std::setprecision(9);
    WriteCsvHeader();

    InitLowCmd();

    lowcmd_publisher_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(GO2_LEG_TOPIC_LOWCMD));
    lowcmd_publisher_->InitChannel();

    lowstate_subscriber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(GO2_LEG_TOPIC_LOWSTATE));
    lowstate_subscriber_->InitChannel(
        std::bind(&TrackingExperiment::LowStateMessageHandler, this, std::placeholders::_1), 1);

    highstate_subscriber_.reset(
        new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(
            GO2_LEG_TOPIC_HIGHSTATE));
    highstate_subscriber_->InitChannel(
        std::bind(
            &TrackingExperiment::HighStateMessageHandler,
            this,
            std::placeholders::_1),
        1);

    std::cout << "Waiting for the uncommanded robot to settle naturally...\n";
    if (!WaitForNaturalSettle(8.0))
    {
        std::cerr << "Robot did not settle before timeout. Check the simulator state.\n";
        return false;
    }
    if (world_feedback_enabled_ && !CaptureWorldReference())
    {
        return false;
    }

    if (const char *ready_file = std::getenv("GO2_READY_FILE"))
    {
        std::ofstream(ready_file) << "ready\n";
    }
    if (const char *start_file = std::getenv("GO2_START_FILE"))
    {
        std::cout << "Waiting for recording start signal...\n";
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(30);
        while (!std::filesystem::exists(start_file))
        {
            if (std::chrono::steady_clock::now() >= deadline)
            {
                std::cerr << "Recording start signal timed out\n";
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    low_cmd_write_thread_ = CreateRecurrentThreadEx(
        "tracklowcmd", UT_CPU_ID_NONE, int(dt_ * 1000000), &TrackingExperiment::LowCmdWrite, this);

    return true;
}

// --- TrackingExperiment::Shutdown ---
void TrackingExperiment::Shutdown()
{
    // CycloneDDS 0.10.2 can assert if an active listener is explicitly
    // destroyed while the simulator is still publishing at high frequency.
    // Stop our writer and make the result durable; main then uses quick_exit
    // so the OS reclaims the process-local DDS resources.
    low_cmd_write_thread_.reset();
    csv_.close();
}

// --- TrackingExperiment::InitLowCmd ---
void TrackingExperiment::InitLowCmd()
{
    low_cmd_.head()[0] = 0xFE;
    low_cmd_.head()[1] = 0xEF;
    low_cmd_.level_flag() = 0xFF;
    low_cmd_.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        low_cmd_.motor_cmd()[i].mode() = 0x01;
        low_cmd_.motor_cmd()[i].q() = PosStopF;
        low_cmd_.motor_cmd()[i].kp() = 0;
        low_cmd_.motor_cmd()[i].dq() = VelStopF;
        low_cmd_.motor_cmd()[i].kd() = 0;
        low_cmd_.motor_cmd()[i].tau() = 0;
    }
}

// --- TrackingExperiment::WaitForNaturalSettle ---
bool TrackingExperiment::WaitForNaturalSettle(double timeout_s)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout_s);
    auto stable_since = std::chrono::steady_clock::time_point{};

    while (std::chrono::steady_clock::now() < deadline)
    {
        bool stable = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (have_low_state_)
            {
                double max_joint_speed = 0.0;
                for (int i = 0; i < kMotorCount; ++i)
                {
                    max_joint_speed = std::max(
                        max_joint_speed,
                        std::abs(static_cast<double>(
                            low_state_.motor_state()[i].dq())));
                }
                double max_body_angular_speed = 0.0;
                for (int axis = 0; axis < 3; ++axis)
                {
                    max_body_angular_speed = std::max(
                        max_body_angular_speed,
                        std::abs(static_cast<double>(
                            low_state_.imu_state().gyroscope()[axis])));
                }
                stable =
                    max_joint_speed <= kNaturalSettleMaxJointSpeed &&
                    max_body_angular_speed <=
                        kNaturalSettleMaxBodyAngularSpeed;
            }
        }

        const auto now = std::chrono::steady_clock::now();
        if (stable)
        {
            if (stable_since == std::chrono::steady_clock::time_point{})
            {
                stable_since = now;
            }
            if (now - stable_since >=
                std::chrono::duration<double>(kNaturalSettleHoldDuration))
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                for (int i = 0; i < kMotorCount; ++i)
                {
                    start_joint_pos_[i] =
                        low_state_.motor_state()[i].q();
                }
                have_start_joint_pos_ = true;
                PrintFootPositions(
                    "Natural settled LowState", start_joint_pos_);
                PrintFootPositions("Stand target", stand_up_joint_pos_);
                return true;
            }
        }
        else
        {
            stable_since = std::chrono::steady_clock::time_point{};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

// --- TrackingExperiment::CaptureWorldReference ---
bool TrackingExperiment::CaptureWorldReference()
{
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(kWorldFeedbackReferenceTimeoutS);
    while (std::chrono::steady_clock::now() < deadline)
    {
        WorldPose pose;
        if (ReadWorldPose(pose))
        {
            world_reference_x_m_ = pose.base.x;
            world_reference_y_m_ = pose.base.y;
            world_reference_yaw_rad_ = pose.yaw_rad;
            have_world_reference_ = true;
            std::cout << "World reference captured: x=" << pose.base.x
                      << " m, y=" << pose.base.y
                      << " m, yaw=" << pose.yaw_rad << " rad\n";
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cerr << "World feedback requires LowState and SportModeState\n";
    return false;
}

// --- TrackingExperiment::LowStateMessageHandler ---
void TrackingExperiment::LowStateMessageHandler(const void *message)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    low_state_ = *(unitree_go::msg::dds_::LowState_ *)message;
    have_low_state_ = true;
}

// --- TrackingExperiment::HighStateMessageHandler ---
void TrackingExperiment::HighStateMessageHandler(const void *message)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    high_state_ = *(unitree_go::msg::dds_::SportModeState_ *)message;
    have_high_state_ = true;
}
