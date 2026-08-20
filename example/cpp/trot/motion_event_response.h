#pragma once

// Shared event-to-reference policy for reactive locomotion experiments.
// Events modify continuous locomotion references; they never splice joint
// trajectories or switch the WBC/MPC plant.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace go2_control
{

enum class MotionEventType
{
    kNone = 0,
    kEmergencyStop,
    kObstacleLeft,
    kObstacleRight,
    kTurnLeft,
    kTurnRight,
    kSlip,
    kLowFriction,
    kImpact,
};

inline const char *MotionEventName(MotionEventType type) noexcept
{
    switch (type)
    {
    case MotionEventType::kEmergencyStop:
        return "emergency_stop";
    case MotionEventType::kObstacleLeft:
        return "obstacle_left";
    case MotionEventType::kObstacleRight:
        return "obstacle_right";
    case MotionEventType::kTurnLeft:
        return "turn_left";
    case MotionEventType::kTurnRight:
        return "turn_right";
    case MotionEventType::kSlip:
        return "slip";
    case MotionEventType::kLowFriction:
        return "low_friction";
    case MotionEventType::kImpact:
        return "impact";
    case MotionEventType::kNone:
        break;
    }
    return "none";
}

inline bool ParseMotionEventType(
    const std::string &name,
    MotionEventType &type)
{
    if (name == "emergency_stop" || name == "stop")
        type = MotionEventType::kEmergencyStop;
    else if (name == "obstacle_left")
        type = MotionEventType::kObstacleLeft;
    else if (name == "obstacle_right")
        type = MotionEventType::kObstacleRight;
    else if (name == "turn_left")
        type = MotionEventType::kTurnLeft;
    else if (name == "turn_right")
        type = MotionEventType::kTurnRight;
    else if (name == "slip")
        type = MotionEventType::kSlip;
    else if (name == "low_friction")
        type = MotionEventType::kLowFriction;
    else if (name == "impact")
        type = MotionEventType::kImpact;
    else if (name == "none")
        type = MotionEventType::kNone;
    else
        return false;
    return true;
}

struct MotionReference
{
    double vx_mps = 0.0;
    double vy_mps = 0.0;
    double yaw_rate_radps = 0.0;
    double step_scale = 1.0;
    double duty_factor = 0.75;
    double foot_lift_m = 0.020;
    bool hold_stance = false;
};

struct MotionEvent
{
    MotionEventType type = MotionEventType::kNone;
    double start_time_s = 0.0;
    double duration_s = 0.0;
    double magnitude = 0.0;

    bool IsActive(double time_s) const noexcept
    {
        return type != MotionEventType::kNone &&
               std::isfinite(start_time_s) &&
               std::isfinite(duration_s) && duration_s > 0.0 &&
               time_s >= start_time_s &&
               time_s < start_time_s + duration_s;
    }
};

inline bool LoadMotionEventScript(
    const std::string &path,
    std::vector<MotionEvent> &events,
    std::string *error = nullptr)
{
    events.clear();
    std::ifstream input(path);
    if (!input)
    {
        if (error)
            *error = "cannot open event script: " + path;
        return false;
    }

    std::string line;
    int line_number = 0;
    while (std::getline(input, line))
    {
        ++line_number;
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos)
            line.resize(comment);
        std::istringstream stream(line);
        double start_s = 0.0;
        double duration_s = 0.0;
        std::string name;
        if (!(stream >> start_s >> duration_s >> name))
        {
            if (line.find_first_not_of(" \t\r\n") == std::string::npos)
                continue;
            if (error)
                *error = "event script line " + std::to_string(line_number) +
                         " requires: start duration event [magnitude]";
            events.clear();
            return false;
        }

        double magnitude = 0.0;
        std::string magnitude_token;
        if (stream >> magnitude_token)
        {
            std::istringstream magnitude_stream(magnitude_token);
            if (!(magnitude_stream >> magnitude) || !magnitude_stream.eof())
            {
                if (error)
                    *error = "event script line " +
                             std::to_string(line_number) +
                             " has invalid magnitude";
                events.clear();
                return false;
            }
        }
        std::string extra;
        if (stream >> extra)
        {
            if (error)
                *error = "event script line " + std::to_string(line_number) +
                         " has too many fields";
            events.clear();
            return false;
        }

        MotionEventType type = MotionEventType::kNone;
        if (!ParseMotionEventType(name, type))
        {
            if (error)
                *error = "event script line " + std::to_string(line_number) +
                         " has unknown event: " + name;
            events.clear();
            return false;
        }
        if (!std::isfinite(start_s) || start_s < 0.0 ||
            !std::isfinite(duration_s) || duration_s <= 0.0 ||
            !std::isfinite(magnitude))
        {
            if (error)
                *error = "event script line " + std::to_string(line_number) +
                         " has invalid timing or magnitude";
            events.clear();
            return false;
        }
        events.push_back({type, start_s, duration_s, magnitude});
    }

    std::sort(events.begin(), events.end(),
              [](const MotionEvent &a, const MotionEvent &b) {
                  return a.start_time_s < b.start_time_s;
              });
    return true;
}

struct MotionSensorSample
{
    double velocity_x_mps = 0.0;
    double raw_velocity_x_mps = 0.0;
    double raw_velocity_y_mps = 0.0;
    double velocity_y_mps = 0.0;
    double accel_x_mps2 = 0.0;
    double accel_y_mps2 = 0.0;
    double accel_z_mps2 = 0.0;
    int contact_count = 0;
    bool have_velocity = false;
    bool have_raw_velocity = false;
    double angular_velocity_z_radps = 0.0;
    bool have_angular_velocity_z = false;
    // Forward obstacle scan in the robot base frame. Distances are measured
    // from the base toward the three forward sectors.
    bool have_obstacle_scan = false;
    double obstacle_scan_age_s = std::numeric_limits<double>::infinity();
    double obstacle_center_distance_m = std::numeric_limits<double>::infinity();
    double obstacle_left_distance_m = std::numeric_limits<double>::infinity();
    double obstacle_right_distance_m = std::numeric_limits<double>::infinity();
    double obstacle_center_height_m = 0.0;
    double obstacle_left_height_m = 0.0;
    double obstacle_right_height_m = 0.0;
};

struct MotionEventDetectorConfig
{
    double obstacle_event_duration_s = 8.00;
    double low_friction_event_duration_s = 1.20;
    double warmup_s = 1.50;
    double slip_velocity_error_rise_mpsps = 1.50;
    double low_friction_velocity_error_mps = 0.30;
    double low_friction_confirm_s = 0.25;
    double obstacle_min_distance_m = 0.20;
    double obstacle_max_distance_m = 1.30;
    double obstacle_min_height_m = 0.10;
    double obstacle_confirm_s = 0.04;
    double obstacle_rearm_clear_s = 0.25;
    double max_sensor_age_s = 0.15;
    double slip_velocity_error_mps = 0.45;
    double slip_confirm_s = 0.06;
    double impact_extreme_vertical_excess_mps2 = 40.0;
    double impact_velocity_jump_mps = 0.35;
    double impact_release_s = 0.50;
    double event_duration_s = 0.80;
    double cooldown_s = 1.50;
    double gravity_mps2 = 9.81;
};

class MotionEventDetector
{
public:
    explicit MotionEventDetector(MotionEventDetectorConfig config = {})
        : config_(config)
    {
    }

    void Reset() noexcept
    {
        active_event_ = {};
        low_friction_accum_s_ = 0.0;
        obstacle_accum_s_ = 0.0;
        obstacle_clear_accum_s_ = 0.0;
        obstacle_first_seen_time_s_ = -1.0;
        slip_candidate_age_s_ = 0.0;
        impact_clear_accum_s_ = 0.0;
        previous_velocity_x_mps_ = 0.0;
        previous_velocity_y_mps_ = 0.0;
        previous_velocity_error_mps_ = 0.0;
        impact_latched_ = false;
        have_previous_velocity_ = false;
        have_previous_velocity_error_ = false;
        obstacle_latched_ = false;
        last_trigger_time_s_ = -1.0e9;
    }

    MotionEvent Observe(
        double time_s,
        double dt_s,
        const MotionSensorSample &sample,
        const MotionReference &nominal)
    {
        const bool active_event = active_event_.IsActive(time_s);
        double velocity_jump_mps = 0.0;
        const bool have_velocity =
            sample.have_raw_velocity || sample.have_velocity;
        const double velocity_x = sample.have_raw_velocity
            ? sample.raw_velocity_x_mps
            : sample.velocity_x_mps;
        const double velocity_y = sample.have_raw_velocity
            ? sample.raw_velocity_y_mps
            : sample.velocity_y_mps;
        if (have_velocity)
        {
            // LowState and SportModeState can arrive twice for one tick. A
            // real impulse may appear on the duplicate, so do not discard a
            // large raw-velocity jump merely because dt_s is zero.
            if (have_previous_velocity_)
            {
                velocity_jump_mps = std::hypot(
                    velocity_x - previous_velocity_x_mps_,
                    velocity_y - previous_velocity_y_mps_);
            }
            previous_velocity_x_mps_ = velocity_x;
            previous_velocity_y_mps_ = velocity_y;
            have_previous_velocity_ = true;
        }
        // A planned gait maneuver can create large vertical and lateral IMU
        // accelerations without an external collision (the obstacle lane
        // change is one example). When body velocity is available, use its
        // discontinuity as the collision signature and do not let controlled
        // acceleration create a false impact. Keep acceleration as a
        // fallback for platforms that expose IMU data but no velocity.
        const bool impact_velocity =
            have_velocity &&
            velocity_jump_mps >= config_.impact_velocity_jump_mps;
        const bool impact_acceleration_fallback =
            !have_velocity &&
            std::abs(sample.accel_z_mps2 - config_.gravity_mps2) >=
                config_.impact_extreme_vertical_excess_mps2 &&
            std::hypot(sample.accel_x_mps2, sample.accel_y_mps2) >= 5.0;
        const bool impact = sample.contact_count >= 2 &&
                            (impact_velocity || impact_acceleration_fallback);
        if (impact_latched_)
        {
            if (impact)
                impact_clear_accum_s_ = 0.0;
            else if (dt_s > 0.0)
                impact_clear_accum_s_ += std::min(dt_s, 0.050);
            if (impact_clear_accum_s_ >= config_.impact_release_s)
            {
                impact_latched_ = false;
                impact_clear_accum_s_ = 0.0;
            }
        }
        // Safety events may arrive while a lower-priority response is still
        // active. Keep updating velocity history above, but let a new impact
        // preempt obstacle/slip/turn responses immediately instead of waiting
        // for their duration or the ordinary event cooldown to expire.
        if (active_event)
        {
            const bool active_is_safety =
                active_event_.type == MotionEventType::kImpact ||
                active_event_.type == MotionEventType::kEmergencyStop;
            if (impact && !impact_latched_ && !active_is_safety)
                return Trigger(MotionEventType::kImpact, time_s);
            const MotionEvent obstacle = DetectObstacle(time_s, dt_s, sample);
            const bool active_is_obstacle =
                active_event_.type == MotionEventType::kObstacleLeft ||
                active_event_.type == MotionEventType::kObstacleRight;
            if (obstacle.type != MotionEventType::kNone &&
                !active_is_safety && !active_is_obstacle)
            {
                // A newly observed physical obstacle outranks an ongoing
                // slip/low-friction response and bypasses the old event's
                // cooldown just like impact does.
                return Trigger(obstacle.type, time_s, obstacle.magnitude);
            }
            return active_event_;
        }
        active_event_ = {};
        const bool duplicate_tick_impact = impact && dt_s == 0.0;
        if (!std::isfinite(time_s) || time_s < config_.warmup_s ||
            dt_s < 0.0 ||
            (dt_s == 0.0 && !duplicate_tick_impact) ||
            time_s - last_trigger_time_s_ < config_.cooldown_s)
        {
            low_friction_accum_s_ = 0.0;
            obstacle_accum_s_ = 0.0;
            slip_candidate_age_s_ = 0.0;
            return {};
        }

        if (impact && !impact_latched_)
            return Trigger(MotionEventType::kImpact, time_s);

        const MotionEvent obstacle = DetectObstacle(time_s, dt_s, sample);
        if (obstacle.type != MotionEventType::kNone)
            return Trigger(obstacle.type, time_s, obstacle.magnitude);

        if (sample.have_velocity)
        {
            const double velocity_error =
                std::hypot(sample.velocity_x_mps - nominal.vx_mps,
                           sample.velocity_y_mps - nominal.vy_mps);
            const double error_rise_mpsps =
                have_previous_velocity_error_ && dt_s > 0.0
                    ? (velocity_error - previous_velocity_error_mps_) / dt_s
                    : 0.0;
            const bool have_contacts = sample.contact_count >= 2;
            const bool sharp_error_rise =
                !have_previous_velocity_error_ ||
                error_rise_mpsps >= config_.slip_velocity_error_rise_mpsps;
            const bool slip_level =
                velocity_error >= config_.slip_velocity_error_mps;
            const bool sustained_slip_level =
                have_previous_velocity_error_ &&
                previous_velocity_error_mps_ >= config_.slip_velocity_error_mps;
            if (have_contacts && slip_level &&
                (sharp_error_rise || sustained_slip_level))
                slip_candidate_age_s_ += std::min(dt_s, 0.050);
            else
                slip_candidate_age_s_ = 0.0;
            if (have_contacts &&
                velocity_error >= config_.low_friction_velocity_error_mps &&
                velocity_error < config_.slip_velocity_error_mps &&
                (!have_previous_velocity_error_ ||
                 error_rise_mpsps < config_.slip_velocity_error_rise_mpsps))
                low_friction_accum_s_ += std::min(dt_s, 0.050);
            else
                low_friction_accum_s_ = 0.0;
            previous_velocity_error_mps_ = velocity_error;
            have_previous_velocity_error_ = true;
            if (slip_candidate_age_s_ >= config_.slip_confirm_s)
                return Trigger(MotionEventType::kSlip, time_s);
            if (low_friction_accum_s_ >= config_.low_friction_confirm_s)
                return Trigger(MotionEventType::kLowFriction, time_s);
        }
        else
        {
            slip_candidate_age_s_ = 0.0;
            low_friction_accum_s_ = 0.0;
            have_previous_velocity_error_ = false;
        }
        return {};
    }

    double previous_velocity_x_mps_ = 0.0;
    double previous_velocity_y_mps_ = 0.0;
    bool have_previous_velocity_ = false;
private:
    MotionEvent DetectObstacle(
        double time_s, double dt_s, const MotionSensorSample &sample)
    {
        if (!sample.have_obstacle_scan ||
            !std::isfinite(sample.obstacle_scan_age_s) ||
            sample.obstacle_scan_age_s > config_.max_sensor_age_s)
        {
            obstacle_accum_s_ = 0.0;
            obstacle_first_seen_time_s_ = -1.0;
            obstacle_clear_accum_s_ = 0.0;
            return {};
        }
        const auto blocked = [&](double distance_m, double height_m) {
            return std::isfinite(distance_m) && std::isfinite(height_m) &&
                   distance_m >= config_.obstacle_min_distance_m &&
                   distance_m <= config_.obstacle_max_distance_m &&
                   height_m >= config_.obstacle_min_height_m;
        };
        const bool center = blocked(
            sample.obstacle_center_distance_m, sample.obstacle_center_height_m);
        const bool left = blocked(
            sample.obstacle_left_distance_m, sample.obstacle_left_height_m);
        const bool right = blocked(
            sample.obstacle_right_distance_m, sample.obstacle_right_height_m);
        if (!center && !left && !right)
        {
            obstacle_accum_s_ = 0.0;
            obstacle_first_seen_time_s_ = -1.0;
            obstacle_clear_accum_s_ = obstacle_latched_
                ? obstacle_clear_accum_s_ + std::min(std::max(dt_s, 0.0), 0.050)
                : 0.0;
            if (obstacle_latched_ &&
                obstacle_clear_accum_s_ >= config_.obstacle_rearm_clear_s)
            {
                obstacle_latched_ = false;
                obstacle_clear_accum_s_ = 0.0;
            }
            return {};
        }
        obstacle_clear_accum_s_ = 0.0;
        if (obstacle_latched_)
            return {};
        if (obstacle_first_seen_time_s_ < 0.0 ||
            !std::isfinite(obstacle_first_seen_time_s_))
            obstacle_first_seen_time_s_ = time_s;
        obstacle_accum_s_ += std::min(std::max(dt_s, 0.0), 0.050);
        const double seen_duration_s = time_s - obstacle_first_seen_time_s_;
        if (obstacle_accum_s_ < config_.obstacle_confirm_s &&
            seen_duration_s < config_.obstacle_confirm_s)
            return {};
        bool avoid_left = false;
        if (right && !left && !center)
            avoid_left = true;
        else if (left && !right && !center)
            avoid_left = false;
        else
        {
            const double left_clear = left
                ? sample.obstacle_left_distance_m
                : std::numeric_limits<double>::infinity();
            const double right_clear = right
                ? sample.obstacle_right_distance_m
                : std::numeric_limits<double>::infinity();
            avoid_left = left_clear >= right_clear;
        }
        obstacle_latched_ = true;
        return {avoid_left ? MotionEventType::kObstacleLeft
                           : MotionEventType::kObstacleRight,
                time_s, 0.0, 0.0};
    }

    MotionEvent Trigger(
        MotionEventType type, double time_s, double magnitude = 0.0)
    {
        const double duration =
            type == MotionEventType::kObstacleLeft ||
                    type == MotionEventType::kObstacleRight
                ? config_.obstacle_event_duration_s
                : type == MotionEventType::kLowFriction
                ? config_.low_friction_event_duration_s
                : config_.event_duration_s;
        active_event_ = {
            type, time_s, duration, magnitude};
        last_trigger_time_s_ = time_s;
        slip_candidate_age_s_ = 0.0;
        low_friction_accum_s_ = 0.0;
        if (type == MotionEventType::kImpact)
        {
            impact_latched_ = true;
            impact_clear_accum_s_ = 0.0;
        }
        return active_event_;
    }

    MotionEventDetectorConfig config_;
    MotionEvent active_event_{};
    double low_friction_accum_s_ = 0.0;
    double obstacle_accum_s_ = 0.0;
    double obstacle_first_seen_time_s_ = -1.0;
    double obstacle_clear_accum_s_ = 0.0;
    double slip_candidate_age_s_ = 0.0;
    double previous_velocity_error_mps_ = 0.0;
    double impact_clear_accum_s_ = 0.0;
    bool impact_latched_ = false;
    bool obstacle_latched_ = false;
    bool have_previous_velocity_error_ = false;
    double last_trigger_time_s_ = -1.0e9;
};

struct MotionEventResponseConfig
{
    double max_abs_vx_mps = 0.60;
    double max_abs_vy_mps = 0.65;
    double max_abs_yaw_rate_radps = 0.90;
    double accel_mps2 = 0.80;
    // Obstacle avoidance keeps its turn amplitude, but releases lateral/yaw
    // references more gently so the body does not roll at event exit.
    double obstacle_entry_lateral_accel_mps2 = 0.45;
    double obstacle_entry_yaw_accel_radps2 = 0.70;
    double obstacle_recovery_accel_mps2 = 0.45;
    double obstacle_recovery_lateral_accel_mps2 = 0.75;
    double obstacle_recovery_yaw_accel_radps2 = 1.50;
    double obstacle_recovery_lateral_damping_gain = 0.35;
    double obstacle_recovery_yaw_damping_gain = 2.00;
    // Obstacle response is a lane change: turn briefly, keep a bounded
    // lateral command until the physical obstacle is cleared, then coast.
    double obstacle_turn_duration_s = 1.40;
    double obstacle_lateral_command_duration_s = 11.0;
    double obstacle_hold_yaw_rate_radps = 0.06;
    double obstacle_duty_factor = 0.78;
    double decel_mps2 = 2.50;
    double lateral_accel_mps2 = 1.50;
    double yaw_accel_radps2 = 2.00;
    double step_scale_rate_s = 2.50;
    double duty_rate_s = 0.80;
    double foot_lift_rate_mps = 0.030;
    double turn_speed_scale = 0.78;
    double obstacle_speed_scale = 0.70;
    double obstacle_lateral_speed_mps = 0.45;
    double slip_speed_scale = 0.45;
    double low_friction_speed_scale = 0.45;
    double low_friction_step_scale = 0.50;
    double low_friction_duty_factor = 0.86;
    double low_friction_foot_lift_m = 0.026;
    double emergency_step_scale = 0.45;
    double protective_duty_factor = 0.82;
    double protective_foot_lift_m = 0.024;
    double default_turn_rate_radps = 0.18;
};

struct MotionEventResponse
{
    MotionReference reference{};
    MotionReference target{};
    MotionEventType active_event = MotionEventType::kNone;
    int active_priority = 0;
    bool event_active = false;
    // The selected event timing is exported so the experiment sequencer can
    // implement terminal events without guessing from the reference ramp.
    double active_event_start_time_s = 0.0;
    double active_event_end_time_s = 0.0;
};

inline int MotionEventPriority(MotionEventType type) noexcept
{
    switch (type)
    {
    case MotionEventType::kEmergencyStop:
    case MotionEventType::kImpact:
        return 100;
    case MotionEventType::kObstacleLeft:
    case MotionEventType::kObstacleRight:
        return 80;
    case MotionEventType::kSlip:
    case MotionEventType::kLowFriction:
        return 60;
    case MotionEventType::kTurnLeft:
    case MotionEventType::kTurnRight:
        return 40;
    case MotionEventType::kNone:
        break;
    }
    return 0;
}

class MotionEventResponseLayer
{
public:
    explicit MotionEventResponseLayer(MotionEventResponseConfig config = {})
        : config_(config)
    {
    }

    void Reset() noexcept
    {
        last_active_event_ = MotionEventType::kNone;
        initialized_ = false;
        current_ = {};
        emergency_stop_latched_ = false;
    }

    // Keep the safety target active after a scheduled emergency event expires.
    // The reference itself still reaches zero through the same slew limiter.
    void SetEmergencyStopLatched(bool latched) noexcept
    {
        emergency_stop_latched_ = latched;
    }

    MotionEventResponse Update(
        double time_s,
        double dt_s,
        const MotionReference &nominal,
        const std::vector<MotionEvent> &scheduled_events = {},
        const MotionEvent *sensor_event = nullptr,
        const MotionSensorSample *sensor_sample = nullptr)
    {
        MotionEvent selected{};
        int selected_priority = 0;
        if (emergency_stop_latched_)
        {
            // A latched stop is an absorbing event, but it is intentionally
            // passed through the ordinary reference slew below. This keeps
            // the emergency response bounded and makes the transition matrix
            // test the same continuous interface after the event window.
            selected = {MotionEventType::kEmergencyStop, time_s, 1.0e9, 0.0};
            selected_priority = MotionEventPriority(selected.type);
        }
        else
        {
            auto consider = [&](const MotionEvent &event) {
                if (!event.IsActive(time_s))
                    return;
                const int priority = MotionEventPriority(event.type);
                if (priority > selected_priority ||
                    (priority == selected_priority &&
                     event.start_time_s > selected.start_time_s))
                {
                    selected = event;
                    selected_priority = priority;
                }
            };
            for (const MotionEvent &event : scheduled_events)
                consider(event);
            if (sensor_event != nullptr)
                consider(*sensor_event);
        }

        MotionReference target = ClampReference(nominal);
        ApplyEventTarget(selected, time_s, target);
        if (!initialized_)
        {
            current_ = ClampReference(nominal);
            initialized_ = true;
        }
        const double dt = std::isfinite(dt_s)
            ? std::clamp(dt_s, 0.0, 0.050)
            : 0.0;
        const bool obstacle_recovery =
            selected.type == MotionEventType::kNone &&
            IsObstacleEvent(last_active_event_) &&
            (std::abs(current_.vx_mps - target.vx_mps) > 1.0e-6 ||
             std::abs(current_.vy_mps - target.vy_mps) > 1.0e-6 ||
             std::abs(current_.yaw_rate_radps - target.yaw_rate_radps) >
                 1.0e-6);
        MotionReference slew_target = target;
        if (obstacle_recovery && sensor_sample != nullptr)
        {
            if (sensor_sample->have_velocity)
            {
                const double damped_vy = std::clamp(
                    -config_.obstacle_recovery_lateral_damping_gain *
                        sensor_sample->velocity_y_mps,
                    -config_.max_abs_vy_mps, config_.max_abs_vy_mps);
                slew_target.vy_mps =
                    last_active_event_ == MotionEventType::kObstacleRight
                        ? std::min(0.0, damped_vy)
                        : std::max(0.0, damped_vy);
            }
            if (sensor_sample->have_angular_velocity_z)
            {
                const double damped_yaw = std::clamp(
                    -config_.obstacle_recovery_yaw_damping_gain *
                        sensor_sample->angular_velocity_z_radps,
                    -config_.max_abs_yaw_rate_radps,
                    config_.max_abs_yaw_rate_radps);
                slew_target.yaw_rate_radps =
                    last_active_event_ == MotionEventType::kObstacleRight
                        ? std::min(0.0, damped_yaw)
                        : std::max(0.0, damped_yaw);
            }
        }
        const bool obstacle_entry = IsObstacleEvent(selected.type);
        current_.vx_mps = RateLimitSigned(
            current_.vx_mps, slew_target.vx_mps, dt,
            obstacle_recovery ? config_.obstacle_recovery_accel_mps2
                              : config_.accel_mps2,
            config_.decel_mps2);
        current_.vy_mps = RateLimit(
            current_.vy_mps, slew_target.vy_mps, dt,
            obstacle_recovery
                ? config_.obstacle_recovery_lateral_accel_mps2
                : obstacle_entry
                ? config_.obstacle_entry_lateral_accel_mps2
                : config_.lateral_accel_mps2);
        current_.yaw_rate_radps = RateLimit(
            current_.yaw_rate_radps, slew_target.yaw_rate_radps, dt,
            obstacle_recovery
                ? config_.obstacle_recovery_yaw_accel_radps2
                : obstacle_entry
                ? config_.obstacle_entry_yaw_accel_radps2
                : config_.yaw_accel_radps2);
        current_.step_scale = RateLimit(
            current_.step_scale, target.step_scale, dt,
            config_.step_scale_rate_s);
        current_.duty_factor = RateLimit(
            current_.duty_factor, target.duty_factor, dt,
            config_.duty_rate_s);
        current_.foot_lift_m = RateLimit(
            current_.foot_lift_m, target.foot_lift_m, dt,
            config_.foot_lift_rate_mps);
        if (selected.type != MotionEventType::kNone)
            last_active_event_ = selected.type;
        else if (!obstacle_recovery)
            last_active_event_ = MotionEventType::kNone;
        current_.hold_stance = target.hold_stance;

        MotionEventResponse output;
        output.reference = current_;
        output.target = target;
        output.active_event = selected.type;
        output.active_priority = selected_priority;
        output.event_active = selected.type != MotionEventType::kNone;
        if (output.event_active)
        {
            output.active_event_start_time_s = selected.start_time_s;
            output.active_event_end_time_s =
                selected.start_time_s + selected.duration_s;
        }
        return output;
    }

private:
    static bool IsObstacleEvent(MotionEventType type) noexcept
    {
        return type == MotionEventType::kObstacleLeft ||
               type == MotionEventType::kObstacleRight;
    }

    static double RateLimit(
        double current, double target, double dt, double rate) noexcept
    {
        if (!(dt > 0.0))
            return current;
        if (!(rate > 0.0) || !std::isfinite(current))
            return target;
        if (!std::isfinite(target))
            return current;
        const double delta = std::clamp(target - current, -rate * dt, rate * dt);
        return current + delta;
    }

    static double RateLimitSigned(
        double current, double target, double dt,
        double accel, double decel) noexcept
    {
        if (!(dt > 0.0))
            return current;
        const bool accelerating = std::abs(target) > std::abs(current) &&
                                  current * target >= 0.0;
        const double rate = accelerating ? accel : decel;
        return RateLimit(current, target, dt, rate);
    }

    MotionReference ClampReference(MotionReference reference) const noexcept
    {
        reference.vx_mps = std::clamp(
            reference.vx_mps, -config_.max_abs_vx_mps, config_.max_abs_vx_mps);
        reference.vy_mps = std::clamp(
            reference.vy_mps, -config_.max_abs_vy_mps, config_.max_abs_vy_mps);
        reference.yaw_rate_radps = std::clamp(
            reference.yaw_rate_radps, -config_.max_abs_yaw_rate_radps,
            config_.max_abs_yaw_rate_radps);
        reference.step_scale = std::clamp(reference.step_scale, 0.25, 1.25);
        reference.duty_factor = std::clamp(reference.duty_factor, 0.45, 0.90);
        reference.foot_lift_m = std::clamp(reference.foot_lift_m, 0.015, 0.080);
        return reference;
    }

    void ApplyEventTarget(
        const MotionEvent &event, double time_s,
        MotionReference &target) const noexcept
    {
        const double turn = event.magnitude != 0.0
            ? std::abs(event.magnitude)
            : config_.default_turn_rate_radps;
        const double obstacle_elapsed_s =
            IsObstacleEvent(event.type) && std::isfinite(time_s)
                ? std::max(0.0, time_s - event.start_time_s)
                : 0.0;
        const bool obstacle_lateral_active =
            obstacle_elapsed_s < config_.obstacle_lateral_command_duration_s;
        const bool obstacle_turn_active =
            obstacle_elapsed_s < config_.obstacle_turn_duration_s;
        switch (event.type)
        {
        case MotionEventType::kEmergencyStop:
        case MotionEventType::kImpact:
            target.vx_mps = 0.0;
            target.vy_mps = 0.0;
            target.yaw_rate_radps = 0.0;
            target.step_scale = config_.emergency_step_scale;
            target.duty_factor = config_.protective_duty_factor;
            target.foot_lift_m = config_.protective_foot_lift_m;
            target.hold_stance = event.type == MotionEventType::kEmergencyStop;
            break;
        case MotionEventType::kObstacleLeft:
            target.vx_mps *= config_.obstacle_speed_scale;
            target.vy_mps = obstacle_lateral_active
                ? config_.obstacle_lateral_speed_mps : 0.0;
            target.yaw_rate_radps = obstacle_turn_active
                ? turn : config_.obstacle_hold_yaw_rate_radps;
            target.step_scale = std::min(target.step_scale, 0.70);
            target.duty_factor = std::max(
                target.duty_factor, config_.obstacle_duty_factor);
            break;
        case MotionEventType::kObstacleRight:
            target.vx_mps *= config_.obstacle_speed_scale;
            target.vy_mps = obstacle_lateral_active
                ? -config_.obstacle_lateral_speed_mps : 0.0;
            target.yaw_rate_radps = obstacle_turn_active
                ? -turn : -config_.obstacle_hold_yaw_rate_radps;
            target.step_scale = std::min(target.step_scale, 0.70);
            target.duty_factor = std::max(
                target.duty_factor, config_.obstacle_duty_factor);
            break;
        case MotionEventType::kTurnLeft:
            target.vx_mps *= config_.turn_speed_scale;
            target.yaw_rate_radps = turn;
            break;
        case MotionEventType::kTurnRight:
            target.vx_mps *= config_.turn_speed_scale;
            target.yaw_rate_radps = -turn;
            break;
        case MotionEventType::kSlip:
            target.vx_mps *= config_.slip_speed_scale;
            target.step_scale = std::min(target.step_scale, 0.60);
            target.duty_factor = std::max(
                target.duty_factor, config_.protective_duty_factor);
            break;
        case MotionEventType::kLowFriction:
            target.vx_mps *= config_.low_friction_speed_scale;
            target.step_scale = std::min(
                target.step_scale, config_.low_friction_step_scale);
            target.duty_factor = std::max(
                target.duty_factor, config_.low_friction_duty_factor);
            target.foot_lift_m = std::max(
                target.foot_lift_m, config_.low_friction_foot_lift_m);
            break;
        case MotionEventType::kNone:
            break;
        }
        target = ClampReference(target);
    }

    MotionEventResponseConfig config_;
    MotionReference current_{};
    bool initialized_ = false;
    MotionEventType last_active_event_ = MotionEventType::kNone;
    bool emergency_stop_latched_ = false;
};

} // namespace go2_control
