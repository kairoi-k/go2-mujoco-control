#include <cstdio>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

#include "motion_event_response.h"

namespace
{

bool Check(bool condition, const char *message)
{
    if (!condition)
        std::cerr << message << "\n";
    return condition;
}

} // namespace

int main()
{
    using namespace go2_control;
    MotionEventResponseLayer layer;
    bool passed = true;
    const std::string script_path = "/tmp/go2_motion_event_test.txt";
    {
        std::ofstream script(script_path);
        script << "# start duration event magnitude\n"
               << "0.5 1.0 turn_left 0.20\n"
               << "1.0 0.5 emergency_stop\n";
    }
    std::vector<MotionEvent> parsed_events;
    std::string parse_error;
    passed &= Check(
        LoadMotionEventScript(script_path, parsed_events, &parse_error) &&
            parsed_events.size() == 2 &&
            parsed_events[0].type == MotionEventType::kTurnLeft,
        "Event script parser failed.");
    std::remove(script_path.c_str());

    MotionReference nominal;
    nominal.vx_mps = 0.30;
    nominal.duty_factor = 0.75;
    nominal.foot_lift_m = 0.020;

    MotionEventDetector detector;
    MotionSensorSample sensor;
    sensor.contact_count = 4;
    sensor.have_velocity = true;
    sensor.velocity_x_mps = 0.0;
    auto automatic = detector.Observe(
        1.6, 0.002, sensor, nominal);
    sensor.velocity_x_mps = 0.8;
    automatic = detector.Observe(
        1.602, 0.002, sensor, nominal);
    passed &= Check(
        automatic.type == MotionEventType::kImpact,
        "Impact detector did not trigger.");
    for (int i = 1; i <= 500; ++i)
        automatic = detector.Observe(1.602 + i * 0.002, 0.002, sensor, nominal);
    passed &= Check(
        automatic.type == MotionEventType::kNone,
        "Impact detector retriggered while the impact signal stayed high.");
    sensor.velocity_x_mps = 0.0;
    for (int i = 0; i < 300; ++i)
        detector.Observe(2.602 + i * 0.002, 0.002, sensor, nominal);
    sensor.velocity_x_mps = 0.8;
    automatic = detector.Observe(3.2, 0.002, sensor, nominal);
    passed &= Check(
        automatic.type == MotionEventType::kImpact,
        "Impact detector did not re-arm after the signal cleared.");
    detector.Reset();
    sensor = {};
    sensor.contact_count = 4;
    sensor.have_velocity = true;
    sensor.velocity_x_mps = 0.0;
    detector.Observe(10.0, 0.002, sensor, nominal);
    sensor.velocity_x_mps = 0.8;
    automatic = detector.Observe(10.0, 0.0, sensor, nominal);
    passed &= Check(
        automatic.type == MotionEventType::kImpact,
        "Impact detector missed a duplicate-tick velocity jump.");
    detector.Reset();
    sensor = {};
    sensor.contact_count = 4;
    sensor.have_velocity = true;
    sensor.velocity_x_mps = 0.8;
    for (int i = 0; i < 100; ++i)
        automatic = detector.Observe(1.6 + i * 0.002, 0.002, sensor, nominal);
    passed &= Check(
        automatic.type == MotionEventType::kSlip,
        "Slip detector did not trigger.");

    detector.Reset();
    sensor = {};
    sensor.contact_count = 4;
    sensor.have_obstacle_scan = true;
    sensor.obstacle_scan_age_s = 0.0;
    sensor.obstacle_center_distance_m = 0.75;
    sensor.obstacle_center_height_m = 0.25;
    for (int i = 0; i < 4; ++i)
        automatic = detector.Observe(
            1.6 + i * 0.02, 0.02, sensor, nominal);
    passed &= Check(
        automatic.type == MotionEventType::kObstacleLeft,
        "Centered obstacle did not select a clear-side response.");
    automatic = detector.Observe(9.8, 0.02, sensor, nominal);
    sensor.obstacle_center_distance_m = -1.0;
    sensor.obstacle_center_height_m = 0.0;
    detector.Observe(9.82, 0.02, sensor, nominal);
    sensor.obstacle_center_distance_m = 0.75;
    sensor.obstacle_center_height_m = 0.25;
    automatic = detector.Observe(9.84, 0.02, sensor, nominal);
    passed &= Check(
        automatic.type == MotionEventType::kNone,
        "Obstacle detector retriggered after a transient scan gap.");

    detector.Reset();
    sensor = {};
    sensor.contact_count = 4;
    sensor.have_obstacle_scan = true;
    sensor.obstacle_scan_age_s = 0.0;
    sensor.obstacle_right_distance_m = 0.70;
    sensor.obstacle_right_height_m = 0.25;
    for (int i = 0; i < 4; ++i)
        automatic = detector.Observe(
            1.6 + i * 0.02, 0.02, sensor, nominal);
    passed &= Check(
        automatic.type == MotionEventType::kObstacleLeft,
        "Right-side obstacle did not select a left response.");

    detector.Reset();
    sensor = {};
    sensor.contact_count = 4;
    sensor.have_velocity = true;
    sensor.velocity_x_mps = 0.0;
    for (int i = 0; i < 20; ++i)
        automatic = detector.Observe(
            1.6 + i * 0.02, 0.02, sensor, nominal);
    passed &= Check(
        automatic.type == MotionEventType::kLowFriction,
        "Sustained moderate velocity mismatch did not trigger low friction.");

    std::vector<MotionEvent> events = {
        {MotionEventType::kTurnLeft, 0.0, 2.0, 0.18},
        {MotionEventType::kEmergencyStop, 0.2, 1.0, 0.0},
    };
    auto output = layer.Update(0.0, 0.0, nominal, events);
    passed &= Check(
        output.active_event == MotionEventType::kTurnLeft,
        "Turn event was not selected at t=0.");
    passed &= Check(
        output.reference.vx_mps == nominal.vx_mps,
        "Initial reference was not initialized from nominal.");

    output = layer.Update(0.2, 0.002, nominal, events);
    passed &= Check(
        output.active_event == MotionEventType::kEmergencyStop &&
            output.active_priority > MotionEventPriority(MotionEventType::kTurnLeft) &&
            output.reference.hold_stance,
        "Emergency stop did not override turn by priority.");
    passed &= Check(
        output.reference.vx_mps < nominal.vx_mps &&
            output.reference.vx_mps > 0.0,
        "Emergency stop bypassed the deceleration rate limit.");
    for (int i = 0; i < 200; ++i)
        output = layer.Update(0.202 + i * 0.002, 0.002, nominal, events);
    passed &= Check(
        std::abs(output.reference.vx_mps) < 1.0e-9,
        "Emergency stop did not converge to zero speed.");

    output = layer.Update(1.4, 0.002, nominal, events);
    passed &= Check(
        output.active_event == MotionEventType::kTurnLeft,
        "Turn event did not resume after emergency event ended.");
    passed &= Check(
        output.reference.yaw_rate_radps > 0.0,
        "Turn reference did not have the expected sign.");
    passed &= Check(!output.reference.hold_stance, "Stance hold did not release after emergency event.");

    MotionEventResponseConfig obstacle_config;
    MotionEventResponseLayer obstacle_layer(obstacle_config);
    const std::vector<MotionEvent> obstacle_events = {
        {MotionEventType::kObstacleRight, 0.0, 20.0, 0.48},
    };
    auto obstacle_output = obstacle_layer.Update(
        0.0, 0.0, nominal, obstacle_events);
    passed &= Check(
        obstacle_output.target.vy_mps < 0.0 &&
            obstacle_output.target.yaw_rate_radps < 0.0,
        "Obstacle entry reference sign was wrong.");
    obstacle_output = obstacle_layer.Update(
        1.5, 0.002, nominal, obstacle_events);
    passed &= Check(
        obstacle_output.target.vy_mps < 0.0 &&
            obstacle_output.target.yaw_rate_radps < 0.0 &&
            std::abs(obstacle_output.target.yaw_rate_radps) < 0.20,
        "Obstacle turn did not transition to the bounded hold yaw.");
    obstacle_output = obstacle_layer.Update(
        12.0, 0.002, nominal, obstacle_events);
    passed &= Check(
        obstacle_output.target.vy_mps == 0.0,
        "Obstacle lateral command did not end after clearance.");
    MotionSensorSample obstacle_sensor{};
    obstacle_sensor.have_velocity = true;
    obstacle_sensor.velocity_y_mps = 0.10;
    obstacle_sensor.have_angular_velocity_z = true;
    obstacle_sensor.angular_velocity_z_radps = 0.10;
    obstacle_output = obstacle_layer.Update(
        20.1, 0.002, nominal, {}, nullptr, &obstacle_sensor);
    passed &= Check(
        obstacle_output.reference.vy_mps <= 0.0 &&
            obstacle_output.reference.yaw_rate_radps <= 0.0,
        "Obstacle recovery produced an opposite-sign command.");
    if (!passed)
        return 1;
    std::cout << "Motion event response checks passed.\n";
    return 0;
}
