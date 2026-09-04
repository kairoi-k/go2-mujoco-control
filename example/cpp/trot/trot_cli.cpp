#include "trot_cli.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace go2_trot {

void PrintTrotCliUsage()
{
    std::cerr
        << "Usage: real_trot_go2 <interface> <duration_s> <csv_path>"
           " [--period s] [--duty d] [--step-length m]"
           " [--kernel hand-coded-trot|raibert-trot]"
           " [--gait-pattern diagonal-trot|running-trot|bound|pace|gallop]"
           " [--raibert-velocity-gain s] [--raibert-max-adjustment m]"
           " [--velocity-filter-cutoff-hz f]"
           " [--wall-clock-motion]"
           " [--foot-lift m] [--kp v] [--kd v] [--max-cycles n]"
           " [--no-world-feedback] [--no-attitude-feedback]"
           " [--world-feedback-gain g] [--world-feedback-max m]"
           " [--world-feedback-slew m]"
           " [--no-velocity-feedforward] [--wbc-shadow]"
           " [--wbc-velocity-wrench] [--wbc-velocity-gain s] [--wbc-max-forward-force n]"
           " [--wbc-torque-feedforward] [--wbc-torque-scale s] [--domain-id n]"
           " [--wbc-primary] [--wbc-full] [--cartesian-world] [--preview-horizon n]"
           " [--wbc-reduced-contact-task]"
           " [--wbc-task-torque-feedforward]"
           " [--direction +/-1] [--support-anchor-feedback]"
           " [--support-anchor-gain g] [--event-script path]"
           " [--velocity-command-script path] [--velocity-max-accel a]"
           " [--velocity-max-decel a] [--velocity-max-jerk j]"
           " [--forever] [--stop-file path]"
           " [--auto-environment]"
           " [--gait-phase-offset fraction]"
           " [--terrain-sensor-only]"
           " [--impact-to-emergency-stop-delay s]"
           " [--task stand-walk-lie]"
           " [--goal-x m] [--goal-y m] [--goal-tol m]\n";
}

bool ParseTrotCli(int argc, const char **argv, TrotCliConfig *out, std::string *error_out)
{
    if (out == nullptr) {
        if (error_out) *error_out = "null out";
        return false;
    }
    if (argc < 4) {
        if (error_out) *error_out = "missing required arguments";
        return false;
    }

    TrotCliConfig cfg;
    cfg.interface = argv[1];
    cfg.duration_s = std::stod(argv[2]);
    cfg.csv_path = argv[3];

    for (int i = 4; i < argc; ++i)
    {
        const std::string option = argv[i];
        auto require_value = [&](const char *name) -> std::string {
            if (i + 1 >= argc)
                throw std::invalid_argument(
                    std::string(name) + " requires a value");
            return argv[++i];
        };
        try
        {
            if (option == "--period")
                cfg.params.period_s = std::stod(require_value("--period"));
            else if (option == "--duty")
                cfg.params.duty_factor = std::stod(require_value("--duty"));
            else if (option == "--step-length")
                cfg.params.step_length_m =
                    std::stod(require_value("--step-length"));
            else if (option == "--foot-lift")
                cfg.params.foot_lift_m =
                    std::stod(require_value("--foot-lift"));
            else if (option == "--kp")
                cfg.params.kp = std::stod(require_value("--kp"));
            else if (option == "--kd")
                cfg.params.kd = std::stod(require_value("--kd"));
            else if (option == "--kernel")
                cfg.params.kernel_name = require_value("--kernel");
            else if (option == "--gait-pattern")
            {
                const std::string value = require_value("--gait-pattern");
                if (value == "crawl")
                    throw std::invalid_argument(
                        "retired terrain actuation option 'crawl'; "
                        "use running-trot and read CURRENT.md");
                if (!go2_control::ParseGaitPattern(
                        value.c_str(), cfg.params.gait_pattern))
                    throw std::invalid_argument(
                        "unsupported gait pattern '" + value + "'");
            }
            else if (option == "--raibert-velocity-gain")
                cfg.params.raibert_velocity_gain_s =
                    std::stod(require_value("--raibert-velocity-gain"));
            else if (option == "--raibert-max-adjustment")
                cfg.params.raibert_max_adjustment_m =
                    std::stod(require_value("--raibert-max-adjustment"));
            else if (option == "--velocity-filter-cutoff-hz")
                cfg.params.velocity_filter_cutoff_hz =
                    std::stod(require_value("--velocity-filter-cutoff-hz"));
            else if (option == "--wall-clock-motion")
                cfg.params.wall_clock_motion = true;
            else if (option == "--max-cycles")
                cfg.max_cycles = std::stoi(require_value("--max-cycles"));
            else if (option == "--reactive-events")
                cfg.params.reactive_events = true;
            else if (option == "--auto-environment")
            {
                cfg.params.auto_environment = true;
                cfg.params.reactive_events = true;
            }
            else if (option == "--terrain-sensor-only")
            {
                cfg.params.terrain_enabled = true;
                cfg.params.terrain_sensor_only = true;
            }
            else if (option == "--stage-c-execution" ||
                     option == "--terrain-planner" ||
                     option == "--terrain-leg-order" ||
                     option == "--terrain-advance-body-before-second")
                throw std::invalid_argument(
                    "retired terrain actuation option '" + option +
                    "'; use --terrain-sensor-only and read CURRENT.md");
            else if (option == "--gait-phase-offset")
                cfg.params.gait_phase_offset =
                    std::stod(require_value("--gait-phase-offset"));
            else if (option == "--impact-to-emergency-stop-delay")
            {
                cfg.params.impact_to_emergency_stop_delay_s =
                    std::stod(require_value("--impact-to-emergency-stop-delay"));
                cfg.params.reactive_events = true;
            }
            else if (option == "--forever")
                cfg.continuous_mode = true;
            else if (option == "--stop-file")
                cfg.stop_file_path = require_value("--stop-file");
            else if (option == "--velocity-command-script")
            {
                cfg.params.velocity_command_script_path =
                    require_value("--velocity-command-script");
                cfg.params.runtime_velocity_command = true;
            }
            else if (option == "--velocity-max-accel")
                cfg.params.velocity_command_shaper.max_accel_mps2 =
                    std::stod(require_value("--velocity-max-accel"));
            else if (option == "--velocity-max-decel")
                cfg.params.velocity_command_shaper.max_decel_mps2 =
                    std::stod(require_value("--velocity-max-decel"));
            else if (option == "--velocity-max-jerk")
                cfg.params.velocity_command_shaper.max_jerk_mps3 =
                    std::stod(require_value("--velocity-max-jerk"));
            else if (option == "--velocity-max-tracking-lead")
                cfg.params.velocity_command_shaper.max_tracking_lead_mps =
                    std::stod(require_value("--velocity-max-tracking-lead"));
            else if (option == "--event-script")
                cfg.params.event_script_path = require_value("--event-script");
            else if (option == "--task")
            {
                const std::string task_name = require_value("--task");
                if (task_name != "stand-walk-lie")
                    throw std::invalid_argument(
                        "unsupported task '" + task_name +
                        "' (use stand-walk-lie)");
                cfg.task_mode = true;
            }
            else if (option == "--goal-x")
            {
                cfg.goal.x = std::stod(require_value("--goal-x"));
                cfg.goal.enabled = true;
            }
            else if (option == "--goal-y")
            {
                cfg.goal.y = std::stod(require_value("--goal-y"));
                cfg.goal.enabled = true;
            }
            else if (option == "--goal-tol")
            {
                cfg.goal.tol = std::stod(require_value("--goal-tol"));
                cfg.goal.enabled = true;
            }
            else if (option == "--domain-id")
                cfg.domain_id = std::stoi(require_value("--domain-id"));
            else if (option == "--direction")
                cfg.params.direction_sign =
                    std::stod(require_value("--direction"));
            else if (option == "--support-anchor-feedback")
                cfg.params.support_anchor_feedback = true;
            else if (option == "--support-anchor-gain")
                cfg.params.support_anchor_gain =
                    std::stod(require_value("--support-anchor-gain"));
            else if (option == "--no-world-feedback")
                cfg.params.world_feedback = false;
            else if (option == "--world-feedback-gain")
                cfg.params.world_feedback_gain =
                    std::stod(require_value("--world-feedback-gain"));
            else if (option == "--world-feedback-max")
                cfg.params.world_feedback_max_m =
                    std::stod(require_value("--world-feedback-max"));
            else if (option == "--world-feedback-slew")
                cfg.params.world_feedback_slew_m =
                    std::stod(require_value("--world-feedback-slew"));
            else if (option == "--no-attitude-feedback")
                cfg.params.attitude_feedback = false;
            else if (option == "--no-velocity-feedforward")
                cfg.params.velocity_feedforward = false;
            else if (option == "--wbc-shadow")
                cfg.params.wbc_shadow = true;
            else if (option == "--step-plan")
            {
                // 格式 "32:0.110,64:0.091"(cycle:step_length)
                const std::string plan = require_value("--step-plan");
                std::stringstream ss(plan);
                std::string seg;
                while (std::getline(ss, seg, ','))
                {
                    const std::size_t colon = seg.find(':');
                    if (colon == std::string::npos)
                        throw std::runtime_error(
                            "bad --step-plan segment: " + seg);
                    const int cycle = std::stoi(seg.substr(0, colon));
                    const double step = std::stod(seg.substr(colon + 1));
                    cfg.params.step_plan.emplace_back(cycle, step);
                }
            }
            else if (option == "--period-plan")
            {
                std::string plan = require_value("--period-plan");
                std::stringstream ss(plan);
                std::string seg;
                while (std::getline(ss, seg, ','))
                {
                    const std::size_t colon = seg.find(':');
                    if (colon == std::string::npos)
                        throw std::runtime_error(
                            "bad --period-plan segment: " + seg);
                    const int cycle = std::stoi(seg.substr(0, colon));
                    const double period = std::stod(seg.substr(colon + 1));
                    cfg.params.period_plan.emplace_back(cycle, period);
                }
            }
            else if (option == "--bounce-amp")
                cfg.params.bounce_acc_amp = std::stod(require_value("--bounce-amp"));
            else if (option == "--tau-limit")
                cfg.params.tau_limit_nm = std::stod(require_value("--tau-limit"));
            else if (option == "--turn-rate")
                cfg.params.turn_rate_radps = std::stod(require_value("--turn-rate"));
            else if (option == "--wbc-primary")
            {
                cfg.params.wbc_primary = true;
                cfg.params.wbc_shadow = true;
                cfg.params.wbc_velocity_wrench = true;
                cfg.params.wbc_velocity_gain_s_inv = 6.0;
            }
            else if (option == "--wbc-full")
            {
                cfg.params.wbc_full = true;
                cfg.params.wbc_primary = true;
                cfg.params.wbc_shadow = true;
                cfg.params.wbc_velocity_wrench = true;
                cfg.params.wbc_velocity_gain_s_inv = 6.0;
                cfg.params.wbc_reduced_contact_task = false;
                if (cfg.params.preview_horizon_steps <= 0)
                    cfg.params.preview_horizon_steps = 4;
            }
            else if (option == "--cartesian-world")
            {
                cfg.params.cartesian_world = true;
                cfg.params.wbc_full = true;
                cfg.params.wbc_primary = true;
                cfg.params.wbc_shadow = true;
                cfg.params.wbc_velocity_wrench = true;
                cfg.params.wbc_velocity_gain_s_inv = 6.0;
                cfg.params.wbc_reduced_contact_task = false;
                if (cfg.params.preview_horizon_steps <= 0)
                    cfg.params.preview_horizon_steps = 4;
            }
            else if (option == "--preview-horizon")
                cfg.params.preview_horizon_steps =
                    std::stoi(require_value("--preview-horizon"));
            else if (option == "--wbc-velocity-wrench")
            {
                cfg.params.wbc_velocity_wrench = true;
                cfg.params.wbc_shadow = true;
            }
            else if (option == "--impulse")
            {
                // 冲量主控: 线动量任务(加速度域 M x a)参考生成。
                // 关闭 wbc_velocity_wrench(线性力域), 避免双推力源冲突
                // (wrench 修复后两个推力都能出去, 叠加导致过推失控)。
                cfg.params.impulse = true;
                cfg.params.wbc_primary = true;
                cfg.params.wbc_shadow = true;
                cfg.params.wbc_velocity_wrench = false;
            }
            else if (option == "--wbc-reduced-contact-task")
            {
                cfg.params.wbc_reduced_contact_task = true;
                cfg.params.wbc_shadow = true;
            }
            else if (option == "--wbc-task-torque-feedforward")
            {
                cfg.params.wbc_task_torque_feedforward = true;
                cfg.params.wbc_torque_feedforward = true;
                cfg.params.wbc_reduced_contact_task = true;
                cfg.params.wbc_shadow = true;
            }
            else if (option == "--wbc-velocity-gain")
                cfg.params.wbc_velocity_gain_s_inv =
                    std::stod(require_value("--wbc-velocity-gain"));
            else if (option == "--wbc-max-forward-force")
                cfg.params.wbc_max_forward_force_n =
                    std::stod(require_value("--wbc-max-forward-force"));
            else if (option == "--wbc-torque-feedforward")
            {
                cfg.params.wbc_torque_feedforward = true;
                cfg.params.wbc_shadow = true;
            }
            else if (option == "--wbc-torque-scale")
                cfg.params.wbc_torque_scale =
                    std::stod(require_value("--wbc-torque-scale"));
            else
                throw std::invalid_argument(
                    "unknown option " + option);
        }
        catch (const std::exception &error)
        {
            if (error_out) *error_out = error.what();
            return false;
        }
    }
    if (cfg.params.runtime_velocity_command &&
        (cfg.params.cartesian_world || !cfg.params.wbc_full ||
         cfg.params.reactive_events || cfg.params.auto_environment ||
         !cfg.params.event_script_path.empty() ||
         cfg.params.gait_pattern != go2_control::GaitPattern::kRunningTrot ||
         !std::isfinite(cfg.params.velocity_command_shaper.max_accel_mps2) ||
         cfg.params.velocity_command_shaper.max_accel_mps2 <= 0.0 ||
         !std::isfinite(cfg.params.velocity_command_shaper.max_decel_mps2) ||
         cfg.params.velocity_command_shaper.max_decel_mps2 <= 0.0 ||
         !std::isfinite(cfg.params.velocity_command_shaper.max_jerk_mps3) ||
         cfg.params.velocity_command_shaper.max_jerk_mps3 <= 0.0 ||
         !std::isfinite(cfg.params.velocity_command_shaper.max_tracking_lead_mps) ||
         cfg.params.velocity_command_shaper.max_tracking_lead_mps < 0.0))
    {
        if (error_out)
            *error_out = "runtime velocity command requires running-trot + wbc-full and positive shaper limits";
        return false;
    }

    if (!std::isfinite(cfg.params.gait_phase_offset) ||
        cfg.params.gait_phase_offset < 0.0 ||
        cfg.params.gait_phase_offset >= 1.0)
    {
        if (error_out) *error_out = "gait phase offset must be in [0,1)";
        return false;
    }
    if (!cfg.params.event_script_path.empty() &&
        !go2_control::LoadMotionEventScript(
            cfg.params.event_script_path, cfg.params.event_schedule,
            error_out))
    {
        if (error_out && error_out->empty())
            *error_out = "invalid event script";
        return false;
    }
    if (cfg.params.runtime_velocity_command &&
        !LoadVelocityCommandProfile(
            cfg.params.velocity_command_script_path,
            cfg.params.velocity_command_profile, error_out))
    {
        return false;
    }

    const double min_duty =
        !cfg.params.wbc_full ? 0.50
        : (cfg.params.gait_pattern == go2_control::GaitPattern::kGallop ||
           cfg.params.gait_pattern == go2_control::GaitPattern::kBound
               ? 0.25
               : 0.35);
    if (!(cfg.params.period_s >= (cfg.params.wbc_full ? 0.08 : 0.35) &&
          cfg.params.period_s <= 3.0) ||
        (cfg.params.kernel_name != "hand-coded-trot" &&
         cfg.params.kernel_name != "raibert-trot") ||
        !std::isfinite(cfg.params.raibert_velocity_gain_s) ||
        !(cfg.params.raibert_velocity_gain_s >= 0.0 &&
          cfg.params.raibert_velocity_gain_s <= 1.0) ||
        !std::isfinite(cfg.params.raibert_max_adjustment_m) ||
        !(cfg.params.raibert_max_adjustment_m >= 0.0 &&
          cfg.params.raibert_max_adjustment_m <=
              (cfg.params.wbc_full ? 0.16 : 0.10)) ||
        !std::isfinite(cfg.params.velocity_filter_cutoff_hz) ||
        !(cfg.params.velocity_filter_cutoff_hz >= 0.0 &&
          cfg.params.velocity_filter_cutoff_hz <= 50.0) ||
        !std::isfinite(cfg.params.world_feedback_gain) ||
        !(cfg.params.world_feedback_gain >= 0.0 &&
          cfg.params.world_feedback_gain <= 5.0) ||
        !std::isfinite(cfg.params.world_feedback_max_m) ||
        !(cfg.params.world_feedback_max_m >= 0.0 &&
          cfg.params.world_feedback_max_m <= 0.10) ||
        !std::isfinite(cfg.params.world_feedback_slew_m) ||
        !(cfg.params.world_feedback_slew_m >= 0.0 &&
          cfg.params.world_feedback_slew_m <= 0.02) ||
        !std::isfinite(cfg.params.wbc_velocity_gain_s_inv) ||
        !(cfg.params.wbc_velocity_gain_s_inv >= 0.0 &&
          cfg.params.wbc_velocity_gain_s_inv <= 10.0) ||
        !std::isfinite(cfg.params.wbc_max_forward_force_n) ||
        !(cfg.params.wbc_max_forward_force_n >= 0.0 &&
          cfg.params.wbc_max_forward_force_n <= 50.0) ||
        !std::isfinite(cfg.params.wbc_torque_scale) ||
        !(cfg.params.wbc_torque_scale > 0.0 &&
          cfg.params.wbc_torque_scale <= kWbcTorqueFeedforwardMaxScale) ||
        !std::isfinite(cfg.params.impact_to_emergency_stop_delay_s) ||
        (cfg.params.impact_to_emergency_stop_delay_s < -1.0 ||
         cfg.params.impact_to_emergency_stop_delay_s > 5.0) ||
        !(cfg.params.duty_factor > min_duty &&
          cfg.params.duty_factor <
              (cfg.params.wbc_full ? 0.85 : 0.80)) ||
        !(cfg.params.step_length_m > 0.0 &&
          cfg.params.step_length_m <=
              (cfg.params.wbc_full ? 1.20 : 0.30)) ||
        !(cfg.params.foot_lift_m >= 0.02 &&
          cfg.params.foot_lift_m <= (cfg.params.wbc_full ? 0.35 : 0.20)) ||
        !(cfg.params.kp > 0.0 && cfg.params.kp <= 150.0) ||
        !(cfg.params.kd > 0.0 && cfg.params.kd <= 15.0) ||
        cfg.max_cycles < 0 || cfg.domain_id < 0 || cfg.domain_id > 232 ||
        !(std::abs(std::abs(cfg.params.direction_sign) - 1.0) < 1e-9) ||
        !(cfg.params.support_anchor_gain >= 0.0 &&
          cfg.params.support_anchor_gain <= 1.0) ||
        cfg.params.preview_horizon_steps < 0 ||
        cfg.params.preview_horizon_steps >
            go2_control::kPreviewHorizonMaxSteps ||
        (cfg.task_mode && cfg.continuous_mode) ||
        (cfg.task_mode &&
         (!std::isfinite(cfg.duration_s) || cfg.duration_s <= 0.0)) ||
        (cfg.goal.enabled &&
         (!std::isfinite(cfg.goal.x) || !std::isfinite(cfg.goal.y) ||
          !std::isfinite(cfg.goal.tol) || cfg.goal.tol < 0.02 ||
          cfg.goal.tol > 1.0)) ||
        (cfg.goal.enabled && cfg.params.cartesian_world))
    {
        if (error_out) *error_out = "Invalid trot parameters";
        return false;
    }

    *out = std::move(cfg);
    return true;
}

void PrintTrotCliSummary(const TrotCliConfig &cfg)
{
    const auto &params = cfg.params;
    std::cout << "Diagonal trot target\n"
              << "  DDS domain=" << cfg.domain_id << "\n"
              << "  period=" << params.period_s << " s\n"
              << "  nominal speed="
              << params.step_length_m / params.period_s << " m/s\n"
              << "  duty=" << params.duty_factor << "\n"
              << "  step length=" << params.step_length_m << " m\n"
              << "  direction=" << params.direction_sign << "\n"
              << "  foot lift=" << params.foot_lift_m << " m\n"
              << "  kp/kd=" << params.kp << "/" << params.kd << "\n"
              << "  kernel=" << params.kernel_name << "\n"
              << "  gait_pattern="
              << go2_control::GaitPatternName(params.gait_pattern) << "\n"
              << "  wbc_primary=" << (params.wbc_primary ? "on" : "off") << "\n"
              << "  wbc_full=" << (params.wbc_full ? "on" : "off") << "\n"
              << "  cartesian_world="
              << (params.cartesian_world ? "on" : "off") << "\n"
              << "  auto_environment=" << (params.auto_environment ? "on" : "off") << "\n"
              << "  terrain_sensor_only="
              << (params.terrain_sensor_only ? "on" : "off") << "\n"
              << "  reactive_events="
              << ((params.reactive_events || params.auto_environment || !params.event_schedule.empty()) ? "on" : "off") << "\n"
              << "  impact_to_emergency_stop_delay="
              << params.impact_to_emergency_stop_delay_s << " s\n"
              << "  preview_horizon=" << params.preview_horizon_steps << "\n"
              << "  impulse=" << (params.impulse ? "on" : "off") << "\n"
              << "  event_script="
              << (params.event_script_path.empty()
                      ? "off"
                      : (params.event_script_path + " (" +
                         std::to_string(params.event_schedule.size()) +
                         " events)"))
              << "\n"
              << "  runtime_velocity_command="
              << (params.runtime_velocity_command ? "on" : "off")
              << (params.runtime_velocity_command
                      ? (" (" + params.velocity_command_script_path + ")")
                      : "")
              << "\n"
              << "  task="
              << (cfg.task_mode ? "stand-walk-lie" : "trot-only") << "\n"
              << "  goal="
              << (cfg.goal.enabled
                      ? ("world (" + std::to_string(cfg.goal.x) + ", " +
                         std::to_string(cfg.goal.y) + ") tol=" +
                         std::to_string(cfg.goal.tol) + " m")
                      : "off")
              << "\n"
              << "  run_mode=" << (cfg.continuous_mode ? "continuous" : "bounded")
              << "\n"
              << "Press enter to start\n";
}

}  // namespace go2_trot
