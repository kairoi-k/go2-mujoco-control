#include "leg_lift_cli.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace go2_leg {

bool ParseLegLiftCli(int argc, const char **argv, LegLiftCliConfig *out)
{
    if (out == nullptr)
        return false;

    LegLiftCliConfig cfg;

    if (argc >= 2)
        cfg.interface = argv[1];
    if (argc >= 3)
        cfg.duration_s = std::stod(argv[2]);
    if (argc >= 4)
        cfg.csv_path = argv[3];
    if (argc >= 5)
        cfg.body_shift_x_m = std::stod(argv[4]);
    if (argc >= 6)
        cfg.body_shift_y_m = std::stod(argv[5]);
    if (argc >= 7)
        cfg.foot_lift_height_m = std::stod(argv[6]);
    if (argc >= 8)
        cfg.cycle_count = std::stoi(argv[7]);
    if (argc >= 9)
        cfg.lift_leg_name = argv[8];
    if (argc >= 10)
        cfg.swing_x_m = std::stod(argv[9]);
    if (argc >= 11)
        cfg.swing_y_m = std::stod(argv[10]);
    if (argc >= 12)
        cfg.body_advance_x_m = std::stod(argv[11]);
    if (argc >= 13)
        cfg.body_advance_y_m = std::stod(argv[12]);
    if (argc >= 14)
    {
        if (argc < 15 || std::string(argv[13]) != "--sequence-file")
        {
            std::cerr << "Expected --sequence-file <path>" << std::endl;
            return false;
        }
        cfg.sequence_file = argv[14];
    }
    cfg.cycle_count = std::max(1, cfg.cycle_count);

    if (argc > 15)
    {
        for (int arg_index = 15; arg_index < argc; ++arg_index)
        {
            const std::string option = argv[arg_index];
            if (option == "--world-feedback")
            {
                cfg.world_feedback_enabled = true;
            }
            else if (option == "--yaw-feedback")
            {
                cfg.yaw_feedback_enabled = true;
            }
            else if (option == "--tempo-scale")
            {
                if (arg_index + 1 >= argc)
                {
                    std::cerr << "--tempo-scale requires a value" << std::endl;
                    return false;
                }
                try
                {
                    cfg.tempo_scale = std::stod(argv[++arg_index]);
                }
                catch (const std::exception &error)
                {
                    std::cerr << "Invalid --tempo-scale: " << error.what()
                              << std::endl;
                    return false;
                }
            }
            else if (option == "--adaptive-tempo")
            {
                cfg.adaptive_tempo = true;
            }
            else if (option == "--support-scale")
            {
                if (arg_index + 1 >= argc)
                {
                    std::cerr << "--support-scale requires a value"
                              << std::endl;
                    return false;
                }
                try
                {
                    cfg.support_scale = std::stod(argv[++arg_index]);
                }
                catch (const std::exception &error)
                {
                    std::cerr << "Invalid --support-scale: " << error.what()
                              << std::endl;
                    return false;
                }
            }
            else if (option == "--repeat-sequence")
            {
                cfg.repeat_sequence = true;
            }
            else if (option == "--infinite")
            {
                cfg.repeat_sequence = true;
                cfg.duration_s = std::numeric_limits<double>::infinity();
            }
            else if (option == "--max-steps")
            {
                if (arg_index + 1 >= argc)
                {
                    std::cerr << "--max-steps requires a value" << std::endl;
                    return false;
                }
                try
                {
                    cfg.max_completed_steps = std::stoi(argv[++arg_index]);
                }
                catch (const std::exception &error)
                {
                    std::cerr << "Invalid --max-steps: " << error.what()
                              << std::endl;
                    return false;
                }
            }
            else
            {
                std::cerr << "Unknown controller option: " << option
                          << std::endl;
                return false;
            }
        }
    }

    if (cfg.yaw_feedback_enabled && !cfg.world_feedback_enabled)
    {
        std::cerr << "--yaw-feedback requires --world-feedback" << std::endl;
        return false;
    }
    if (!(cfg.tempo_scale >= 0.25 && cfg.tempo_scale <= 1.0))
    {
        std::cerr << "--tempo-scale must be in [0.25, 1.0]" << std::endl;
        return false;
    }
    if (!(cfg.support_scale >= 0.25 && cfg.support_scale <= 1.0))
    {
        std::cerr << "--support-scale must be in [0.25, 1.0]" << std::endl;
        return false;
    }
    if (cfg.max_completed_steps < 0)
    {
        std::cerr << "--max-steps must be non-negative" << std::endl;
        return false;
    }
    if (cfg.max_completed_steps > 0 && !cfg.repeat_sequence)
    {
        std::cerr << "--max-steps requires repeat mode" << std::endl;
        return false;
    }
    if (cfg.world_feedback_enabled && cfg.sequence_file.empty())
    {
        std::cerr << "World feedback requires --sequence-file" << std::endl;
        return false;
    }
    if (cfg.repeat_sequence && cfg.sequence_file.empty())
    {
        std::cerr << "Repeat mode requires --sequence-file" << std::endl;
        return false;
    }
    if (cfg.sequence_file.empty() && cfg.cycle_count > 1 &&
        (std::abs(cfg.swing_x_m) > 1e-9 || std::abs(cfg.swing_y_m) > 1e-9 ||
         std::abs(cfg.body_advance_x_m) > 1e-9 ||
         std::abs(cfg.body_advance_y_m) > 1e-9))
    {
        std::cerr
            << "Swing and body advance are single-step parameters; "
               "use cycle_count=1"
            << std::endl;
        return false;
    }

    go2::Leg lift_leg;
    try
    {
        lift_leg = ParseLegName(cfg.lift_leg_name);
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << std::endl;
        return false;
    }

    if (!cfg.sequence_file.empty())
    {
        if (!LoadStepSequence(cfg.sequence_file, cfg.steps))
            return false;
        cfg.sequence_mode = true;
    }
    else
    {
        StepConfig step{
            lift_leg,
            cfg.body_shift_x_m,
            cfg.body_shift_y_m,
            cfg.foot_lift_height_m,
            cfg.swing_x_m,
            cfg.swing_y_m,
            cfg.body_advance_x_m,
            cfg.body_advance_y_m};
        cfg.steps.assign(static_cast<std::size_t>(cfg.cycle_count), step);
        cfg.sequence_mode = false;
    }

    *out = std::move(cfg);
    return true;
}

void PrintLegLiftCliSummary(const LegLiftCliConfig &cfg)
{
    std::cout << "Interface: " << cfg.interface << "\n"
              << "Duration: " << cfg.duration_s << " s\n"
              << "CSV: " << cfg.csv_path << "\n"
              << "Body shift: x=" << cfg.body_shift_x_m
              << " m, y=" << cfg.body_shift_y_m << " m\n"
              << "Swing target: x=" << cfg.swing_x_m
              << " m, y=" << cfg.swing_y_m << " m\n"
              << "Body advance: x=" << cfg.body_advance_x_m
              << " m, y=" << cfg.body_advance_y_m << " m\n"
              << cfg.lift_leg_name << " foot lift: " << cfg.foot_lift_height_m
              << " m\n"
              << "Lift cycles: " << cfg.cycle_count << "\n"
              << "Sequence steps: " << cfg.steps.size() << "\n"
              << "Sequence file: "
              << (cfg.sequence_file.empty() ? "repeated step" : cfg.sequence_file)
              << "\n"
              << "World feedback: "
              << (cfg.world_feedback_enabled ? "x/y closed-loop" : "disabled")
              << "\n"
              << "Repeat sequence: "
              << (cfg.repeat_sequence ? "enabled" : "disabled") << "\n"
              << "Max completed steps: " << cfg.max_completed_steps << "\n"
              << "Yaw feedback: "
              << (cfg.yaw_feedback_enabled ? "heading hold" : "disabled")
              << "\n"
              << "Tempo scale: " << cfg.tempo_scale << "x\n"
              << "Support scale: " << cfg.support_scale << "x\n"
              << "Tempo mode: "
              << (cfg.adaptive_tempo ? "adaptive governor" : "phase-safe")
              << "\n"
              << "Press enter to start\n";
}

}  // namespace go2_leg
