#pragma once
// CLI parsing for real_leg_lift_go2.

#include <string>
#include <vector>

#include "leg_lift_types.h"

namespace go2_leg {

struct LegLiftCliConfig {
    std::string interface = "lo";
    double duration_s = 16.0;
    std::string csv_path = "go2_real_leg_lift.csv";
    double body_shift_x_m = -0.070;
    double body_shift_y_m = 0.060;
    double foot_lift_height_m = 0.040;
    int cycle_count = 1;
    std::string lift_leg_name = "FR";
    double swing_x_m = 0.0;
    double swing_y_m = 0.0;
    double body_advance_x_m = 0.0;
    double body_advance_y_m = 0.0;
    std::string sequence_file;
    bool world_feedback_enabled = false;
    bool yaw_feedback_enabled = false;
    double tempo_scale = 1.0;
    double support_scale = 1.0;
    bool repeat_sequence = false;
    int max_completed_steps = 0;
    bool adaptive_tempo = false;
    std::vector<StepConfig> steps;
    bool sequence_mode = false;
};

// Returns false on error (message already printed to stderr).
bool ParseLegLiftCli(int argc, const char **argv, LegLiftCliConfig *out);
void PrintLegLiftCliSummary(const LegLiftCliConfig &cfg);

}  // namespace go2_leg
