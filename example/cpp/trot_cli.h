#pragma once
// CLI parsing for real_trot_go2.

#include <string>

#include "trot_types.h"

namespace go2_trot {

struct TrotCliConfig {
    std::string interface;
    double duration_s = 0.0;
    std::string csv_path;
    TrotParams params{};
    int max_cycles = 0;
    int domain_id = 1;
    bool continuous_mode = false;
    bool task_mode = false;
    std::string stop_file_path;
};

// Returns false and writes message to *error_out on failure.
bool ParseTrotCli(int argc, const char **argv, TrotCliConfig *out, std::string *error_out);
void PrintTrotCliUsage();
void PrintTrotCliSummary(const TrotCliConfig &cfg);

}  // namespace go2_trot
