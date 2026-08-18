// main only — parse CLI then run TrackingExperiment; see docs/CODE_GUIDE.md
#include <chrono>
#include <iostream>
#include <thread>
#include <utility>

#include <unitree/robot/channel/channel_factory.hpp>

#include "leg_lift_cli.h"
#include "leg_lift_experiment.h"

using namespace unitree::robot;
using namespace go2_leg;

int main(int argc, const char **argv)
{
    LegLiftCliConfig cfg;
    if (!ParseLegLiftCli(argc, argv, &cfg))
        return 2;

    PrintFootPositions(
        "Reference stand target",
        {0.00571868, 0.608813, -1.21763,
         -0.00571868, 0.608813, -1.21763,
         0.00571868, 0.608813, -1.21763,
         -0.00571868, 0.608813, -1.21763});

    ChannelFactory::Instance()->Init(1, cfg.interface);
    PrintLegLiftCliSummary(cfg);
    std::cin.get();

    TrackingExperiment experiment(
        cfg.duration_s, cfg.csv_path, std::move(cfg.steps), cfg.sequence_mode,
        cfg.repeat_sequence, cfg.max_completed_steps,
        cfg.world_feedback_enabled, cfg.yaw_feedback_enabled, cfg.tempo_scale,
        cfg.support_scale, cfg.adaptive_tempo);
    if (!experiment.Init())
        return 1;

    while (!experiment.Finished())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    experiment.Shutdown();
    std::cout.flush();
    std::cerr.flush();
    std::quick_exit(0);
}
