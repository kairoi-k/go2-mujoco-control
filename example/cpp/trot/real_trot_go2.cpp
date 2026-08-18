// main only — implementation in trot_*.cpp; see docs/CODE_GUIDE.md
#include <csignal>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>

#include "trot_cli.h"
#include "trot_experiment.h"

using namespace unitree::robot;
using namespace go2_trot;

volatile std::sig_atomic_t g_signal_stop_requested = 0;

void HandleStopSignal(int)
{
    g_signal_stop_requested = 1;
}

int main(int argc, const char **argv)
{
    TrotCliConfig cfg;
    std::string error;
    if (!ParseTrotCli(argc, argv, &cfg, &error))
    {
        if (!error.empty())
            std::cerr << "Argument error: " << error << "\n";
        PrintTrotCliUsage();
        return 2;
    }

    ChannelFactory::Instance()->Init(cfg.domain_id, cfg.interface);
    std::signal(SIGINT, HandleStopSignal);
    std::signal(SIGTERM, HandleStopSignal);

    PrintTrotCliSummary(cfg);
    std::cin.get();

    TrotExperiment experiment(
        cfg.duration_s, cfg.csv_path, cfg.params, cfg.max_cycles,
        cfg.continuous_mode, cfg.stop_file_path, cfg.task_mode, cfg.goal);
    if (!experiment.Init())
        return 1;

    while (!experiment.Finished())
    {
        if (g_signal_stop_requested || experiment.StopFileRequested())
            experiment.RequestStop();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    experiment.Shutdown();
    std::cout.flush();
    std::cerr.flush();
    std::quick_exit(0);
}
