#include <array>
#include <iostream>
#include <string>

#include "trot_cli.h"

namespace {

bool Parse(const std::initializer_list<const char *> args,
           go2_trot::TrotCliConfig *config, std::string *error)
{
    std::array<const char *, 8> argv{};
    std::size_t argc = 0;
    for (const char *arg : args)
        argv[argc++] = arg;
    return go2_trot::ParseTrotCli(
        static_cast<int>(argc), argv.data(), config, error);
}

bool ExpectRetired(const std::initializer_list<const char *> args)
{
    go2_trot::TrotCliConfig config;
    std::string error;
    return !Parse(args, &config, &error) &&
        error.find("retired terrain actuation option") != std::string::npos;
}

}  // namespace

int main()
{
    go2_trot::TrotCliConfig sensor_only;
    std::string error;
    if (!Parse({"real_trot_go2", "lo", "1", "out.csv",
                "--terrain-sensor-only"}, &sensor_only, &error) ||
        !sensor_only.params.terrain_enabled ||
        !sensor_only.params.terrain_sensor_only)
    {
        std::cerr << "sensor-only route must remain available: " << error
                  << "\n";
        return 1;
    }

    if (!ExpectRetired({"real_trot_go2", "lo", "1", "out.csv",
                        "--stage-c-execution"}) ||
        !ExpectRetired({"real_trot_go2", "lo", "1", "out.csv",
                        "--terrain-planner"}) ||
        !ExpectRetired({"real_trot_go2", "lo", "1", "out.csv",
                        "--terrain-leg-order", "lateral"}) ||
        !ExpectRetired({"real_trot_go2", "lo", "1", "out.csv",
                        "--terrain-advance-body-before-second"}) ||
        !ExpectRetired({"real_trot_go2", "lo", "1", "out.csv",
                        "--gait-pattern", "crawl"}))
    {
        std::cerr << "a retired terrain route was accepted\n";
        return 1;
    }

    return 0;
}
