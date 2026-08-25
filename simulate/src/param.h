#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace param
{

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;
    std::filesystem::path ground_truth_log;

    int domain_id;
    std::string interface;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    // Run without a GUI window (no GLFW), used for headless experiments.
    bool headless = false;

    // Track the Go2 base body in the GUI camera for long-run recordings.
    bool camera_follow = false;

    // Publish simulated terrain sensing only for terrain-enabled runs.  The
    // default keeps the accepted Phase 1 simulator path unchanged.
    bool terrain_lidar = false;
    double initial_x_m = 0.0;
    double initial_y_m = 0.0;

    // Disturbance push on the base link (disabled when push_time_s < 0).
    double push_time_s = -1.0;
    double push_force_x_n = 0.0;
    double push_torque_pitch_nm = 0.0;
    double push_vel_x_mps = 0.0;
    double payload_kg = 0.0;
    double push_duration_s = 0.2;
    double friction_time_s = -1.0;
    double friction_mu = 0.20;
    double friction_duration_s = 1.0;

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            ground_truth_log.clear();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
        ("ground-truth-log", po::value<std::filesystem::path>(&config.ground_truth_log), "MuJoCo contact-force CSV output path")
        ("headless", po::bool_switch(&config.headless), "Run without a GUI window (no GLFW)")
        ("camera-follow", po::bool_switch(&config.camera_follow), "Track the Go2 base body with the GUI camera")
        ("terrain-lidar", po::bool_switch(&config.terrain_lidar), "Publish simulated terrain sensor maps")
        ("initial-x", po::value<double>(&config.initial_x_m), "Initial base world x (harness only)")
        ("initial-y", po::value<double>(&config.initial_y_m), "Initial base world y (harness only)")
        ("push-time", po::value<double>(&config.push_time_s), "Disturbance push start time (s); <0 disables")
        ("push-force-x", po::value<double>(&config.push_force_x_n), "Disturbance push force along world x (N)")
        ("push-torque-pitch", po::value<double>(&config.push_torque_pitch_nm), "Disturbance pitch torque (Nm)")
        ("friction-time", po::value<double>(&config.friction_time_s), "Ground friction change start time (s); <0 disables")
        ("friction-mu", po::value<double>(&config.friction_mu), "Ground friction coefficient during the event")
        ("friction-duration", po::value<double>(&config.friction_duration_s), "Ground friction event duration (s)")
        ("push-vel-x", po::value<double>(&config.push_vel_x_mps), "Disturbance base velocity kick along world x (m/s)")
        ("payload-kg", po::value<double>(&config.payload_kg), "Additional torso payload mass (kg)")
        ("push-duration", po::value<double>(&config.push_duration_s), "Disturbance push duration (s)")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}
