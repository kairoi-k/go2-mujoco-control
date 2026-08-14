// Record LowCmd/LowState tracking CSV for analysis.
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <unitree/common/thread/thread.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = 2.146E+9;
constexpr double VelStopF = 16000.0;
constexpr int kMotorCount = 12;
constexpr double kStandUpDuration = 3.0;

const std::array<const char *, kMotorCount> kMotorNames = {
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf"};

class TrackingExperiment
{
public:
    explicit TrackingExperiment(double duration_s, const std::string &csv_path)
        : duration_s_(duration_s), csv_path_(csv_path) {}

    bool Init();
    bool Finished() const { return finished_.load(); }

private:
    void InitLowCmd();
    void WriteCsvHeader();
    bool WaitForLowState(double timeout_s);
    void LowStateMessageHandler(const void *message);
    void LowCmdWrite();
    void LogSample();

private:
    const std::array<double, kMotorCount> stand_up_joint_pos_ = {
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763};

    std::array<double, kMotorCount> start_joint_pos_{};

    const double dt_ = 0.002;
    const double duration_s_;
    const std::string csv_path_;

    double running_time_ = 0.0;
    double phase_ = 0.0;

    unitree_go::msg::dds_::LowCmd_ low_cmd_{};
    unitree_go::msg::dds_::LowState_ low_state_{};
    bool have_low_state_ = false;
    bool have_start_joint_pos_ = false;

    std::mutex state_mutex_;
    std::ofstream csv_;
    std::atomic<bool> finished_{false};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber_;
    ThreadPtr low_cmd_write_thread_;
};

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

double smoothstep(double x)
{
    if (x <= 0.0)
        return 0.0;
    if (x >= 1.0)
        return 1.0;
    return x * x * (3 - 2 * x);
}

bool TrackingExperiment::Init()
{
    csv_.open(csv_path_);
    if (!csv_)
    {
        std::cerr << "Failed to open CSV file: " << csv_path_ << std::endl;
        return false;
    }
    csv_ << std::fixed << std::setprecision(9);
    WriteCsvHeader();

    InitLowCmd();

    lowcmd_publisher_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher_->InitChannel();

    lowstate_subscriber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber_->InitChannel(
        std::bind(&TrackingExperiment::LowStateMessageHandler, this, std::placeholders::_1), 1);

    if (!WaitForLowState(3.0))
    {
        std::cerr << "No LowState received. Is the Go2 simulator running on the same DDS interface/domain?"
                  << std::endl;
        return false;
    }

    low_cmd_write_thread_ = CreateRecurrentThreadEx(
        "tracklowcmd", UT_CPU_ID_NONE, int(dt_ * 1000000), &TrackingExperiment::LowCmdWrite, this);

    return true;
}

void TrackingExperiment::WriteCsvHeader()
{
    csv_ << "cmd_time_s,state_tick_s,has_state";
    for (int i = 0; i < kMotorCount; i++)
    {
        csv_ << "," << kMotorNames[i] << "_q_target"
             << "," << kMotorNames[i] << "_dq_target"
             << "," << kMotorNames[i] << "_kp"
             << "," << kMotorNames[i] << "_kd"
             << "," << kMotorNames[i] << "_tau_ff"
             << "," << kMotorNames[i] << "_q_state"
             << "," << kMotorNames[i] << "_dq_state"
             << "," << kMotorNames[i] << "_tau_est"
             << "," << kMotorNames[i] << "_q_error";
    }
    csv_ << "\n";
}

void TrackingExperiment::InitLowCmd()
{
    low_cmd_.head()[0] = 0xFE;
    low_cmd_.head()[1] = 0xEF;
    low_cmd_.level_flag() = 0xFF;
    low_cmd_.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        low_cmd_.motor_cmd()[i].mode() = 0x01;
        low_cmd_.motor_cmd()[i].q() = PosStopF;
        low_cmd_.motor_cmd()[i].kp() = 0;
        low_cmd_.motor_cmd()[i].dq() = VelStopF;
        low_cmd_.motor_cmd()[i].kd() = 0;
        low_cmd_.motor_cmd()[i].tau() = 0;
    }
}

bool TrackingExperiment::WaitForLowState(double timeout_s)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline)
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (have_low_state_)
            {
                for (int i = 0; i < kMotorCount; i++)
                {
                    start_joint_pos_[i] = low_state_.motor_state()[i].q();
                }
                have_start_joint_pos_ = true;
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

void TrackingExperiment::LowStateMessageHandler(const void *message)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    low_state_ = *(unitree_go::msg::dds_::LowState_ *)message;
    have_low_state_ = true;
}

void TrackingExperiment::LowCmdWrite()
{
    if (finished_.load())
    {
        return;
    }
    if (!have_start_joint_pos_)
    {
        return;
    }

    running_time_ += dt_;
    if (running_time_ < 3.0)
    {
        phase_ = smoothstep(running_time_ / kStandUpDuration);
        for (int i = 0; i < kMotorCount; i++)
        {
            low_cmd_.motor_cmd()[i].q() = phase_ * stand_up_joint_pos_[i] + (1 - phase_) * start_joint_pos_[i];
            low_cmd_.motor_cmd()[i].dq() = 0;
            low_cmd_.motor_cmd()[i].kp() = phase_ * 100.0 + (1 - phase_) * 20.0;
            low_cmd_.motor_cmd()[i].kd() = 3.5;
            low_cmd_.motor_cmd()[i].tau() = 0;
        }
    }
    else
    {
        for (int i = 0; i < kMotorCount; i++)
        {
            low_cmd_.motor_cmd()[i].q() = stand_up_joint_pos_[i];
            low_cmd_.motor_cmd()[i].dq() = 0;
            low_cmd_.motor_cmd()[i].kp() = 100;
            low_cmd_.motor_cmd()[i].kd() = 3.5;
            low_cmd_.motor_cmd()[i].tau() = 0;
        }
    }

    low_cmd_.crc() = crc32_core((uint32_t *)&low_cmd_, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher_->Write(low_cmd_);
    LogSample();

    if (running_time_ >= duration_s_)
    {
        finished_.store(true);
        csv_.flush();
        std::cout << "Finished. CSV saved to " << csv_path_ << std::endl;
    }
}

void TrackingExperiment::LogSample()
{
    unitree_go::msg::dds_::LowState_ state_snapshot{};
    bool have_state = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_snapshot = low_state_;
        have_state = have_low_state_;
    }

    const double state_tick_s = have_state ? state_snapshot.tick() * 0.001 : 0.0;
    csv_ << running_time_ << "," << state_tick_s << "," << (have_state ? 1 : 0);

    for (int i = 0; i < kMotorCount; i++)
    {
        const double q_target = low_cmd_.motor_cmd()[i].q();
        const double dq_target = low_cmd_.motor_cmd()[i].dq();
        const double kp = low_cmd_.motor_cmd()[i].kp();
        const double kd = low_cmd_.motor_cmd()[i].kd();
        const double tau_ff = low_cmd_.motor_cmd()[i].tau();
        const double q_state = have_state ? state_snapshot.motor_state()[i].q() : 0.0;
        const double dq_state = have_state ? state_snapshot.motor_state()[i].dq() : 0.0;
        const double tau_est = have_state ? state_snapshot.motor_state()[i].tau_est() : 0.0;
        const double q_error = have_state ? q_target - q_state : 0.0;

        csv_ << "," << q_target
             << "," << dq_target
             << "," << kp
             << "," << kd
             << "," << tau_ff
             << "," << q_state
             << "," << dq_state
             << "," << tau_est
             << "," << q_error;
    }
    csv_ << "\n";
}

int main(int argc, const char **argv)
{
    std::string interface = "lo";
    double duration_s = 10.0;
    std::string csv_path = "go2_lowcmd_lowstate_tracking.csv";

    if (argc >= 2)
    {
        interface = argv[1];
    }
    if (argc >= 3)
    {
        duration_s = std::stod(argv[2]);
    }
    if (argc >= 4)
    {
        csv_path = argv[3];
    }

    ChannelFactory::Instance()->Init(1, interface);

    std::cout << "Interface: " << interface << "\n"
              << "Duration: " << duration_s << " s\n"
              << "CSV: " << csv_path << "\n"
              << "Press enter to start";
    std::cin.get();

    TrackingExperiment experiment(duration_s, csv_path);
    if (!experiment.Init())
    {
        return 1;
    }

    while (!experiment.Finished())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
