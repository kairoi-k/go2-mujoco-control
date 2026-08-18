#pragma once
// Decode base mass matrix / bias packed into LowState motor spare slots.

#include <array>
#include <cmath>

#include <unitree/idl/go2/LowState_.hpp>

namespace go2_trot {

struct TrueDynamics
{
    bool valid = false;
    std::array<double, 36> base_mass_matrix{};
    std::array<double, 6> base_qfrc_bias{};
};

inline TrueDynamics ExtractTrueDynamics(
    const unitree_go::msg::dds_::LowState_ &state)
{
    TrueDynamics dyn;
    for (int slot = 0; slot < 42; ++slot)
    {
        const int motor = 12 + slot / 7;
        const int field = slot % 7;
        const auto &ms = state.motor_state()[motor];
        double value = 0.0;
        switch (field)
        {
            case 0: value = ms.q(); break;
            case 1: value = ms.dq(); break;
            case 2: value = ms.ddq(); break;
            case 3: value = ms.tau_est(); break;
            case 4: value = ms.q_raw(); break;
            case 5: value = ms.dq_raw(); break;
            case 6: value = ms.ddq_raw(); break;
        }
        if (!std::isfinite(value))
            return dyn;
        if (slot < 36)
            dyn.base_mass_matrix[slot] = value;
        else
            dyn.base_qfrc_bias[slot - 36] = value;
    }
    dyn.valid = true;
    return dyn;
}


}  // namespace go2_trot
