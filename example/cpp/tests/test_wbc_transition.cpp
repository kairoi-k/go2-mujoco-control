#include <cmath>
#include <iostream>

#include "wbc_transition.h"

namespace
{

bool Check(bool condition, const char *message)
{
    if (!condition)
        std::cerr << message << "\n";
    return condition;
}

}  // namespace

int main()
{
    bool passed = true;
    passed &= Check(!go2_control::WbcPlantStageAllowed(true, 0),
                    "stand-up must stay on the PD plant");
    passed &= Check(go2_control::WbcPlantStageAllowed(true, 1),
                    "settle must allow full WBC");
    passed &= Check(go2_control::WbcPlantStageAllowed(true, 2),
                    "gait must allow full WBC");
    passed &= Check(go2_control::WbcPlantStageAllowed(true, 3),
                    "return-to-stand must allow full WBC");
    passed &= Check(!go2_control::WbcPlantStageAllowed(true, 4),
                    "lie-down must return to PD");
    passed &= Check(!go2_control::WbcPlantStageAllowed(false, 1),
                    "incremental WBC must keep its gait-only gate");

    double blend = 0.0;
    blend = go2_control::WbcSlewUnitBlend(blend, true, 0.25, 1.0, 1.0);
    passed &= Check(std::abs(blend - 0.25) < 1.0e-12,
                    "rise slew is not bounded");
    blend = go2_control::WbcSlewUnitBlend(blend, false, 0.10, 1.0, 0.5);
    passed &= Check(std::abs(blend - 0.05) < 1.0e-12,
                    "fall slew is not bounded");
    passed &= Check(
        go2_control::WbcGaitReferenceBlend(0.0, 0.8) == 0.0 &&
        go2_control::WbcGaitReferenceBlend(0.8, 0.8) == 1.0,
        "gait reference blend endpoints are wrong");
    passed &= Check(
        go2_control::WbcContactScheduleBlend(0.2, 0.3, 0.3) == 0.0 &&
        go2_control::WbcContactScheduleBlend(0.6, 0.3, 0.3) == 1.0,
        "contact handoff endpoints are wrong");

    if (!passed)
        return 1;
    std::cout << "WBC transition checks passed\n";
    return 0;
}
