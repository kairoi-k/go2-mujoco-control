#include <cassert>
#include <string>

#include "wbc_runtime_gate.h"

int main()
{
    go2_control::WbcFeedforwardGateInput input;
    assert(
        go2_control::EvaluateWbcFeedforwardGate(input).code ==
        go2_control::WbcFeedforwardGateCode::kDisabled);

    input.requested = true;
    assert(
        go2_control::EvaluateWbcFeedforwardGate(input).code ==
        go2_control::WbcFeedforwardGateCode::kShadowDisabled);

    input.shadow_enabled = true;
    assert(
        go2_control::EvaluateWbcFeedforwardGate(input).code ==
        go2_control::WbcFeedforwardGateCode::kNotLocomotion);

    input.locomotion_active = true;
    assert(
        go2_control::EvaluateWbcFeedforwardGate(input).code ==
        go2_control::WbcFeedforwardGateCode::kShadowNotReady);

    input.solver_ok = true;
    input.mapping_ok = true;
    assert(
        go2_control::EvaluateWbcFeedforwardGate(input).code ==
        go2_control::WbcFeedforwardGateCode::kTaskUnsatisfied);

    input.wrench_satisfied = true;
    assert(
        go2_control::EvaluateWbcFeedforwardGate(input).code ==
        go2_control::WbcFeedforwardGateCode::kConstraintInfeasible);

    input.constraint_feasible = true;
    input.active_contacts = 2;
    assert(
        go2_control::EvaluateWbcFeedforwardGate(input).code ==
        go2_control::WbcFeedforwardGateCode::kOverBudget);

    input.within_budget = true;
    assert(
        go2_control::EvaluateWbcFeedforwardGate(input).code ==
        go2_control::WbcFeedforwardGateCode::kInvalidScale);

    input.torque_scale = 0.10;
    auto result = go2_control::EvaluateWbcFeedforwardGate(input);
    assert(result.ready);
    assert(
        result.code == go2_control::WbcFeedforwardGateCode::kReady);
    assert(
        std::string(
            go2_control::WbcFeedforwardGateReasonName(result.code)) ==
        "ready");

    input.candidate_values_finite = false;
    assert(
        go2_control::EvaluateWbcFeedforwardGate(input).code ==
        go2_control::WbcFeedforwardGateCode::kCandidateInvalid);
    return 0;
}
