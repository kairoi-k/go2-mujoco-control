// WBC feedforward gate codes and runtime allow/deny helpers.
#pragma once

#include <cmath>

namespace go2_control
{

enum class WbcFeedforwardGateCode : int
{
    kDisabled = 0,
    kShadowDisabled = 1,
    kNotLocomotion = 2,
    kShadowNotReady = 3,
    kTaskUnsatisfied = 4,
    kConstraintInfeasible = 5,
    kInsufficientContacts = 6,
    kOverBudget = 7,
    kInvalidScale = 8,
    kCandidateInvalid = 9,
    kReady = 10,
};

inline const char *WbcFeedforwardGateReasonName(
    WbcFeedforwardGateCode code)
{
    switch (code)
    {
    case WbcFeedforwardGateCode::kDisabled:
        return "disabled";
    case WbcFeedforwardGateCode::kShadowDisabled:
        return "shadow_disabled";
    case WbcFeedforwardGateCode::kNotLocomotion:
        return "not_locomotion";
    case WbcFeedforwardGateCode::kShadowNotReady:
        return "shadow_not_ready";
    case WbcFeedforwardGateCode::kTaskUnsatisfied:
        return "task_unsatisfied";
    case WbcFeedforwardGateCode::kConstraintInfeasible:
        return "constraint_infeasible";
    case WbcFeedforwardGateCode::kInsufficientContacts:
        return "insufficient_contacts";
    case WbcFeedforwardGateCode::kOverBudget:
        return "over_budget";
    case WbcFeedforwardGateCode::kInvalidScale:
        return "invalid_scale";
    case WbcFeedforwardGateCode::kCandidateInvalid:
        return "candidate_invalid";
    case WbcFeedforwardGateCode::kReady:
        return "ready";
    }
    return "unknown";
}

struct WbcFeedforwardGateInput
{
    bool requested = false;
    bool shadow_enabled = false;
    bool locomotion_active = false;
    bool solver_ok = false;
    bool mapping_ok = false;
    bool wrench_satisfied = false;
    bool reduced_task_gate = false;
    bool constraint_feasible = false;
    int active_contacts = 0;
    int minimum_contacts = 2;
    bool within_budget = false;
    double torque_scale = 0.0;
    double max_torque_scale = 0.25;
    bool candidate_values_finite = true;
    bool scaled_candidate_within_limit = true;
};

struct WbcFeedforwardGateResult
{
    WbcFeedforwardGateCode code = WbcFeedforwardGateCode::kDisabled;
    bool ready = false;
};

inline WbcFeedforwardGateResult EvaluateWbcFeedforwardGate(
    const WbcFeedforwardGateInput &input)
{
    const auto reject = [](WbcFeedforwardGateCode code) {
        return WbcFeedforwardGateResult{code, false};
    };
    if (!input.requested)
        return reject(WbcFeedforwardGateCode::kDisabled);
    if (!input.shadow_enabled)
        return reject(WbcFeedforwardGateCode::kShadowDisabled);
    if (!input.locomotion_active)
        return reject(WbcFeedforwardGateCode::kNotLocomotion);
    if (!input.solver_ok || !input.mapping_ok)
        return reject(WbcFeedforwardGateCode::kShadowNotReady);
    if (!input.wrench_satisfied && !input.reduced_task_gate)
        return reject(WbcFeedforwardGateCode::kTaskUnsatisfied);
    if (!input.constraint_feasible)
        return reject(WbcFeedforwardGateCode::kConstraintInfeasible);
    if (input.active_contacts < input.minimum_contacts)
        return reject(WbcFeedforwardGateCode::kInsufficientContacts);
    if (!input.within_budget)
        return reject(WbcFeedforwardGateCode::kOverBudget);
    if (!std::isfinite(input.torque_scale) ||
        input.torque_scale <= 0.0 ||
        !std::isfinite(input.max_torque_scale) ||
        input.torque_scale > input.max_torque_scale)
    {
        return reject(WbcFeedforwardGateCode::kInvalidScale);
    }
    if (!input.candidate_values_finite ||
        !input.scaled_candidate_within_limit)
    {
        return reject(WbcFeedforwardGateCode::kCandidateInvalid);
    }
    return {WbcFeedforwardGateCode::kReady, true};
}

} // namespace go2_control
