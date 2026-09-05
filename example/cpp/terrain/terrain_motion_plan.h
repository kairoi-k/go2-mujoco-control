#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "go2_forward_kinematics.h"
#include "terrain_control_interface.h"

namespace go2_terrain
{

constexpr std::size_t kTerrainPlanMaxKnots = kTerrainContactMaxKnots;

enum class TerrainPlanStatus : std::uint8_t
{
    kEmpty = 0,
    kValid,
    kDegraded,
    kStale,
    kRejected,
    kSafeStop,
};

inline const char *TerrainPlanStatusName(TerrainPlanStatus status)
{
    switch (status)
    {
    case TerrainPlanStatus::kValid: return "valid";
    case TerrainPlanStatus::kDegraded: return "degraded";
    case TerrainPlanStatus::kStale: return "stale";
    case TerrainPlanStatus::kRejected: return "rejected";
    case TerrainPlanStatus::kSafeStop: return "safe_stop";
    default: return "empty";
    }
}

enum class TerrainPlanFailure : std::uint8_t
{
    kNone = 0,
    kNoMap,
    kStaleMap,
    kUnknownTerrain,
    kNoSafeFoothold,
    kSupportInfeasible,
    kBodyInfeasible,
    kSolverFailure,
    kDeadlineMiss,
    kInvalidInput,
    kContactDisagreement,
};

struct TerrainBodyReference
{
    go2::Vec3 position{};
    go2::Vec3 linear_velocity{};
    go2::Vec3 linear_acceleration{};
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
    double yaw_rate_radps = 0.0;
    double height_m = 0.0;
    bool valid = false;
};

struct TerrainFootholdPrediction
{
    bool valid = false;
    bool touchdown = false;
    double touchdown_time_s = 0.0;
    double touchdown_phase = 0.0;
    go2::Vec3 position_world{};
    std::array<double, 3> surface_normal{0.0, 0.0, 1.0};
    std::uint32_t region_id = 0;
    double edge_margin_m = 0.0;
    double reachability_margin_m = 0.0;
    double swing_clearance_m = 0.0;
    double support_margin_m = 0.0;
    double collision_margin_m = 0.0;
    double uncertainty_m = 0.0;
    // Terrain-derived vertical swing profile. This is a trajectory input,
    // not a timing or topology change; the running-trot phase remains the
    // sole swing clock.
    double swing_lift_m = 0.0;
};

struct TerrainVelocityRequest
{
    bool valid = false;
    bool is_cap = true;
    double target_vx_mps = 0.0;
    double max_vx_mps = 0.0;
    double max_accel_mps2 = 0.8;
    double max_decel_mps2 = 1.2;
    double max_jerk_mps3 = 4.0;
    int priority = 0;
    std::uint64_t plan_id = 0;
    double valid_until_s = 0.0;
    std::string reason;
};

struct TerrainSolverDiagnostics
{
    bool attempted = false;
    bool success = false;
    bool deadline_miss = false;
    std::uint32_t iterations = 0;
    double elapsed_us = 0.0;
    double deadline_us = 5000.0;
    TerrainPlanFailure failure = TerrainPlanFailure::kNone;
};

struct TerrainMotionPlan
{
    std::uint64_t plan_id = 0;
    std::uint64_t plan_epoch = 0;
    std::uint64_t map_epoch = 0;
    double state_stamp_s = 0.0;
    double generated_at_s = 0.0;
    double valid_until_s = 0.0;
    std::string frame_id;
    TerrainPlanStatus status = TerrainPlanStatus::kEmpty;
    TerrainPlanFailure failure = TerrainPlanFailure::kNone;
    double map_age_s = 0.0;
    double uncertainty_m = 0.0;
    std::size_t horizon_knots = 0;
    std::size_t current_support_count = 0;
    double min_edge_margin_m = 0.0;
    double min_uncertainty_inflated_edge_margin_m = 0.0;
    double min_slope_rad = 0.0;
    double max_roughness_m = 0.0;
    double min_reachability_margin_m = 0.0;
    double min_swing_clearance_m = 0.0;
    double min_support_margin_m = 0.0;
    double min_uncertainty_inflated_support_margin_m = 0.0;
    std::size_t committed_touchdowns = 0;
    std::array<TerrainBodyReference, kTerrainPlanMaxKnots> body_reference{};
    TerrainContactSchedule contact_schedule{};
    std::array<std::array<TerrainFootholdPrediction, go2::kLegCount>,
               kTerrainPlanMaxKnots>
        predicted_foothold{};
    std::array<TerrainFootholdPrediction, go2::kLegCount>
        current_support_anchor{};
    TerrainVelocityRequest velocity_request{};
    TerrainSolverDiagnostics solver{};
    bool fallback_to_phase1 = true;
    bool safe_stop_requested = false;

    bool valid() const
    {
        if ((status != TerrainPlanStatus::kValid &&
             status != TerrainPlanStatus::kDegraded) ||
            plan_id == 0 || plan_epoch == 0 || map_epoch == 0 ||
            !contact_schedule.valid(horizon_knots) || frame_id.empty() ||
            !std::isfinite(state_stamp_s) || !std::isfinite(generated_at_s) ||
            !std::isfinite(valid_until_s) || valid_until_s < generated_at_s ||
            horizon_knots == 0 || horizon_knots > kTerrainPlanMaxKnots)
            return false;
        for (std::size_t k = 0; k < horizon_knots; ++k)
        {
            if (!body_reference[k].valid)
                return false;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                if (!contact_schedule.planned_contact[k][leg])
                    continue;
                const auto &foot = predicted_foothold[k][leg];
                if (!foot.valid || !std::isfinite(foot.touchdown_time_s) ||
                    !std::isfinite(foot.position_world.x) ||
                    !std::isfinite(foot.position_world.y) ||
                    !std::isfinite(foot.position_world.z) ||
                    !std::isfinite(foot.swing_lift_m) ||
                    foot.swing_lift_m < 0.0)
                    return false;
            }
        }
        return true;
    }

    bool usable_at(double now_s) const
    {
        return valid() && std::isfinite(now_s) && now_s <= valid_until_s;
    }
};

class TerrainPlanStore
{
public:
    void Publish(const TerrainMotionPlan &plan)
    {
        if (!plan.valid())
            return;
        auto next = std::make_shared<const TerrainMotionPlan>(plan);
        std::atomic_store_explicit(&latest_, std::move(next),
                                   std::memory_order_release);
    }

    std::shared_ptr<const TerrainMotionPlan> Load() const
    {
        return std::atomic_load_explicit(&latest_, std::memory_order_acquire);
    }

    std::shared_ptr<const TerrainMotionPlan> LoadUsable(double now_s) const
    {
        auto plan = Load();
        if (plan && plan->usable_at(now_s))
            return plan;
        return nullptr;
    }

private:
    std::shared_ptr<const TerrainMotionPlan> latest_;
};

} // namespace go2_terrain
