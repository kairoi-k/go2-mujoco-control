#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "terrain_model.h"
#include "go2_forward_kinematics.h"

namespace go2_terrain
{

enum class TerrainCrawlLegOrder : unsigned char
{
    kLegacyFrontFirst = 0,
    kLateral = 1,
};

// Body advance is optional so the proven legacy sequence remains the default.
// The early variant moves in four-contact stance after FL commits, before FR
// asks the mixed-height triangle to carry the COM shift.
enum class TerrainCrawlAdvancePolicy : unsigned char
{
    kAfterSecondStep = 0,
    kBeforeSecondStep = 1,
};

inline constexpr std::array<std::size_t, go2::kLegCount>
    kLegacyFrontFirstLegOrder = {1, 0, 2, 3};
inline constexpr std::array<std::size_t, go2::kLegCount>
    kLateralLegOrder = {1, 3, 0, 2};

inline constexpr const char *TerrainCrawlLegOrderName(
    TerrainCrawlLegOrder order) noexcept
{
    return order == TerrainCrawlLegOrder::kLateral ? "lateral" : "legacy";
}

// The script fixes sequencing and deadlines only. Terrain geometry remains
// an observation: this helper never contains a scene/world coordinate.

struct TerrainStagingReference
{
    bool valid = false;
    double edge_x_m = std::numeric_limits<double>::quiet_NaN();
    double target_world_x_m = std::numeric_limits<double>::quiet_NaN();
    // Full world-frame target retained for non-zero-yaw staging consumers.
    go2::Vec3 target_world{};
    double error_m = std::numeric_limits<double>::quiet_NaN();
};

// Estimate the first observed rising edge in the lidar map and express the
// canonical body target relative to that edge. Map cells are base-frame
// observations; rotating the edge into world coordinates makes the reference
// independent of the asynchronous map/detection timestamp.
inline double MeasureTerrainEdgeX(const TerrainModel &terrain) noexcept
{
    double edge_x = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t iy = 0; iy < terrain.height; ++iy)
        for (std::size_t ix = 1; ix < terrain.width; ++ix)
        {
            const auto *before = terrain.CellAt(ix - 1, iy);
            const auto *after = terrain.CellAt(ix, iy);
            if (before == nullptr || after == nullptr || !before->known ||
                !after->known || after->height_m <= before->height_m + 0.02)
                continue;
            const double x = terrain.origin_m[0] +
                static_cast<double>(ix) * terrain.resolution_m;
            edge_x = std::isfinite(edge_x) ? std::min(edge_x, x) : x;
        }
    return edge_x;
}

inline TerrainStagingReference MeasureTerrainStagingReferenceWithOffset(
    const TerrainModel &terrain,
    const go2::Vec3 &base_position_world,
    double base_yaw_rad,
    double forward_offset_m)
{
    TerrainStagingReference result;
    if (!terrain.valid() || !std::isfinite(base_position_world.x) ||
        !std::isfinite(base_position_world.y) ||
        !std::isfinite(base_yaw_rad) || !std::isfinite(forward_offset_m))
        return result;
    const double edge_x = MeasureTerrainEdgeX(terrain);
    if (!std::isfinite(edge_x))
        return result;
    const double c = std::cos(base_yaw_rad);
    const double s = std::sin(base_yaw_rad);
    // The map edge is local to the measured base pose. Transform both the
    // observed edge and the desired base point into world coordinates before
    // deriving the servo error; never compare a local map x with world x.
    const go2::Vec3 edge_world{
        base_position_world.x + c * edge_x,
        base_position_world.y + s * edge_x,
        base_position_world.z};
    result.target_world = {
        edge_world.x - c * forward_offset_m,
        edge_world.y - s * forward_offset_m,
        base_position_world.z};
    const double forward_error =
        c * (result.target_world.x - base_position_world.x) +
        s * (result.target_world.y - base_position_world.y);
    result.valid = std::isfinite(result.target_world.x) &&
        std::isfinite(result.target_world.y);
    result.edge_x_m = edge_x;
    result.target_world_x_m = result.target_world.x;
    // error_m is signed distance along the measured forward axis, rather
    // than a world-X difference that changes meaning when yaw is nonzero.
    result.error_m = forward_error;
    result.valid = result.valid && std::isfinite(result.target_world_x_m) &&
        std::isfinite(result.error_m);
    return result;
}

inline TerrainStagingReference MeasureTerrainStagingReference(
    const TerrainModel &terrain,
    const go2::Vec3 &base_position_world,
    double base_yaw_rad,
    double nominal_front_foot_x_m,
    double standoff_m = 0.25)
{
    if (!std::isfinite(nominal_front_foot_x_m) ||
        !std::isfinite(standoff_m) || standoff_m < 0.0)
        return {};
    return MeasureTerrainStagingReferenceWithOffset(
        terrain, base_position_world, base_yaw_rad,
        standoff_m + nominal_front_foot_x_m);
}

// Order-060's entry basin is an observed body pose band relative to the
// measured edge, not a nominal foot standoff. Keep this helper separate from
// the legacy staging reference so planner/v1 callers retain their arithmetic.
inline constexpr double kMeasuredBasinEdgeMinusBaseMinM = 0.318;
inline constexpr double kMeasuredBasinEdgeMinusBaseMaxM = 0.330;
inline constexpr double kMeasuredBasinEdgeMinusBaseTargetM =
    0.5 * (kMeasuredBasinEdgeMinusBaseMinM +
           kMeasuredBasinEdgeMinusBaseMaxM);

inline TerrainStagingReference MeasureTerrainBasinStagingReference(
    const TerrainModel &terrain,
    const go2::Vec3 &base_position_world,
    double base_yaw_rad)
{
    return MeasureTerrainStagingReferenceWithOffset(
        terrain, base_position_world, base_yaw_rad,
        kMeasuredBasinEdgeMinusBaseTargetM);
}

struct TerrainScriptTarget
{
    bool valid = false;
    go2::Vec3 position_base{};
    double edge_margin_m = 0.0;
    double height_m = std::numeric_limits<double>::quiet_NaN();
};

inline TerrainScriptTarget MeasureTerrainScriptTarget(
    const TerrainModel &terrain, go2::Leg leg,
    const go2::Vec3 &current_foot_base,
    double stand_off_m = 0.080, double patch_radius_m = 0.025)
{
    TerrainScriptTarget result;
    if (!terrain.valid() || !std::isfinite(current_foot_base.x) ||
        !std::isfinite(current_foot_base.y) || !std::isfinite(current_foot_base.z) ||
        !std::isfinite(stand_off_m) || stand_off_m < 0.0 ||
        !std::isfinite(patch_radius_m) || patch_radius_m < 0.0)
        return result;

    // Scan map cells in stable order and rank by forward progress, then
    // lateral displacement, then cell index. This is not the planner stream.
    const double step = terrain.resolution_m;
    // Reject lateral risers before taking the minimum edge. Otherwise a
    // side obstacle can become the apparent forward foothold.
    constexpr double kForwardCorridorHalfWidthM = 0.10;
    // The measured foot can be behind the nominal body anchor after terrain
    // braking. Keep the bounded direct target search reachable without
    // changing its edge stand-off or candidate ordering.
    constexpr double kMaximumProgressM = 0.65;
    // Edge estimation may use a wider lateral consensus than foothold
    // selection: the raised platform is broad, while a side obstacle must
    // not win unless it appears across multiple rows.
    constexpr double kEdgeConsensusHalfWidthM = 0.30;
    const double edge_x = [&]() {
        double edge = std::numeric_limits<double>::quiet_NaN();
        constexpr std::size_t kEdgeConsensusRows = 2;
        for (std::size_t ix = 1; ix + 1 < terrain.width; ++ix)
        {
            std::size_t transition_rows = 0;
            for (std::size_t iy = 0; iy < terrain.height; ++iy)
            {
                const double y = terrain.origin_m[1] +
                    (static_cast<double>(iy) + 0.5) * step;
                if (std::abs(y - current_foot_base.y) >
                        kForwardCorridorHalfWidthM)
                    continue;
                const auto *before = terrain.CellAt(ix - 1, iy);
                const auto *after = terrain.CellAt(ix, iy);
                const auto *following = terrain.CellAt(ix + 1, iy);
                // A single blended/quantized cell or isolated ray cannot
                // define an edge. Require a persistent elevated run with
                // lateral consensus inside the forward corridor.
                if (before != nullptr && after != nullptr &&
                    following != nullptr && before->known && after->known &&
                    following->known &&
                    after->height_m > before->height_m + 0.02 &&
                    following->height_m >= after->height_m - 0.01)
                    ++transition_rows;
            }
            if (transition_rows < kEdgeConsensusRows)
                continue;
            const double x = terrain.origin_m[0] +
                static_cast<double>(ix) * step;
            edge = std::isfinite(edge) ? std::min(edge, x) : x;
        }
        return edge;
    }();
    if (!std::isfinite(edge_x))
        return result;

    bool have_best = false;
    double best_progress = std::numeric_limits<double>::infinity();
    double best_lateral = std::numeric_limits<double>::infinity();
    std::size_t best_ix = 0;
    std::size_t best_iy = 0;
    TerrainPatch best_patch{};
    for (std::size_t iy = 0; iy < terrain.height; ++iy)
    {
        for (std::size_t ix = 0; ix < terrain.width; ++ix)
        {
            const double x = terrain.origin_m[0] +
                (static_cast<double>(ix) + 0.5) * step;
            const double y = terrain.origin_m[1] +
                (static_cast<double>(iy) + 0.5) * step;
            const double progress = x - current_foot_base.x;
            if (progress < stand_off_m || progress > kMaximumProgressM ||
                std::abs(y - current_foot_base.y) >
                    kForwardCorridorHalfWidthM ||
                x < edge_x + stand_off_m)
                continue;
            TerrainPatch patch;
            if (!terrain.SamplePatch(x, y, patch_radius_m, patch) ||
                !patch.all_known || patch.HasUnknownInside() ||
                patch.map_edge_margin_m < patch_radius_m ||
                !std::isfinite(patch.center_height_m) ||
                patch.center_height_m <=
                    go2::FootSiteToContactPatch(current_foot_base).z +
                        0.02)
                continue;
            const double lateral = std::abs(y - current_foot_base.y);
            const bool better = !have_best || progress < best_progress ||
                (progress == best_progress && lateral < best_lateral) ||
                (progress == best_progress && lateral == best_lateral &&
                 (iy < best_iy || (iy == best_iy && ix < best_ix)));
            if (!better)
                continue;
            have_best = true;
            best_progress = progress;
            best_lateral = lateral;
            best_ix = ix;
            best_iy = iy;
            best_patch = patch;
        }
    }
    if (!have_best)
        return result;
    (void)leg;
    result.valid = true;
    result.position_base = {
        terrain.origin_m[0] + (static_cast<double>(best_ix) + 0.5) * step,
        terrain.origin_m[1] + (static_cast<double>(best_iy) + 0.5) * step,
        best_patch.center_height_m};
    result.edge_margin_m = best_patch.map_edge_margin_m;
    result.height_m = best_patch.center_height_m;
    return result;
}

struct TerrainCrawlScriptSignals
{
    bool transfer_window_active = false;
    bool support_valid = false;
    int support_contacts = 0;
    bool target_valid = false;
    bool measured_contact = false;
    bool endpoint_within_tolerance = false;
    bool rear_targets_fk_reachable = false;
    bool base_clear = false;
    bool all_feet_clear = false;
    bool stable = false;
    bool measured_force_valid = false;
    std::array<double, go2::kLegCount> measured_normal_force_n{};
    double now_s = 0.0;
};

enum class TerrainCrawlScriptStage : unsigned char
{
    kInactive = 0, kShiftCom, kSwing, kEndpointHold, kAdvanceBody,
    kClear, kResume, kAbort
};

class TerrainCrawlScript
{
public:
    // Retain the historical compile-time alias for callers that only need
    // the old order; live instances select the order explicitly.
    static constexpr std::array<std::size_t, go2::kLegCount> kLegOrder =
        kLegacyFrontFirstLegOrder;
    static constexpr std::array<std::size_t, go2::kLegCount> kLateralOrder =
        kLateralLegOrder;
    static constexpr double kShiftRampS = 0.40;
    static constexpr double kShiftSettleS = 0.20;
    static constexpr double kSwingDurationS = 0.60;
    static constexpr double kSwingApexPhase = 0.40;
    static constexpr int kMaxRetries = 2;
    static constexpr double kStableS = 0.45;

    void SetLegOrder(TerrainCrawlLegOrder order) noexcept
    {
        leg_order_ = order;
        leg_order_values_ = order == TerrainCrawlLegOrder::kLateral
            ? kLateralLegOrder : kLegacyFrontFirstLegOrder;
    }

    TerrainCrawlLegOrder leg_order() const noexcept { return leg_order_; }

    void Reset() noexcept
    {
        stage_ = TerrainCrawlScriptStage::kInactive;
        order_index_ = 0;
        retry_count_ = 0;
        state_enter_s_ = 0.0;
        stable_start_s_ = 0.0;
    }

    void Start(double now_s) noexcept
    {
        Reset();
        stage_ = TerrainCrawlScriptStage::kShiftCom;
        state_enter_s_ = now_s;
    }

    TerrainCrawlScriptStage Update(const TerrainCrawlScriptSignals &s) noexcept
    {
        if (!s.transfer_window_active)
        {
            Reset();
            return stage_;
        }
        if (stage_ == TerrainCrawlScriptStage::kInactive)
            Start(s.now_s);
        if (stage_ == TerrainCrawlScriptStage::kAbort)
            return stage_;
        const double elapsed = s.now_s - state_enter_s_;
        if (!std::isfinite(elapsed) || elapsed < 0.0)
        {
            Set(TerrainCrawlScriptStage::kAbort, s.now_s);
            return stage_;
        }
        switch (stage_)
        {
        case TerrainCrawlScriptStage::kShiftCom:
            if (s.support_valid && s.support_contacts >= 3 &&
                s.target_valid && elapsed + 1.0e-9 >= kShiftRampS + kShiftSettleS)
                Set(TerrainCrawlScriptStage::kSwing, s.now_s);
            break;
        case TerrainCrawlScriptStage::kSwing:
            if (s.measured_contact && s.endpoint_within_tolerance &&
                ForceSupported(active_leg(), s))
                SetNext(s.now_s);
            else if (elapsed + 1.0e-9 >= kSwingDurationS)
                Set(TerrainCrawlScriptStage::kEndpointHold, s.now_s);
            break;
        case TerrainCrawlScriptStage::kEndpointHold:
            if (s.measured_contact && s.endpoint_within_tolerance &&
                ForceSupported(active_leg(), s))
                SetNext(s.now_s);
            else if (elapsed >= 0.20)
            {
                if (retry_count_ < kMaxRetries)
                {
                    ++retry_count_;
                    Set(TerrainCrawlScriptStage::kShiftCom, s.now_s);
                }
                else
                    Set(TerrainCrawlScriptStage::kAbort, s.now_s);
            }
            break;
        case TerrainCrawlScriptStage::kAdvanceBody:
            if (s.rear_targets_fk_reachable &&
                ForceSupported(go2::kLegCount, s))
                Set(TerrainCrawlScriptStage::kShiftCom, s.now_s);
            break;
        case TerrainCrawlScriptStage::kClear:
            if (s.base_clear && s.all_feet_clear)
            {
                stable_start_s_ = s.now_s;
                Set(TerrainCrawlScriptStage::kResume, s.now_s);
            }
            break;
        case TerrainCrawlScriptStage::kResume:
            if (s.stable && s.now_s - stable_start_s_ >= kStableS)
                Reset();
            break;
        default: break;
        }
        return stage_;
    }

    TerrainCrawlScriptStage stage() const noexcept { return stage_; }
    std::size_t active_leg() const noexcept
    {
        return order_index_ < leg_order_values_.size()
            ? leg_order_values_[order_index_] : go2::kLegCount;
    }
    std::size_t order_index() const noexcept { return order_index_; }
    int retry_count() const noexcept { return retry_count_; }
    double state_enter_time_s() const noexcept { return state_enter_s_; }
    static constexpr double SwingEndTime(double start_s) noexcept
    { return start_s + kSwingDurationS; }

private:
    bool ForceSupported(std::size_t lifted_leg,
                        const TerrainCrawlScriptSignals &s) const noexcept
    {
        if (!s.measured_force_valid)
            return false;
        double total = 0.0;
        double minimum = std::numeric_limits<double>::infinity();
        double maximum = 0.0;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (leg == lifted_leg)
                continue;
            const double force = s.measured_normal_force_n[leg];
            if (!std::isfinite(force) || force < 10.0)
                return false;
            total += force;
            minimum = std::min(minimum, force);
            maximum = std::max(maximum, force);
        }
        return total >= 50.0 && minimum > 0.0 && maximum / minimum <= 4.0;
    }

    void Set(TerrainCrawlScriptStage stage, double now_s) noexcept
    {
        stage_ = stage;
        state_enter_s_ = now_s;
    }
    void SetNext(double now_s) noexcept
    {
        retry_count_ = 0;
        if (order_index_ == 1)
            Set(TerrainCrawlScriptStage::kAdvanceBody, now_s);
        else if (order_index_ + 1 < leg_order_values_.size())
        {
            ++order_index_;
            Set(TerrainCrawlScriptStage::kShiftCom, now_s);
        }
        else
            Set(TerrainCrawlScriptStage::kClear, now_s);
    }

    TerrainCrawlScriptStage stage_ = TerrainCrawlScriptStage::kInactive;
    TerrainCrawlLegOrder leg_order_ = TerrainCrawlLegOrder::kLegacyFrontFirst;
    std::array<std::size_t, go2::kLegCount> leg_order_values_ =
        kLegacyFrontFirstLegOrder;
    std::size_t order_index_ = 0;
    int retry_count_ = 0;
    double state_enter_s_ = 0.0;
    double stable_start_s_ = 0.0;
};

} // namespace go2_terrain
