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

// The script fixes sequencing and deadlines only. Terrain geometry remains
// an observation: this helper never contains a scene/world coordinate.
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
    const double edge_x = [&]() {
        double edge = std::numeric_limits<double>::quiet_NaN();
        for (std::size_t iy = 0; iy < terrain.height; ++iy)
        {
            for (std::size_t ix = 1; ix < terrain.width; ++ix)
            {
                const auto *before = terrain.CellAt(ix - 1, iy);
                const auto *after = terrain.CellAt(ix, iy);
                if (before == nullptr || after == nullptr || !before->known ||
                    !after->known || after->height_m <= before->height_m + 0.02)
                    continue;
                const double x = terrain.origin_m[0] +
                    static_cast<double>(ix) * step;
                edge = std::isfinite(edge) ? std::min(edge, x) : x;
            }
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
            if (progress < stand_off_m || progress > 0.55 ||
                std::abs(y - current_foot_base.y) > 0.14 ||
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
    static constexpr std::array<std::size_t, go2::kLegCount> kLegOrder =
        {1, 0, 2, 3};
    static constexpr double kShiftRampS = 0.40;
    static constexpr double kShiftSettleS = 0.20;
    static constexpr double kSwingDurationS = 0.60;
    static constexpr double kSwingApexPhase = 0.40;
    static constexpr int kMaxRetries = 2;
    static constexpr double kStableS = 0.45;

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
            return stage_;
        switch (stage_)
        {
        case TerrainCrawlScriptStage::kShiftCom:
            if (s.support_valid && s.support_contacts >= 3 &&
                s.target_valid && elapsed + 1.0e-9 >= kShiftRampS + kShiftSettleS)
                Set(TerrainCrawlScriptStage::kSwing, s.now_s);
            break;
        case TerrainCrawlScriptStage::kSwing:
            if (s.measured_contact && s.endpoint_within_tolerance)
                SetNext(s.now_s);
            else if (elapsed + 1.0e-9 >= kSwingDurationS)
                Set(TerrainCrawlScriptStage::kEndpointHold, s.now_s);
            break;
        case TerrainCrawlScriptStage::kEndpointHold:
            if (s.measured_contact && s.endpoint_within_tolerance)
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
            if (s.rear_targets_fk_reachable)
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
        return order_index_ < kLegOrder.size() ? kLegOrder[order_index_]
                                                : go2::kLegCount;
    }
    std::size_t order_index() const noexcept { return order_index_; }
    int retry_count() const noexcept { return retry_count_; }
    double state_enter_time_s() const noexcept { return state_enter_s_; }
    static constexpr double SwingEndTime(double start_s) noexcept
    { return start_s + kSwingDurationS; }

private:
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
        else if (order_index_ + 1 < kLegOrder.size())
        {
            ++order_index_;
            Set(TerrainCrawlScriptStage::kShiftCom, now_s);
        }
        else
            Set(TerrainCrawlScriptStage::kClear, now_s);
    }

    TerrainCrawlScriptStage stage_ = TerrainCrawlScriptStage::kInactive;
    std::size_t order_index_ = 0;
    int retry_count_ = 0;
    double state_enter_s_ = 0.0;
    double stable_start_s_ = 0.0;
};

} // namespace go2_terrain
