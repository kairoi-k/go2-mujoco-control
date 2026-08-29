#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "go2_forward_kinematics.h"

namespace go2_terrain
{

enum class TerrainCrawlState : std::uint8_t
{
    kInactive = 0,
    kApproach,
    kDecelerateToCreep,
    kShiftCom,
    kCrawlStep,
    kAdvanceBody,
    kClear,
    kResume,
    kAbort,
};

inline const char *TerrainCrawlStateName(TerrainCrawlState state) noexcept
{
    switch (state)
    {
    case TerrainCrawlState::kApproach: return "APPROACH";
    case TerrainCrawlState::kDecelerateToCreep: return "DECELERATE_TO_CREEP";
    case TerrainCrawlState::kShiftCom: return "SHIFT_COM";
    case TerrainCrawlState::kCrawlStep: return "CRAWL_STEP";
    case TerrainCrawlState::kAdvanceBody: return "ADVANCE_BODY";
    case TerrainCrawlState::kClear: return "CLEAR";
    case TerrainCrawlState::kResume: return "RESUME";
    case TerrainCrawlState::kAbort: return "ABORT";
    default: return "INACTIVE";
    }
}

struct TerrainCrawlSignals
{
    bool transfer_window_active = false;
    bool plan_valid = false;
    bool measured_contact_valid = false;
    std::array<bool, go2::kLegCount> measured_contact{};
    double measured_velocity_mps = 0.0;
    std::array<bool, go2::kLegCount> target_valid{};
    std::array<bool, go2::kLegCount> committed{};
    bool measured_com_valid = false;
    go2::Vec3 measured_com_world{};
    bool measured_foot_valid = false;
    std::array<go2::Vec3, go2::kLegCount> measured_foot_world{};
    bool rear_targets_fk_reachable = false;
    bool base_clear = false;
    bool all_feet_clear = false;
    bool stable = false;
    bool step_failed = false;
    double now_s = 0.0;
};

inline int TerrainCrawlContactCount(
    const std::array<bool, go2::kLegCount> &contact) noexcept
{
    return static_cast<int>(std::count(contact.begin(), contact.end(), true));
}

struct TerrainSupportTriangle
{
    std::array<go2::Vec3, 3> vertex{};
    bool valid = false;
};

struct TerrainSupportTriangleMetrics
{
    bool valid = false;
    bool inside = false;
    double signed_margin_m = -std::numeric_limits<double>::infinity();
    std::array<double, 3> edge_margin_m{};
};

inline TerrainSupportTriangle ComputeTerrainSupportTriangle(
    const std::array<go2::Vec3, go2::kLegCount> &feet,
    std::size_t lifted_leg) noexcept
{
    TerrainSupportTriangle triangle;
    if (lifted_leg >= go2::kLegCount)
        return triangle;
    std::size_t out = 0;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (leg == lifted_leg)
            continue;
        triangle.vertex[out++] = feet[leg];
    }
    triangle.valid = true;
    for (const auto &v : triangle.vertex)
        triangle.valid = triangle.valid && std::isfinite(v.x) &&
            std::isfinite(v.y);
    const double twice_area = (triangle.vertex[1].x - triangle.vertex[0].x) *
            (triangle.vertex[2].y - triangle.vertex[0].y) -
        (triangle.vertex[1].y - triangle.vertex[0].y) *
            (triangle.vertex[2].x - triangle.vertex[0].x);
    triangle.valid = triangle.valid && std::abs(twice_area) > 1.0e-6;
    return triangle;
}

inline TerrainSupportTriangleMetrics MeasureTerrainSupportTriangle(
    const TerrainSupportTriangle &triangle, const go2::Vec3 &point) noexcept
{
    TerrainSupportTriangleMetrics metrics;
    if (!triangle.valid || !std::isfinite(point.x) || !std::isfinite(point.y))
        return metrics;
    auto vertex = triangle.vertex;
    const double area = (vertex[1].x - vertex[0].x) *
            (vertex[2].y - vertex[0].y) -
        (vertex[1].y - vertex[0].y) * (vertex[2].x - vertex[0].x);
    if (area < 0.0)
        std::swap(vertex[1], vertex[2]);
    metrics.valid = true;
    metrics.signed_margin_m = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < 3; ++i)
    {
        const auto &a = vertex[i];
        const auto &b = vertex[(i + 1) % 3];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double edge = (dx * (point.y - a.y) -
                             dy * (point.x - a.x)) / std::hypot(dx, dy);
        metrics.edge_margin_m[i] = edge;
        metrics.signed_margin_m = std::min(metrics.signed_margin_m, edge);
    }
    metrics.inside = metrics.signed_margin_m >= 0.0;
    return metrics;
}

inline go2::Vec3 TerrainSupportTriangleCentroid(
    const TerrainSupportTriangle &triangle) noexcept
{
    return {(triangle.vertex[0].x + triangle.vertex[1].x + triangle.vertex[2].x) / 3.0,
            (triangle.vertex[0].y + triangle.vertex[1].y + triangle.vertex[2].y) / 3.0,
            0.0};
}

class TerrainCrawlStateMachine
{
public:
    static constexpr std::array<std::size_t, go2::kLegCount> kLegOrder =
        {1, 0, 2, 3};
    static constexpr int kMaxRetries = 2;
    static constexpr double kComMarginM = 0.02;

    void Reset() noexcept
    {
        state_ = TerrainCrawlState::kInactive;
        order_index_ = 0;
        retry_count_ = 0;
        state_enter_time_s_ = 0.0;
        stable_start_time_s_ = 0.0;
        transition_count_ = 0;
        com_target_world_ = {};
        com_margin_m_ = -std::numeric_limits<double>::infinity();
        triangle_valid_ = false;
    }

    void Enter(double now_s) noexcept
    {
        state_ = TerrainCrawlState::kApproach;
        order_index_ = 0;
        retry_count_ = 0;
        state_enter_time_s_ = now_s;
        stable_start_time_s_ = 0.0;
        com_target_world_ = {};
        com_margin_m_ = -std::numeric_limits<double>::infinity();
        triangle_valid_ = false;
        ++transition_count_;
    }

    TerrainCrawlState Update(const TerrainCrawlSignals &signals) noexcept
    {
        if (!signals.transfer_window_active)
        {
            if (state_ != TerrainCrawlState::kInactive &&
                state_ != TerrainCrawlState::kAbort)
                Reset();
            return state_;
        }
        if (state_ == TerrainCrawlState::kInactive)
            Enter(signals.now_s);
        if (state_ == TerrainCrawlState::kAbort)
            return state_;

        const int contacts = signals.measured_contact_valid
            ? TerrainCrawlContactCount(signals.measured_contact) : 0;
        const bool three_contacts = signals.measured_contact_valid &&
            contacts >= 3;
        const bool valid_time = std::isfinite(signals.now_s);
        switch (state_)
        {
        case TerrainCrawlState::kApproach:
            if (signals.plan_valid && three_contacts)
                SetState(TerrainCrawlState::kDecelerateToCreep,
                         signals.now_s);
            break;
        case TerrainCrawlState::kDecelerateToCreep:
            if (signals.plan_valid && three_contacts &&
                std::isfinite(signals.measured_velocity_mps) &&
                signals.measured_velocity_mps >= 0.05 &&
                signals.measured_velocity_mps <= 0.30 + 1.0e-6)
                SetState(TerrainCrawlState::kShiftCom, signals.now_s);
            break;
        case TerrainCrawlState::kShiftCom:
            if (!three_contacts)
            {
                SetState(TerrainCrawlState::kAbort, signals.now_s);
                break;
            }
            UpdateComTarget(signals);
            if (signals.plan_valid && ComShiftReady())
                SetState(TerrainCrawlState::kCrawlStep, signals.now_s);
            break;
        case TerrainCrawlState::kCrawlStep:
        {
            const std::size_t leg = ActiveLeg();
            // The active leg may be in swing; the three remaining measured
            // contacts are the invariant that protects the crawl triangle.
            const int support_contacts = signals.measured_contact_valid
                ? TerrainCrawlContactCount(signals.measured_contact) -
                    (signals.measured_contact[leg] ? 1 : 0)
                : 0;
            if (support_contacts < 3)
            {
                SetState(TerrainCrawlState::kAbort, signals.now_s);
                break;
            }
            UpdateComTarget(signals);
            if (signals.step_failed)
            {
                if (retry_count_ < kMaxRetries)
                    ++retry_count_;
                else
                    SetState(TerrainCrawlState::kAbort, signals.now_s);
                break;
            }
            if (leg >= go2::kLegCount || !three_contacts ||
                !signals.plan_valid || !signals.target_valid[leg] ||
                !signals.committed[leg])
                break;
            retry_count_ = 0;
            if (order_index_ == 1)
                SetState(TerrainCrawlState::kAdvanceBody, signals.now_s);
            else if (order_index_ + 1 < kLegOrder.size())
            {
                ++order_index_;
                SetState(TerrainCrawlState::kShiftCom, signals.now_s);
            }
            else
                SetState(TerrainCrawlState::kClear, signals.now_s);
            break;
        }
        case TerrainCrawlState::kAdvanceBody:
            if (signals.plan_valid && three_contacts &&
                signals.rear_targets_fk_reachable)
            {
                ++order_index_;
                SetState(TerrainCrawlState::kShiftCom, signals.now_s);
            }
            break;
        case TerrainCrawlState::kClear:
            if (three_contacts && signals.base_clear &&
                signals.all_feet_clear)
            {
                stable_start_time_s_ = valid_time ? signals.now_s : 0.0;
                SetState(TerrainCrawlState::kResume, signals.now_s);
            }
            break;
        case TerrainCrawlState::kResume:
            if (signals.stable && valid_time &&
                signals.now_s - stable_start_time_s_ >= 0.45)
                Reset();
            break;
        default:
            break;
        }
        return state_;
    }

    TerrainCrawlState state() const noexcept { return state_; }
    int retry_count() const noexcept { return retry_count_; }
    std::size_t order_index() const noexcept { return order_index_; }
    std::size_t ActiveLeg() const noexcept
    {
        return state_ == TerrainCrawlState::kCrawlStep &&
                order_index_ < kLegOrder.size()
            ? kLegOrder[order_index_]
            : go2::kLegCount;
    }
    double state_enter_time_s() const noexcept { return state_enter_time_s_; }
    double stable_start_time_s() const noexcept { return stable_start_time_s_; }
    std::uint64_t transition_count() const noexcept { return transition_count_; }
    bool aborted() const noexcept { return state_ == TerrainCrawlState::kAbort; }
    bool com_target_valid() const noexcept { return triangle_valid_; }
    go2::Vec3 com_target_world() const noexcept { return com_target_world_; }
    double com_margin_m() const noexcept { return com_margin_m_; }
    std::size_t com_target_leg() const noexcept { return ActiveLegForSupport(); }

private:
    std::size_t ActiveLegForSupport() const noexcept
    {
        return order_index_ < kLegOrder.size() ? kLegOrder[order_index_]
                                                : go2::kLegCount;
    }

    void UpdateComTarget(const TerrainCrawlSignals &signals) noexcept
    {
        const std::size_t leg = ActiveLegForSupport();
        const auto triangle = ComputeTerrainSupportTriangle(
            signals.measured_foot_world, leg);
        triangle_valid_ = signals.measured_foot_valid && triangle.valid &&
            signals.measured_com_valid;
        if (!triangle_valid_)
        {
            com_margin_m_ = -std::numeric_limits<double>::infinity();
            return;
        }
        const auto metrics = MeasureTerrainSupportTriangle(
            triangle, signals.measured_com_world);
        com_margin_m_ = metrics.signed_margin_m;
        if (metrics.signed_margin_m >= kComMarginM)
            com_target_world_ = signals.measured_com_world;
        else
            com_target_world_ = TerrainSupportTriangleCentroid(triangle);
    }

    bool ComShiftReady() const noexcept
    {
        return triangle_valid_ && com_margin_m_ >= kComMarginM;
    }


    void SetState(TerrainCrawlState state, double now_s) noexcept
    {
        if (state_ != state)
            ++transition_count_;
        state_ = state;
        state_enter_time_s_ = now_s;
    }

    TerrainCrawlState state_ = TerrainCrawlState::kInactive;
    std::size_t order_index_ = 0;
    int retry_count_ = 0;
    double state_enter_time_s_ = 0.0;
    double stable_start_time_s_ = 0.0;
    std::uint64_t transition_count_ = 0;
    go2::Vec3 com_target_world_{};
    double com_margin_m_ = -std::numeric_limits<double>::infinity();
    bool triangle_valid_ = false;
};

}  // namespace go2_terrain
