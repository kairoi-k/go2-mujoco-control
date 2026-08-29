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

// The COM-shift state is a four-foot stance even before a terrain swing
// transaction exists. Keep this policy beside the state machine so the WBC
// cannot fall back to the running-trot schedule at handoff.
inline bool TerrainCrawlWbcContactOverride(
    TerrainCrawlState state,
    std::size_t active_leg,
    std::array<bool, go2::kLegCount> &contact) noexcept
{
    if (state == TerrainCrawlState::kShiftCom)
    {
        contact.fill(true);
        return true;
    }
    if (state == TerrainCrawlState::kCrawlStep &&
        active_leg < go2::kLegCount)
    {
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            contact[leg] = leg != active_leg;
        return true;
    }
    return false;
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
            std::isfinite(v.y) && std::isfinite(v.z);
    const go2::Vec3 edge01{
        triangle.vertex[1].x - triangle.vertex[0].x,
        triangle.vertex[1].y - triangle.vertex[0].y,
        triangle.vertex[1].z - triangle.vertex[0].z};
    const go2::Vec3 edge02{
        triangle.vertex[2].x - triangle.vertex[0].x,
        triangle.vertex[2].y - triangle.vertex[0].y,
        triangle.vertex[2].z - triangle.vertex[0].z};
    const go2::Vec3 cross{
        edge01.y * edge02.z - edge01.z * edge02.y,
        edge01.z * edge02.x - edge01.x * edge02.z,
        edge01.x * edge02.y - edge01.y * edge02.x};
    const double twice_area_3d = std::sqrt(
        cross.x * cross.x + cross.y * cross.y + cross.z * cross.z);
    triangle.valid = triangle.valid && twice_area_3d > 1.0e-6;
    return triangle;
}

inline TerrainSupportTriangleMetrics MeasureTerrainSupportTriangle(
    const TerrainSupportTriangle &triangle, const go2::Vec3 &point) noexcept
{
    TerrainSupportTriangleMetrics metrics;
    if (!triangle.valid || !std::isfinite(point.x) ||
        !std::isfinite(point.y) || !std::isfinite(point.z))
        return metrics;
    // Preserve the exact flat-ground arithmetic. The transfer-window-only
    // 3-D path below is needed only when a support vertex has a distinct
    // measured height.
    if (triangle.vertex[0].z == triangle.vertex[1].z &&
        triangle.vertex[1].z == triangle.vertex[2].z)
    {
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

    // Support is a 3-D triangle, not a height-discarding XY hull. Build an
    // orthonormal basis on its plane and orthogonally project the COM onto
    // that plane before measuring signed edge distances. This keeps a raised
    // committed foot in the geometry while retaining a scalar support margin.
    const go2::Vec3 e01{
        triangle.vertex[1].x - triangle.vertex[0].x,
        triangle.vertex[1].y - triangle.vertex[0].y,
        triangle.vertex[1].z - triangle.vertex[0].z};
    const go2::Vec3 e02{
        triangle.vertex[2].x - triangle.vertex[0].x,
        triangle.vertex[2].y - triangle.vertex[0].y,
        triangle.vertex[2].z - triangle.vertex[0].z};
    const go2::Vec3 normal{
        e01.y * e02.z - e01.z * e02.y,
        e01.z * e02.x - e01.x * e02.z,
        e01.x * e02.y - e01.y * e02.x};
    const double normal_length = std::sqrt(
        normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
    const double edge_length = std::sqrt(
        e01.x * e01.x + e01.y * e01.y + e01.z * e01.z);
    if (!(normal_length > 1.0e-6) || !(edge_length > 1.0e-9))
        return metrics;
    const go2::Vec3 u{
        e01.x / edge_length, e01.y / edge_length, e01.z / edge_length};
    const go2::Vec3 n{
        normal.x / normal_length, normal.y / normal_length,
        normal.z / normal_length};
    const go2::Vec3 v{
        n.y * u.z - n.z * u.y,
        n.z * u.x - n.x * u.z,
        n.x * u.y - n.y * u.x};
    const auto coordinates = [&](const go2::Vec3 &p) {
        const go2::Vec3 d{
            p.x - triangle.vertex[0].x,
            p.y - triangle.vertex[0].y,
            p.z - triangle.vertex[0].z};
        return std::array<double, 2>{
            d.x * u.x + d.y * u.y + d.z * u.z,
            d.x * v.x + d.y * v.y + d.z * v.z};
    };
    const go2::Vec3 d{
        point.x - triangle.vertex[0].x,
        point.y - triangle.vertex[0].y,
        point.z - triangle.vertex[0].z};
    const double normal_distance =
        d.x * n.x + d.y * n.y + d.z * n.z;
    const go2::Vec3 projected_point{
        point.x - normal_distance * n.x,
        point.y - normal_distance * n.y,
        point.z - normal_distance * n.z};
    const auto projected = coordinates(projected_point);
    std::array<std::array<double, 2>, 3> vertex{};
    for (std::size_t i = 0; i < vertex.size(); ++i)
        vertex[i] = coordinates(triangle.vertex[i]);
    const double area = (vertex[1][0] - vertex[0][0]) *
            (vertex[2][1] - vertex[0][1]) -
        (vertex[1][1] - vertex[0][1]) * (vertex[2][0] - vertex[0][0]);
    if (area < 0.0)
        std::swap(vertex[1], vertex[2]);
    metrics.valid = true;
    metrics.signed_margin_m = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < 3; ++i)
    {
        const auto &a = vertex[i];
        const auto &b = vertex[(i + 1) % vertex.size()];
        const double dx = b[0] - a[0];
        const double dy = b[1] - a[1];
        const double length = std::hypot(dx, dy);
        if (!(length > 1.0e-9))
            return TerrainSupportTriangleMetrics{};
        const double edge = (dx * (projected[1] - a[1]) -
                             dy * (projected[0] - a[0])) / length;
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
            (triangle.vertex[0].z + triangle.vertex[1].z + triangle.vertex[2].z) / 3.0};
}

class TerrainCrawlStateMachine
{
public:
    static constexpr std::array<std::size_t, go2::kLegCount> kLegOrder =
        {1, 0, 2, 3};
    static constexpr int kMaxRetries = 2;
    static constexpr double kComMarginM = 0.02;
    static constexpr double kComShiftRampS = 0.40;
    // 250 Hz control tick; keep the shift reference latency below 20 ms.
    static constexpr int kComShiftMpcPeriodTicks = 5;
    // The four-foot shift is the only path that asks stance to resist a
    // moving COM reference; retain ordinary non-transfer weights elsewhere.
    static constexpr double kShiftStanceNoSlipWeight = 80.0;
    static constexpr double kCreepSpeedMps = 0.12;
    static constexpr double kContactRecoveryGraceS = 0.80;
    static constexpr double kCrawlStepHandoffGraceS = 0.10;
    // Allow the endpoint confirmation to arrive after the first force sample
    // that still contains the active leg, without weakening the three-foot
    // invariant for a genuinely unsupported swing.
    static constexpr double kCrawlStepCommitGraceS = 0.30;

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
        com_shift_start_world_ = {};
        com_shift_start_time_s_ = 0.0;
        com_shift_start_valid_ = false;
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
        com_shift_start_world_ = {};
        com_shift_start_time_s_ = 0.0;
        com_shift_start_valid_ = false;
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
            // Running trot legitimately has diagonal and flight phases. Do
            // not apply the crawl support invariant until sequencing starts.
            if (signals.plan_valid)
                SetState(TerrainCrawlState::kDecelerateToCreep,
                         signals.now_s);
            break;
        case TerrainCrawlState::kDecelerateToCreep:
            // Keep trot in charge while slowing to creep; SHIFT_COM owns the
            // measured-contact invariant after this speed handoff.
            if (signals.plan_valid &&
                std::isfinite(signals.measured_velocity_mps) &&
                signals.measured_velocity_mps >= 0.05 &&
                signals.measured_velocity_mps <= kCreepSpeedMps)
                SetState(TerrainCrawlState::kShiftCom, signals.now_s);
            break;
        case TerrainCrawlState::kShiftCom:
        {
            if (!three_contacts)
            {
                // Update() runs after target generation. Allow one control
                // tick for the newly selected crawl schedule to replace the
                // preceding trot phase before enforcing the crawl invariant.
                const bool gait_handoff_pending =
                    std::isfinite(signals.now_s) &&
                    signals.now_s - state_enter_time_s_ <
                        kContactRecoveryGraceS;
                if (!gait_handoff_pending)
                    SetState(TerrainCrawlState::kAbort, signals.now_s);
                break;
            }
            UpdateComTarget(signals);
            const std::size_t target_leg = ActiveLegForSupport();
            if (signals.plan_valid && ComShiftReady() &&
                target_leg < go2::kLegCount)
            {
                // The gait adapter prepares the selected target after this
                // update. Enter CRAWL_STEP once the live plan is ready so it
                // can perform that same-tick handoff; a missing target is
                // returned to SHIFT_COM on the next update without swinging.
                SetState(TerrainCrawlState::kCrawlStep, signals.now_s);
            }
            break;
        }
        case TerrainCrawlState::kCrawlStep:
        {
            const std::size_t leg = ActiveLeg();
            // A measured touchdown is the transaction boundary. Advance to
            // the next leg before checking the old swing's support mask;
            // otherwise a force-filter sample that still contains the just
            // landed leg can make the old active-leg subtraction report one
            // support and abort before FR SHIFT_COM is entered.
            const bool active_leg_committed = leg < go2::kLegCount &&
                signals.plan_valid && signals.committed[leg];
            if (active_leg_committed)
            {
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
            // A stale prepared target cannot be repaired by changing its
            // immutable touchdown timestamp. Return to SHIFT_COM and wait
            // for the next fresh planner handoff instead of spending the
            // contact grace interval in a targetless swing state.
            if (leg >= go2::kLegCount || !signals.target_valid[leg])
            {
                SetState(TerrainCrawlState::kShiftCom, signals.now_s);
                break;
            }
            // The active leg may be in swing; the three remaining measured
            // contacts are the invariant that protects the crawl triangle.
            const int support_contacts = signals.measured_contact_valid
                ? TerrainCrawlContactCount(signals.measured_contact) -
                    (signals.measured_contact[leg] ? 1 : 0)
                : 0;
            if (support_contacts < 3)
            {
                // The force filter can still report the active foot loaded
                // while endpoint confirmation catches up. Give this bounded
                // interval to the commit path; once it expires, enforce the
                // three-foot invariant for a genuinely unsupported swing.
                const bool touchdown_boundary_pending =
                    signals.measured_contact_valid &&
                    leg < go2::kLegCount && signals.measured_contact[leg] &&
                    std::isfinite(signals.now_s) &&
                    signals.now_s - state_enter_time_s_ <
                        kCrawlStepHandoffGraceS + kCrawlStepCommitGraceS;
                if (touchdown_boundary_pending)
                {
                    break;
                }
                // The explicit contact override takes one control handoff to
                // replace the preceding trot schedule. Do not call a
                // transient boundary sample a failed three-foot stance.
                const bool crawl_handoff_pending =
                    std::isfinite(signals.now_s) &&
                    signals.now_s - state_enter_time_s_ <
                        kCrawlStepHandoffGraceS;
                if (!crawl_handoff_pending)
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
            if (signals.base_clear && signals.all_feet_clear)
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
    bool UsesCrawlExecution() const noexcept
    {
        return state_ == TerrainCrawlState::kShiftCom ||
            state_ == TerrainCrawlState::kCrawlStep ||
            state_ == TerrainCrawlState::kAdvanceBody ||
            state_ == TerrainCrawlState::kClear ||
            state_ == TerrainCrawlState::kResume ||
            state_ == TerrainCrawlState::kAbort;
    }
    int retry_count() const noexcept { return retry_count_; }
    std::size_t order_index() const noexcept { return order_index_; }
    std::size_t ActiveLeg() const noexcept
    {
        return state_ == TerrainCrawlState::kCrawlStep &&
                order_index_ < kLegOrder.size()
            ? kLegOrder[order_index_]
            : go2::kLegCount;
    }
    // The sequencer owns the next transition identity before a foothold
    // snapshot is available. This keeps candidate intent alive through the
    // asynchronous planner handoff instead of rediscovering it from each map.
    std::size_t PendingTransitionLeg() const noexcept
    {
        const bool pending = state_ == TerrainCrawlState::kApproach ||
            state_ == TerrainCrawlState::kDecelerateToCreep ||
            state_ == TerrainCrawlState::kShiftCom ||
            state_ == TerrainCrawlState::kCrawlStep ||
            state_ == TerrainCrawlState::kAdvanceBody;
        return pending ? ActiveLegForSupport() : go2::kLegCount;
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
        {
            com_target_world_ = signals.measured_com_world;
            com_shift_start_valid_ = false;
            return;
        }

        // A centroid target can be about 100 mm from the measured COM at
        // handoff. Start at the measured point and ramp the existing WBC
        // reference, rather than injecting that displacement in one MPC
        // update and unloading the stance legs.
        if (!com_shift_start_valid_)
        {
            com_shift_start_world_ = signals.measured_com_world;
            com_shift_start_time_s_ = signals.now_s;
            com_shift_start_valid_ = std::isfinite(com_shift_start_time_s_);
        }
        const double elapsed = signals.now_s - com_shift_start_time_s_;
        const double alpha = std::clamp(
            elapsed / kComShiftRampS, 0.0, 1.0);
        const auto centroid = TerrainSupportTriangleCentroid(triangle);
        com_target_world_ = {
            com_shift_start_world_.x +
                alpha * (centroid.x - com_shift_start_world_.x),
            com_shift_start_world_.y +
                alpha * (centroid.y - com_shift_start_world_.y),
            0.0};
    }

    bool ComShiftReady() const noexcept
    {
        return triangle_valid_ && com_margin_m_ >= kComMarginM;
    }


    void SetState(TerrainCrawlState state, double now_s) noexcept
    {
        if (state_ != state)
            ++transition_count_;
        if (state == TerrainCrawlState::kShiftCom &&
            state_ != TerrainCrawlState::kShiftCom)
        {
            com_shift_start_world_ = {};
            com_shift_start_time_s_ = 0.0;
            com_shift_start_valid_ = false;
        }
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
    go2::Vec3 com_shift_start_world_{};
    double com_shift_start_time_s_ = 0.0;
    bool com_shift_start_valid_ = false;
};

}  // namespace go2_terrain
