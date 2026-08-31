#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "go2_forward_kinematics.h"
#include "terrain_crawl_script.h"

namespace go2_terrain
{

enum class TerrainCrawlState : std::uint8_t
{
    kInactive = 0,
    kApproach,
    kDecelerateToCreep,
    kStage,
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
    case TerrainCrawlState::kStage: return "STAGE";
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
    // Order-032 enables fixed timing only inside the scripted window.
    bool scripted_execution = false;
    bool plan_valid = false;
    // Sequencer STAGE must finish before the legacy machine can launch SWING.
    bool sequencer_stage_pending = false;
    // Keep the legacy contact override from lifting FL while the sequencer
    // is still in SHIFT; SWING owns the topology handoff.
    bool sequencer_pre_swing_pending = false;
    // The event sequencer owns the measured SWING boundary. Once it has
    // passed its own shift margin/force gates, synchronize the legacy state
    // in the same tick so its four-foot override cannot mask the swing.
    bool sequencer_swing_active = false;
    bool measured_contact_valid = false;
    std::array<bool, go2::kLegCount> measured_contact{};
    double measured_velocity_mps = 0.0;
    bool measured_posture_valid = false;
    double measured_roll_rad = 0.0;
    double measured_pitch_rad = 0.0;
    // Canonical entry is measured from the lidar edge in the current map.
    // The state machine receives only this relative error, never a scene
    // coordinate or a detection timestamp.
    bool staging_target_valid = false;
    double staging_error_m = 0.0;
    std::array<bool, go2::kLegCount> target_valid{};
    std::array<bool, go2::kLegCount> committed{};
    // Force balance and COM stillness are used only as a transfer-window
    // readiness witness; flat-ground callers leave these fields invalid.
    bool measured_force_valid = false;
    std::array<double, go2::kLegCount> measured_normal_force_n{};
    bool measured_com_velocity_valid = false;
    double measured_com_velocity_mps = 0.0;
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

struct TerrainStancePlane
{
    bool valid = false;
    go2::Vec3 normal{0.0, 0.0, 1.0};
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
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

struct TerrainSupportTriangleQuality
{
    bool valid = false;
    double area_m2 = 0.0;
    double minimum_altitude_m = 0.0;
};

// Geometric pre-check metric: area is the 3-D support triangle area and the
// minimum altitude is 2A divided by its longest edge.  The latter is the
// conservative load-transfer width, and remains meaningful for mixed-height
// support planes without introducing a COM or WBC assumption.
inline TerrainSupportTriangleQuality MeasureTerrainSupportTriangleQuality(
    const TerrainSupportTriangle &triangle) noexcept
{
    TerrainSupportTriangleQuality quality;
    if (!triangle.valid)
        return quality;
    double longest_edge = 0.0;
    for (std::size_t i = 0; i < triangle.vertex.size(); ++i)
    {
        const auto &a = triangle.vertex[i];
        const auto &b = triangle.vertex[(i + 1) % triangle.vertex.size()];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double dz = b.z - a.z;
        longest_edge = std::max(longest_edge,
            std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    const auto &a = triangle.vertex[0];
    const auto &b = triangle.vertex[1];
    const auto &c = triangle.vertex[2];
    const go2::Vec3 ab{b.x - a.x, b.y - a.y, b.z - a.z};
    const go2::Vec3 ac{c.x - a.x, c.y - a.y, c.z - a.z};
    const go2::Vec3 cross{
        ab.y * ac.z - ab.z * ac.y,
        ab.z * ac.x - ab.x * ac.z,
        ab.x * ac.y - ab.y * ac.x};
    quality.area_m2 = 0.5 * std::sqrt(
        cross.x * cross.x + cross.y * cross.y + cross.z * cross.z);
    quality.valid = std::isfinite(quality.area_m2) &&
        std::isfinite(longest_edge) && quality.area_m2 > 1.0e-9 &&
        longest_edge > 1.0e-9;
    if (quality.valid)
        quality.minimum_altitude_m = 2.0 * quality.area_m2 / longest_edge;
    return quality;
}

// Return the body attitude whose z axis is normal to the measured support
// triangle. The plane is kept in world coordinates, then expressed in the
// yaw-aligned body frame so non-zero heading is handled correctly.
inline TerrainStancePlane ComputeTerrainStancePlane(
    const TerrainSupportTriangle &triangle, double base_yaw_rad) noexcept
{
    TerrainStancePlane plane;
    if (!triangle.valid || !std::isfinite(base_yaw_rad))
        return plane;
    const go2::Vec3 e01{
        triangle.vertex[1].x - triangle.vertex[0].x,
        triangle.vertex[1].y - triangle.vertex[0].y,
        triangle.vertex[1].z - triangle.vertex[0].z};
    const go2::Vec3 e02{
        triangle.vertex[2].x - triangle.vertex[0].x,
        triangle.vertex[2].y - triangle.vertex[0].y,
        triangle.vertex[2].z - triangle.vertex[0].z};
    go2::Vec3 normal{
        e01.y * e02.z - e01.z * e02.y,
        e01.z * e02.x - e01.x * e02.z,
        e01.x * e02.y - e01.y * e02.x};
    const double length = std::sqrt(
        normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
    if (!(length > 1.0e-6))
        return plane;
    if (normal.z < 0.0)
    {
        normal.x = -normal.x;
        normal.y = -normal.y;
        normal.z = -normal.z;
    }
    plane.normal = {normal.x / length, normal.y / length, normal.z / length};
    const double c = std::cos(base_yaw_rad);
    const double s = std::sin(base_yaw_rad);
    // Rz(yaw) Ry(pitch) Rx(roll) * [0,0,1] = plane.normal.
    const double body_x = c * plane.normal.x + s * plane.normal.y;
    const double body_y = -s * plane.normal.x + c * plane.normal.y;
    plane.roll_rad = std::atan2(-body_y, plane.normal.z);
    plane.pitch_rad = std::atan2(body_x,
        std::hypot(body_y, plane.normal.z));
    plane.valid = std::isfinite(plane.roll_rad) &&
        std::isfinite(plane.pitch_rad);
    return plane;
}

// Fit the measured four-foot support surface after a leg commit. A
// committed raised foot and the three remaining contact feet are generally
// not exactly coplanar in the plant, so use the least-squares plane z=ax+by+c
// rather than silently discarding the fourth measured support point.
inline TerrainStancePlane ComputeTerrainStancePlaneFromFeet(
    const std::array<go2::Vec3, go2::kLegCount> &feet,
    double base_yaw_rad) noexcept
{
    TerrainStancePlane plane;
    if (!std::isfinite(base_yaw_rad))
        return plane;
    double normal_matrix[3][3]{};
    double rhs[3]{};
    for (const auto &foot : feet)
    {
        if (!std::isfinite(foot.x) || !std::isfinite(foot.y) ||
            !std::isfinite(foot.z))
            return plane;
        const double feature[3] = {foot.x, foot.y, 1.0};
        for (int row = 0; row < 3; ++row)
        {
            rhs[row] += feature[row] * foot.z;
            for (int column = 0; column < 3; ++column)
                normal_matrix[row][column] +=
                    feature[row] * feature[column];
        }
    }
    for (int pivot = 0; pivot < 3; ++pivot)
    {
        int best = pivot;
        for (int row = pivot + 1; row < 3; ++row)
            if (std::abs(normal_matrix[row][pivot]) >
                std::abs(normal_matrix[best][pivot]))
                best = row;
        if (std::abs(normal_matrix[best][pivot]) <= 1.0e-9)
            return plane;
        if (best != pivot)
        {
            for (int column = pivot; column < 3; ++column)
                std::swap(normal_matrix[pivot][column],
                          normal_matrix[best][column]);
            std::swap(rhs[pivot], rhs[best]);
        }
        for (int row = pivot + 1; row < 3; ++row)
        {
            const double scale = normal_matrix[row][pivot] /
                normal_matrix[pivot][pivot];
            for (int column = pivot; column < 3; ++column)
                normal_matrix[row][column] -=
                    scale * normal_matrix[pivot][column];
            rhs[row] -= scale * rhs[pivot];
        }
    }
    double coefficient[3]{};
    for (int row = 2; row >= 0; --row)
    {
        coefficient[row] = rhs[row];
        for (int column = row + 1; column < 3; ++column)
            coefficient[row] -= normal_matrix[row][column] *
                coefficient[column];
        coefficient[row] /= normal_matrix[row][row];
    }
    go2::Vec3 normal{-coefficient[0], -coefficient[1], 1.0};
    const double length = std::sqrt(
        normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
    if (!(length > 1.0e-6) || !std::isfinite(length))
        return plane;
    plane.normal = {normal.x / length, normal.y / length,
                    normal.z / length};
    const double c = std::cos(base_yaw_rad);
    const double sine = std::sin(base_yaw_rad);
    const double body_x = c * plane.normal.x + sine * plane.normal.y;
    const double body_y = -sine * plane.normal.x + c * plane.normal.y;
    plane.roll_rad = std::atan2(-body_y, plane.normal.z);
    plane.pitch_rad = std::atan2(body_x,
        std::hypot(body_y, plane.normal.z));
    plane.valid = std::isfinite(plane.roll_rad) &&
        std::isfinite(plane.pitch_rad);
    return plane;
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

// The incenter is the measured support triangle's most interior point.
// Weighting vertices by their opposite 3-D edge lengths keeps the target on
// the measured support plane, including a raised terrain vertex.
inline go2::Vec3 TerrainSupportTriangleIncenter(
    const TerrainSupportTriangle &triangle) noexcept
{
    if (!triangle.valid)
        return {};
    const auto edge_length = [](const go2::Vec3 &a, const go2::Vec3 &b) {
        return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
    };
    const double weight0 = edge_length(triangle.vertex[1], triangle.vertex[2]);
    const double weight1 = edge_length(triangle.vertex[0], triangle.vertex[2]);
    const double weight2 = edge_length(triangle.vertex[0], triangle.vertex[1]);
    const double perimeter = weight0 + weight1 + weight2;
    if (!std::isfinite(perimeter) || perimeter <= 1.0e-9)
        return {};
    return {(weight0 * triangle.vertex[0].x +
             weight1 * triangle.vertex[1].x +
             weight2 * triangle.vertex[2].x) / perimeter,
            (weight0 * triangle.vertex[0].y +
             weight1 * triangle.vertex[1].y +
             weight2 * triangle.vertex[2].y) / perimeter,
            (weight0 * triangle.vertex[0].z +
             weight1 * triangle.vertex[1].z +
             weight2 * triangle.vertex[2].z) / perimeter};
}

// Canonical measured support margin shared by the sequencer and planner.
// A valid lifted leg selects the other three measured feet; ADVANCE passes
// kLegCount and uses the three currently measured contacts.
// Four-contact STAGE uses the measured convex support polygon. The lifted
// leg path remains the measured 3-D triangle used by SHIFT_COM.
inline double TerrainMeasuredSupportPolygonMargin(
    const std::array<go2::Vec3, go2::kLegCount> &feet,
    const std::array<bool, go2::kLegCount> &contact,
    const go2::Vec3 &com) noexcept
{
    std::vector<go2::Vec3> points;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        if (contact[leg] && std::isfinite(feet[leg].x) &&
            std::isfinite(feet[leg].y))
            points.push_back(feet[leg]);
    if (points.size() < 3)
        return -std::numeric_limits<double>::infinity();
    std::sort(points.begin(), points.end(), [](const go2::Vec3 &a,
                                               const go2::Vec3 &b) {
        return a.x != b.x ? a.x < b.x : a.y < b.y;
    });
    const auto cross = [](const go2::Vec3 &a, const go2::Vec3 &b,
                          const go2::Vec3 &p) {
        return (b.x - a.x) * (p.y - a.y) -
               (b.y - a.y) * (p.x - a.x);
    };
    std::vector<go2::Vec3> hull;
    for (const auto &point : points) {
        while (hull.size() >= 2 &&
               cross(hull[hull.size() - 2], hull.back(), point) <= 0.0)
            hull.pop_back();
        hull.push_back(point);
    }
    const std::size_t lower = hull.size();
    for (std::size_t i = points.size(); i-- > 0;) {
        const auto &point = points[i];
        while (hull.size() > lower &&
               cross(hull[hull.size() - 2], hull.back(), point) <= 0.0)
            hull.pop_back();
        hull.push_back(point);
    }
    if (hull.size() > 1)
        hull.pop_back();
    if (hull.size() < 3 || !std::isfinite(com.x) || !std::isfinite(com.y))
        return -std::numeric_limits<double>::infinity();
    double margin = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const auto &a = hull[i];
        const auto &b = hull[(i + 1) % hull.size()];
        const double length = std::hypot(b.x - a.x, b.y - a.y);
        if (!(length > 1.0e-9))
            return -std::numeric_limits<double>::infinity();
        margin = std::min(margin, cross(a, b, com) / length);
    }
    return margin;
}

inline go2::Vec3 TerrainMeasuredSupportPolygonIncenter(
    const std::array<go2::Vec3, go2::kLegCount> &feet,
    const std::array<bool, go2::kLegCount> &contact) noexcept
{
    std::vector<go2::Vec3> points;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        if (contact[leg] && std::isfinite(feet[leg].x) &&
            std::isfinite(feet[leg].y))
            points.push_back(feet[leg]);
    if (points.size() < 3)
        return {};
    std::sort(points.begin(), points.end(), [](const go2::Vec3 &a,
                                               const go2::Vec3 &b) {
        return a.x != b.x ? a.x < b.x : a.y < b.y;
    });
    const auto cross = [](const go2::Vec3 &a, const go2::Vec3 &b,
                          const go2::Vec3 &p) {
        return (b.x - a.x) * (p.y - a.y) -
               (b.y - a.y) * (p.x - a.x);
    };
    std::vector<go2::Vec3> hull;
    for (const auto &point : points) {
        while (hull.size() >= 2 &&
               cross(hull[hull.size() - 2], hull.back(), point) <= 0.0)
            hull.pop_back();
        hull.push_back(point);
    }
    const std::size_t lower = hull.size();
    for (std::size_t i = points.size(); i-- > 0;) {
        const auto &point = points[i];
        while (hull.size() > lower &&
               cross(hull[hull.size() - 2], hull.back(), point) <= 0.0)
            hull.pop_back();
        hull.push_back(point);
    }
    if (hull.size() > 1)
        hull.pop_back();
    if (hull.size() < 3)
        return {};
    double sum = 0.0;
    go2::Vec3 result{};
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const auto &a = hull[i];
        const auto &b = hull[(i + 1) % hull.size()];
        const double weight = std::hypot(b.x - a.x, b.y - a.y);
        sum += weight;
        result.x += weight * a.x;
        result.y += weight * a.y;
        result.z += weight * a.z;
    }
    if (!(sum > 1.0e-9))
        return {};
    return {result.x / sum, result.y / sum, result.z / sum};
}

inline double TerrainMeasuredSupportMargin(
    const std::array<go2::Vec3, go2::kLegCount> &feet,
    const std::array<bool, go2::kLegCount> &contact,
    std::size_t lifted_leg, const go2::Vec3 &com) noexcept
{
    TerrainSupportTriangle triangle;
    if (lifted_leg < go2::kLegCount)
    {
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            if (leg != lifted_leg && !contact[leg])
                return -std::numeric_limits<double>::infinity();
        triangle = ComputeTerrainSupportTriangle(feet, lifted_leg);
    }
    else
    {
        std::size_t out = 0;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            if (contact[leg] && out < triangle.vertex.size())
                triangle.vertex[out++] = feet[leg];
        triangle.valid = out == triangle.vertex.size();
        if (triangle.valid)
        {
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
            triangle.valid = std::isfinite(cross.x) &&
                std::isfinite(cross.y) && std::isfinite(cross.z) &&
                std::sqrt(cross.x * cross.x + cross.y * cross.y +
                          cross.z * cross.z) > 1.0e-6;
        }
    }
    if (lifted_leg >= go2::kLegCount)
        return TerrainMeasuredSupportPolygonMargin(feet, contact, com);
    return triangle.valid
        ? MeasureTerrainSupportTriangle(triangle, com).signed_margin_m
        : -std::numeric_limits<double>::infinity();
}

class TerrainCrawlStateMachine
{
public:
    // Historical alias retained for legacy callers; instances select an
    // order so the sequencer and this owner cannot diverge.
    static constexpr std::array<std::size_t, go2::kLegCount> kLegOrder =
        kLegacyFrontFirstLegOrder;
    static constexpr std::array<std::size_t, go2::kLegCount> kLateralOrder =
        kLateralLegOrder;
    static constexpr int kMaxRetries = 2;
    static constexpr double kComMarginM = 0.02;
    static constexpr double kComShiftRampS = 0.40;
    // Asymmetric support can need more than the symmetric 0.40 s ramp. The
    // duration is selected from measured COM-to-incenter displacement.
    static constexpr double kComShiftRampMaxS = 1.20;
    static constexpr double kComShiftDistanceRateMps = 0.10;
    static constexpr double kStableComMarginM = -0.040;
    static constexpr double kStableComVelocityMps = 0.08;
    static constexpr double kStableForceMinN = 10.0;
    static constexpr double kStableForceTotalMinN = 50.0;
    static constexpr double kStableForceImbalanceRatio = 4.0;
    static constexpr double kStableDwellS = 0.12;
    static constexpr double kComShiftTimeoutS = 2.50;
    static constexpr int kMaxShiftRecoveries = 2;
    // 250 Hz control tick; keep the shift reference latency below 20 ms.
    static constexpr int kComShiftMpcPeriodTicks = 5;
    // The four-foot shift is the only path that asks stance to resist a
    // moving COM reference; retain ordinary non-transfer weights elsewhere.
    static constexpr double kShiftStanceNoSlipWeight = 80.0;
    // A lower handoff speed leaves the trot controller enough authority to
    // settle without transferring a residual stride into SHIFT_COM.
    static constexpr double kCreepSpeedMps = 0.08;
    // Body advance is a bounded, window-scoped creep rather than a stop:
    // rear-target reachability is evaluated at the moving measured pose.
    static constexpr double kAdvanceBodySpeedMps = 0.12;
    static constexpr double kEntrySettleS = 0.24;
    static constexpr double kStageSettleS = 0.30;
    static constexpr double kStagePositionToleranceM = 0.015;
    static constexpr double kStageVelocityToleranceMps = 0.04;
    static constexpr double kStagePostureLimitRad = 0.08;
    static constexpr double kStageTimeoutS = 4.0;
    // Order-060 requires the measured pre-SHIFT basin, not merely an edge
    // standoff. Failed basin checks get two small alternating creep probes.
    static constexpr double kStageBasinMarginM = 0.020;
    static constexpr double kStageTargetMarginM = 0.030;
    static constexpr double kStageBasinHalfWidthM =
        0.5 * (0.330 - 0.318);
    static constexpr int kMaxStageRetries = 2;
    static constexpr double kStageMicroAdjustSpeedMps = 0.03;
    static constexpr double kStageMicroAdjustDurationS = 0.50;
    static constexpr double kCanonicalStandoffM = 0.25;
    // The command ramp is still driven to kCreepSpeedMps; this guard keeps a
    // falling trot from waiting for a noisy velocity estimate to reach it.
    static constexpr double kEntryVelocityGuardMps = 0.50;
    static constexpr double kEntryPostureLimitRad = 0.20;
    // During crawl, posture safety is measured from the deliberate stance
    // plane reference; this is not an absolute flat-ground angle limit.
    static constexpr double kStancePostureDeviationLimitRad = 0.20;
    static constexpr double kContactRecoveryGraceS = 0.80;
    static constexpr double kCrawlStepHandoffGraceS = 0.10;
    // Allow the endpoint confirmation to arrive after the first force sample
    // that still contains the active leg, without weakening the three-foot
    // invariant for a genuinely unsupported swing.
    static constexpr double kCrawlStepCommitGraceS = 0.30;
    // Endpoint holding can trail the fixed flight deadline while the force
    // filter settles. Keep the captured support until just before the
    // scripted 0.80 s retry boundary without changing the commit predicate.
    static constexpr double kCrawlStepEndpointGraceS = 0.70;

    void SetLegOrder(TerrainCrawlLegOrder order) noexcept
    {
        leg_order_ = order;
        leg_order_values_ = order == TerrainCrawlLegOrder::kLateral
            ? kLateralLegOrder : kLegacyFrontFirstLegOrder;
    }

    void SetAdvancePolicy(TerrainCrawlAdvancePolicy policy) noexcept
    {
        advance_policy_ = policy;
    }

    TerrainCrawlLegOrder leg_order() const noexcept { return leg_order_; }
    TerrainCrawlAdvancePolicy advance_policy() const noexcept
    {
        return advance_policy_;
    }

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
        support_triangle_ = {};
        support_triangle_latched_ = false;
        committed_latched_.fill(false);
        com_shift_start_world_ = {};
        com_shift_start_time_s_ = 0.0;
        com_shift_start_valid_ = false;
        com_shift_duration_s_ = kComShiftRampS;
        asymmetric_shift_ = false;
        com_stable_start_time_s_ = std::numeric_limits<double>::infinity();
        shift_recovery_count_ = 0;
        decel_stable_start_time_s_ = std::numeric_limits<double>::infinity();
        stage_stable_start_time_s_ = std::numeric_limits<double>::infinity();
        stage_retry_count_ = 0;
        stage_micro_adjust_direction_ = 1.0;
        stage_micro_adjust_until_s_ =
            -std::numeric_limits<double>::infinity();
        stage_basin_margin_m_ = -std::numeric_limits<double>::infinity();
        stage_target_margin_m_ = -std::numeric_limits<double>::infinity();
        stage_com_target_valid_ = false;
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
        support_triangle_ = {};
        support_triangle_latched_ = false;
        committed_latched_.fill(false);
        com_shift_start_world_ = {};
        com_shift_start_time_s_ = 0.0;
        com_shift_start_valid_ = false;
        com_shift_duration_s_ = kComShiftRampS;
        asymmetric_shift_ = false;
        com_stable_start_time_s_ = std::numeric_limits<double>::infinity();
        shift_recovery_count_ = 0;
        decel_stable_start_time_s_ = std::numeric_limits<double>::infinity();
        stage_stable_start_time_s_ = std::numeric_limits<double>::infinity();
        stage_retry_count_ = 0;
        stage_micro_adjust_direction_ = 1.0;
        stage_micro_adjust_until_s_ =
            -std::numeric_limits<double>::infinity();
        stage_basin_margin_m_ = -std::numeric_limits<double>::infinity();
        stage_target_margin_m_ = -std::numeric_limits<double>::infinity();
        stage_com_target_valid_ = false;
        ++transition_count_;
    }

    // Harness-only staged-start entry. It preserves the measured STAGE
    // lifecycle while removing approach/deceleration from crawl isolation.
    void EnterStaged(double now_s) noexcept
    {
        Enter(now_s);
        state_ = TerrainCrawlState::kStage;
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
        if (!signals.step_failed)
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                committed_latched_[leg] = committed_latched_[leg] ||
                    signals.committed[leg];
        if (state_ == TerrainCrawlState::kAbort)
            return state_;
        // A commit can arrive on the same tick that a bounded recovery has
        // already returned to SHIFT_COM. Consume the latched fact before
        // selecting another target; otherwise recovery would retry that leg.
        if (!signals.step_failed && state_ == TerrainCrawlState::kShiftCom) {
            const std::size_t leg = ActiveLegForSupport();
            if (leg < go2::kLegCount && committed_latched_[leg] &&
                ForceBalanceReady(signals, leg)) {
                if (ShouldAdvanceBodyAfterCommit())
                    SetState(TerrainCrawlState::kAdvanceBody, signals.now_s);
                else if (order_index_ + 1 < leg_order_values_.size()) {
                    ++order_index_;
                    SetState(TerrainCrawlState::kShiftCom, signals.now_s);
                } else
                    SetState(TerrainCrawlState::kClear, signals.now_s);
                return state_;
            }
        }

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
        {
            // Keep the moving body in the support pattern until bounded entry
            // speed and posture have settled for a complete dwell. This
            // prevents a transient low velocity during a fall from entering
            // SHIFT_COM.
            const bool posture_settled = signals.measured_posture_valid &&
                std::isfinite(signals.measured_roll_rad) &&
                std::isfinite(signals.measured_pitch_rad) &&
                std::abs(signals.measured_roll_rad) <= kEntryPostureLimitRad &&
                std::abs(signals.measured_pitch_rad) <= kEntryPostureLimitRad;
            const bool entry_settled = signals.measured_contact_valid &&
                contacts >= 2 &&
                posture_settled &&
                std::isfinite(signals.measured_velocity_mps) &&
                signals.measured_velocity_mps >= 0.0 &&
                signals.measured_velocity_mps <= kEntryVelocityGuardMps;
            if (!entry_settled)
            {
                decel_stable_start_time_s_ = std::numeric_limits<double>::infinity();
                break;
            }
            if (!std::isfinite(decel_stable_start_time_s_))
                decel_stable_start_time_s_ = signals.now_s;
            if (std::isfinite(signals.now_s) &&
                signals.now_s - decel_stable_start_time_s_ + 1.0e-9 >=
                    kEntrySettleS)
            {
                // Scripted v2 entry always passes through canonical STAGE;
                // non-scripted callers retain the pre-Order-041 transition.
                SetState(signals.scripted_execution
                             ? TerrainCrawlState::kStage
                             : TerrainCrawlState::kShiftCom,
                         signals.now_s);
            }
            break;
        }
        case TerrainCrawlState::kStage:
        {
            int force_contacts = 0;
            if (signals.measured_force_valid)
                for (const double force : signals.measured_normal_force_n)
                    if (std::isfinite(force) && force >= kStableForceMinN)
                        ++force_contacts;
            const bool stage_contact_ready = signals.scripted_execution
                ? (signals.measured_contact_valid &&
                   contacts == static_cast<int>(go2::kLegCount))
                : ((signals.measured_contact_valid && contacts >= 3) ||
                   (signals.measured_force_valid && force_contacts >= 3));
            // STAGE retains all four contacts; no swing leg should be
            // excluded until SHIFT selects the active leg.
            const std::size_t lifted_leg = state_ ==
                TerrainCrawlState::kStage ? go2::kLegCount :
                ActiveLegForSupport();
            stage_basin_margin_m_ = -std::numeric_limits<double>::infinity();
            stage_com_target_valid_ = false;
            stage_target_margin_m_ = -std::numeric_limits<double>::infinity();
            if (signals.scripted_execution && stage_contact_ready &&
                signals.measured_foot_valid && signals.measured_com_valid)
            {
                stage_basin_margin_m_ = TerrainMeasuredSupportMargin(
                    signals.measured_foot_world, signals.measured_contact,
                    lifted_leg, signals.measured_com_world);
                const auto target = TerrainMeasuredSupportPolygonIncenter(
                    signals.measured_foot_world, signals.measured_contact);
                stage_target_margin_m_ =
                    TerrainMeasuredSupportPolygonMargin(
                        signals.measured_foot_world, signals.measured_contact,
                        target);
                if (std::isfinite(stage_target_margin_m_) &&
                    stage_basin_margin_m_ < kStageBasinMarginM &&
                    stage_target_margin_m_ >= kStageTargetMarginM)
                {
                    com_target_world_ = target;
                    com_target_world_.z = signals.measured_com_world.z;
                    stage_com_target_valid_ = true;
                }
            }
            const bool basin_ready = !signals.scripted_execution ||
                (std::isfinite(stage_basin_margin_m_) &&
                 stage_basin_margin_m_ >= kStageBasinMarginM);
            // If transfer detection is early enough to see the edge after
            // the body has crossed the canonical point, settling in place is
            // valid; reversing would reintroduce the late-trot failure.
            const bool already_past_standoff =
                std::isfinite(signals.staging_error_m) &&
                signals.staging_error_m < 0.0;
            // Scripted STAGE already has sequencer authority and a measured
            // support witness. The transient map-edge target may be invalid
            // after the body reaches the edge, so it cannot gate the bounded
            // settle/micro-adjust servo; the measured basin gate below does.
            const bool stage_location_ready = signals.scripted_execution
                ? true
                : (signals.staging_target_valid &&
                   std::isfinite(signals.staging_error_m) &&
                   // Do not turn the empirical base-x band into a hard gate;
                   // measured support margin is the causal pre-SHIFT witness.
                   (already_past_standoff ||
                    std::abs(signals.staging_error_m) <=
                        kStagePositionToleranceM));
            const bool stage_ready = stage_location_ready && stage_contact_ready &&
                std::isfinite(signals.measured_velocity_mps) &&
                signals.measured_velocity_mps <= kStageVelocityToleranceMps &&
                signals.measured_posture_valid &&
                std::isfinite(signals.measured_roll_rad) &&
                std::isfinite(signals.measured_pitch_rad) &&
                std::abs(signals.measured_roll_rad) <= kStagePostureLimitRad &&
                std::abs(signals.measured_pitch_rad) <= kStagePostureLimitRad;
            if (stage_ready)
            {
                if (!std::isfinite(stage_stable_start_time_s_))
                    stage_stable_start_time_s_ = signals.now_s;
                const bool stage_dwell_complete =
                    signals.now_s - stage_stable_start_time_s_ + 1.0e-9 >=
                        kStageSettleS;
                if (basin_ready && stage_dwell_complete)
                    SetState(TerrainCrawlState::kShiftCom, signals.now_s);
                else if (!basin_ready && signals.scripted_execution &&
                         stage_dwell_complete)
                {
                    if (stage_retry_count_ < kMaxStageRetries)
                    {
                        ++stage_retry_count_;
                        stage_micro_adjust_direction_ =
                            -stage_micro_adjust_direction_;
                        stage_micro_adjust_until_s_ = signals.now_s +
                            kStageMicroAdjustDurationS;
                        stage_stable_start_time_s_ =
                            std::numeric_limits<double>::infinity();
                    }
                    else
                        SetState(TerrainCrawlState::kAbort, signals.now_s);
                }
            }
            else
                stage_stable_start_time_s_ =
                    std::numeric_limits<double>::infinity();
            if (signals.scripted_execution &&
                std::isfinite(signals.now_s) &&
                signals.now_s - state_enter_time_s_ + 1.0e-9 >=
                    kStageTimeoutS)
                SetState(TerrainCrawlState::kAbort, signals.now_s);
            break;
        }
        case TerrainCrawlState::kShiftCom:
        {
            const std::size_t target_leg = ActiveLegForSupport();
            // The hysteretic contact bit can lag a quiet 10 N-class support
            // load. Treat a measured, balanced three-leg force plant as
            // equivalent for SHIFT_COM; this is the stability witness, not
            // a generic contact bypass.
            const bool force_supported = ForceBalanceReady(signals, target_leg);
            if (!three_contacts && !force_supported)
            {
                // Update() runs after target generation. Allow one control
                // tick for the newly selected crawl schedule to replace the
                // preceding trot phase before enforcing the crawl invariant.
                const bool gait_handoff_pending =
                    std::isfinite(signals.now_s) &&
                    signals.now_s - state_enter_time_s_ <
                        kContactRecoveryGraceS;
                if (!gait_handoff_pending)
                {
                    if (signals.scripted_execution &&
                        shift_recovery_count_ < kMaxShiftRecoveries)
                    {
                        ++shift_recovery_count_;
                        RestartShift(signals.now_s);
                    }
                    else
                        SetState(TerrainCrawlState::kAbort, signals.now_s);
                }
                break;
            }
            UpdateComTarget(signals);
            const double shift_elapsed = signals.now_s - state_enter_time_s_;
            const bool finite_shift_elapsed = std::isfinite(shift_elapsed) &&
                shift_elapsed >= 0.0;
            const bool ramp_ready = !signals.scripted_execution ||
                (finite_shift_elapsed && shift_elapsed + 1.0e-9 >=
                    com_shift_duration_s_);
            const bool geometric_ready = !signals.scripted_execution ||
                (finite_shift_elapsed && shift_elapsed + 1.0e-9 >=
                    com_shift_duration_s_ + 0.20);
            const bool force_balanced = ForceBalanceReady(signals, target_leg);
            const bool com_static = signals.measured_com_velocity_valid &&
                std::isfinite(signals.measured_com_velocity_mps) &&
                signals.measured_com_velocity_mps <= kStableComVelocityMps;
            const bool stability_ready = ramp_ready && force_balanced &&
                com_static && com_margin_m_ >= kStableComMarginM;
            if (stability_ready && !std::isfinite(com_stable_start_time_s_))
                com_stable_start_time_s_ = signals.now_s;
            if (!stability_ready)
                com_stable_start_time_s_ = std::numeric_limits<double>::infinity();
            const bool stable_ready = stability_ready && finite_shift_elapsed &&
                std::isfinite(com_stable_start_time_s_) &&
                signals.now_s - com_stable_start_time_s_ + 1.0e-9 >=
                    kStableDwellS;
            const bool shift_ready = geometric_ready && ComShiftReady() &&
                force_balanced;
            if (signals.plan_valid && !signals.sequencer_stage_pending &&
                !signals.sequencer_pre_swing_pending &&
                (shift_ready || signals.sequencer_swing_active ||
                 (!signals.scripted_execution && stable_ready)) &&
                target_leg < go2::kLegCount)
            {
                // The gait adapter prepares the selected target after this
                // update. Enter CRAWL_STEP once the live plan is ready so it
                // can perform that same-tick handoff; a missing target is
                // returned to SHIFT_COM on the next update without swinging.
                SetState(TerrainCrawlState::kCrawlStep, signals.now_s);
            }
            // Do not wait indefinitely for a geometric margin after the
            // stance has stopped carrying the COM. Re-start the measured
            // ramp (a bounded re-shift/re-square) before the existing abort.
            if (state_ == TerrainCrawlState::kShiftCom &&
                signals.scripted_execution && finite_shift_elapsed &&
                shift_elapsed + 1.0e-9 >= kComShiftTimeoutS)
            {
                if (shift_recovery_count_ < kMaxShiftRecoveries)
                {
                    ++shift_recovery_count_;
                    RestartShift(signals.now_s);
                }
                else
                    SetState(TerrainCrawlState::kAbort, signals.now_s);
            }
            break;
        }
        case TerrainCrawlState::kCrawlStep:
        {
            const std::size_t leg = ActiveLeg();
            // Failure wins over a same-tick commit latch.
            if (signals.step_failed)
            {
                if (retry_count_ < kMaxRetries)
                    ++retry_count_;
                else
                    SetState(TerrainCrawlState::kAbort, signals.now_s);
                break;
            }
            // Scripted swings have a fixed 0.60 s flight and a bounded 0.20 s
            // endpoint confirmation window. A missed measured commit retries
            // from SHIFT_COM instead of waiting on planner timing.
            if (signals.scripted_execution && leg < go2::kLegCount &&
                !signals.committed[leg] && std::isfinite(signals.now_s) &&
                signals.now_s - state_enter_time_s_ + 1.0e-9 >= 0.80)
            {
                if (retry_count_ < kMaxRetries)
                {
                    ++retry_count_;
                    SetState(TerrainCrawlState::kShiftCom, signals.now_s);
                }
                else
                    SetState(TerrainCrawlState::kAbort, signals.now_s);
                break;
            }
            // A measured touchdown is the transaction boundary. Advance to
            // the next leg before checking the old swing's support mask;
            // otherwise a force-filter sample that still contains the just
            // landed leg can make the old active-leg subtraction report one
            // support and abort before FR SHIFT_COM is entered.
            const bool active_leg_committed = leg < go2::kLegCount &&
                signals.plan_valid && committed_latched_[leg] &&
                ForceBalanceReady(signals, leg);
            if (active_leg_committed)
            {
                retry_count_ = 0;
                if (ShouldAdvanceBodyAfterCommit())
                    SetState(TerrainCrawlState::kAdvanceBody, signals.now_s);
                else if (order_index_ + 1 < leg_order_values_.size())
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
                // Endpoint promotion and the force-filter contact bit do
                // not arrive on the same tick. Keep the captured stance for
                // the bounded commit interval even when the active leg's
                // contact bit briefly drops while WBC settles the endpoint.
                const bool touchdown_boundary_pending =
                    std::isfinite(signals.now_s) &&
                    signals.now_s - state_enter_time_s_ <
                        kCrawlStepEndpointGraceS;
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
            if (leg >= go2::kLegCount || !three_contacts ||
                !signals.plan_valid || !signals.target_valid[leg] ||
                !committed_latched_[leg] || !ForceBalanceReady(signals, leg))
                break;
            retry_count_ = 0;
            if (ShouldAdvanceBodyAfterCommit())
                SetState(TerrainCrawlState::kAdvanceBody, signals.now_s);
            else if (order_index_ + 1 < leg_order_values_.size())
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
                signals.rear_targets_fk_reachable &&
                ForceBalanceReady(signals, go2::kLegCount))
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
        return state_ == TerrainCrawlState::kStage ||
            state_ == TerrainCrawlState::kShiftCom ||
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
                order_index_ < leg_order_values_.size()
            ? leg_order_values_[order_index_]
            : go2::kLegCount;
    }
    // The sequencer owns the next transition identity before a foothold
    // snapshot is available. This keeps candidate intent alive through the
    // asynchronous planner handoff instead of rediscovering it from each map.
    std::size_t PendingTransitionLeg() const noexcept
    {
        const bool pending = state_ == TerrainCrawlState::kApproach ||
            state_ == TerrainCrawlState::kDecelerateToCreep ||
            state_ == TerrainCrawlState::kStage ||
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
    // SHIFT owns one measured support snapshot. Keeping this geometry fixed
    // prevents kinematic foot drift from moving the incenter while the WBC
    // is already transferring the body toward it.
    const TerrainSupportTriangle &com_support_triangle() const noexcept
    {
        return support_triangle_;
    }
    std::size_t com_support_lifted_leg() const noexcept
    {
        return ActiveLegForSupport();
    }
    double com_shift_duration_s() const noexcept { return com_shift_duration_s_; }
    int shift_recovery_count() const noexcept { return shift_recovery_count_; }
    int stage_retry_count() const noexcept { return stage_retry_count_; }
    double stage_basin_margin_m() const noexcept { return stage_basin_margin_m_; }
    double stage_target_margin_m() const noexcept { return stage_target_margin_m_; }
    bool stage_com_target_valid() const noexcept { return stage_com_target_valid_; }
    bool stage_micro_adjust_active(double now_s) const noexcept
    {
        return state_ == TerrainCrawlState::kStage &&
            std::isfinite(now_s) && now_s < stage_micro_adjust_until_s_;
    }
    double stage_micro_adjust_direction() const noexcept
    {
        return stage_micro_adjust_direction_;
    }
    std::size_t com_target_leg() const noexcept { return ActiveLegForSupport(); }

private:
    bool ShouldAdvanceBodyAfterCommit() const noexcept
    {
        const std::size_t advance_index = advance_policy_ ==
            TerrainCrawlAdvancePolicy::kBeforeSecondStep ? 0 : 1;
        return order_index_ == advance_index;
    }

    std::size_t ActiveLegForSupport() const noexcept
    {
        return order_index_ < leg_order_values_.size() ? leg_order_values_[order_index_]
                                                : go2::kLegCount;
    }

    void UpdateComTarget(const TerrainCrawlSignals &signals) noexcept
    {
        const std::size_t leg = ActiveLegForSupport();
        if (!support_triangle_latched_ && signals.measured_foot_valid)
        {
            const auto measured_triangle = ComputeTerrainSupportTriangle(
                signals.measured_foot_world, leg);
            if (measured_triangle.valid)
            {
                support_triangle_ = measured_triangle;
                support_triangle_latched_ = true;
            }
        }
        const auto &triangle = support_triangle_;
        triangle_valid_ = support_triangle_latched_ && triangle.valid &&
            signals.measured_foot_valid && signals.measured_com_valid;
        if (!triangle_valid_)
        {
            com_margin_m_ = -std::numeric_limits<double>::infinity();
            return;
        }
        const auto metrics = MeasureTerrainSupportTriangle(
            triangle, signals.measured_com_world);
        asymmetric_shift_ = std::abs(triangle.vertex[0].z - triangle.vertex[1].z) >
                1.0e-4 ||
            std::abs(triangle.vertex[1].z - triangle.vertex[2].z) > 1.0e-4 ||
            std::abs(triangle.vertex[2].z - triangle.vertex[0].z) > 1.0e-4;
        com_margin_m_ = metrics.signed_margin_m;
        // CRAWL_STEP owns the already-prepared swing. Do not replace the
        // measured incenter with a new ramp origin when the swing wrench
        // briefly carries COM below the readiness margin; that makes the
        // target follow the drifting body across the support edge.
        if (state_ == TerrainCrawlState::kCrawlStep)
            return;
        if (metrics.signed_margin_m >= kComMarginM)
        {
            // The measured support is already safe, but retain the computed
            // incenter as the explicit handoff target so SHIFT and its
            // diagnostics use one triangle-derived reference.
            const auto interior = TerrainSupportTriangleIncenter(triangle);
            const auto interior_metrics = MeasureTerrainSupportTriangle(
                triangle, interior);
            com_target_world_ = interior_metrics.valid && interior_metrics.inside
                ? interior : signals.measured_com_world;
            com_shift_start_valid_ = false;
            return;
        }

        // The centroid can leave the measured COM close to an edge on an
        // asymmetric triangle. Use the measured triangle's incenter so the
        // final reference has a positive inward edge margin. Start at the
        // measured point and ramp the existing WBC reference, rather than
        // injecting that displacement in one MPC update and unloading the
        // stance legs. Asymmetric shifts get a displacement-proportional
        // ramp, capped to keep recovery bounded.
        const auto interior = TerrainSupportTriangleIncenter(triangle);
        if (!com_shift_start_valid_)
        {
            com_shift_start_world_ = signals.measured_com_world;
            com_shift_start_time_s_ = signals.now_s;
            const double distance = std::hypot(
                interior.x - com_shift_start_world_.x,
                interior.y - com_shift_start_world_.y);
            com_shift_duration_s_ = asymmetric_shift_
                ? std::clamp(
                      kComShiftRampS + distance / kComShiftDistanceRateMps,
                      kComShiftRampS, kComShiftRampMaxS)
                : kComShiftRampS;
            com_shift_start_valid_ = std::isfinite(com_shift_start_time_s_);
        }
        const double elapsed = signals.now_s - com_shift_start_time_s_;
        const double alpha = std::clamp(
            elapsed / com_shift_duration_s_, 0.0, 1.0);
        com_target_world_ = {
            com_shift_start_world_.x +
                alpha * (interior.x - com_shift_start_world_.x),
            com_shift_start_world_.y +
                alpha * (interior.y - com_shift_start_world_.y),
            com_shift_start_world_.z +
                alpha * (interior.z - com_shift_start_world_.z)};
    }

    bool ComShiftReady() const noexcept
    {
        return triangle_valid_ && com_margin_m_ >= kComMarginM;
    }

    bool ForceBalanceReady(const TerrainCrawlSignals &signals,
                           std::size_t lifted_leg) const noexcept
    {
        if (!signals.measured_force_valid)
            return false;
        double total = 0.0;
        double minimum = std::numeric_limits<double>::infinity();
        double maximum = 0.0;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (leg == lifted_leg)
                continue;
            const double force = signals.measured_normal_force_n[leg];
            if (!std::isfinite(force) || force < kStableForceMinN)
                return false;
            total += force;
            minimum = std::min(minimum, force);
            maximum = std::max(maximum, force);
        }
        return total >= kStableForceTotalMinN && minimum > 0.0 &&
            maximum / minimum <= kStableForceImbalanceRatio;
    }

    void RestartShift(double now_s) noexcept
    {
        state_enter_time_s_ = now_s;
        com_shift_start_world_ = {};
        com_shift_start_time_s_ = 0.0;
        com_shift_start_valid_ = false;
        com_shift_duration_s_ = kComShiftRampS;
        asymmetric_shift_ = false;
        com_stable_start_time_s_ = std::numeric_limits<double>::infinity();
        triangle_valid_ = false;
        support_triangle_ = {};
        support_triangle_latched_ = false;
        com_margin_m_ = -std::numeric_limits<double>::infinity();
        ++transition_count_;
    }

    void SetState(TerrainCrawlState state, double now_s) noexcept
    {
        if (state_ != state)
            ++transition_count_;
        if (state == TerrainCrawlState::kStage)
            stage_stable_start_time_s_ =
                std::numeric_limits<double>::infinity();
        if (state == TerrainCrawlState::kShiftCom)
        {
            com_shift_start_world_ = {};
            com_shift_start_time_s_ = 0.0;
            com_shift_start_valid_ = false;
            com_shift_duration_s_ = kComShiftRampS;
            com_stable_start_time_s_ = std::numeric_limits<double>::infinity();
            support_triangle_ = {};
            support_triangle_latched_ = false;
            shift_recovery_count_ = 0;
        }
        state_ = state;
        state_enter_time_s_ = now_s;
    }

    TerrainCrawlState state_ = TerrainCrawlState::kInactive;
    // A measured commit is a sequencer fact, not a planner snapshot field.
    // Keep it through bounded SHIFT_COM recovery so a cleared snapshot
    // cannot move the active leg backwards.
    std::array<bool, go2::kLegCount> committed_latched_{};
    TerrainCrawlLegOrder leg_order_ = TerrainCrawlLegOrder::kLegacyFrontFirst;
    TerrainCrawlAdvancePolicy advance_policy_ =
        TerrainCrawlAdvancePolicy::kAfterSecondStep;
    std::array<std::size_t, go2::kLegCount> leg_order_values_ =
        kLegacyFrontFirstLegOrder;
    std::size_t order_index_ = 0;
    int retry_count_ = 0;
    double state_enter_time_s_ = 0.0;
    double stable_start_time_s_ = 0.0;
    std::uint64_t transition_count_ = 0;
    go2::Vec3 com_target_world_{};
    double com_margin_m_ = -std::numeric_limits<double>::infinity();
    bool triangle_valid_ = false;
    TerrainSupportTriangle support_triangle_{};
    bool support_triangle_latched_ = false;
    go2::Vec3 com_shift_start_world_{};
    double com_shift_start_time_s_ = 0.0;
    bool com_shift_start_valid_ = false;
    double com_shift_duration_s_ = kComShiftRampS;
    double com_stable_start_time_s_ = std::numeric_limits<double>::infinity();
    int shift_recovery_count_ = 0;
    bool asymmetric_shift_ = false;
    double decel_stable_start_time_s_ = 0.0;
    double stage_stable_start_time_s_ = std::numeric_limits<double>::infinity();
    int stage_retry_count_ = 0;
    double stage_micro_adjust_direction_ = 1.0;
    double stage_micro_adjust_until_s_ =
        -std::numeric_limits<double>::infinity();
    double stage_basin_margin_m_ = -std::numeric_limits<double>::infinity();
    double stage_target_margin_m_ = -std::numeric_limits<double>::infinity();
    bool stage_com_target_valid_ = false;
};

}  // namespace go2_terrain
