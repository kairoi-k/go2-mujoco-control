// Terrain-aware foothold and reference planning primitives.
//
// This module deliberately stops before torque control: it turns a local
// height map into feasible foot targets and a bounded body-motion reference.
// The existing gait kernel, SRBD-MPC, and ID-WBC remain the execution layer.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "go2_inverse_kinematics.h"

namespace go2_control::terrain
{

enum class TerrainPlanStatus
{
    kValid,
    kInvalidInput,
    kUnknownSurface,
    kNoSupportPatch,
    kStepTooHigh,
    kUnreachable,
};

struct TerrainCell
{
    double height_m = 0.0;
    bool known = false;
};

// A small, robot-local 2.5-D map. Cell centres are at
// (x_min + (ix + .5) * resolution, y_min + (iy + .5) * resolution).
class HeightMap
{
public:
    HeightMap() = default;

    HeightMap(
        double x_min_m,
        double y_min_m,
        double resolution_m,
        std::size_t nx,
        std::size_t ny)
        : x_min_m_(x_min_m),
          y_min_m_(y_min_m),
          resolution_m_(resolution_m),
          nx_(nx),
          ny_(ny),
          cells_(nx * ny)
    {
    }

    bool IsValid() const noexcept
    {
        return std::isfinite(x_min_m_) && std::isfinite(y_min_m_) &&
               std::isfinite(resolution_m_) && resolution_m_ > 0.0 &&
               nx_ > 0 && ny_ > 0 && cells_.size() == nx_ * ny_;
    }

    double x_min_m() const noexcept { return x_min_m_; }
    double y_min_m() const noexcept { return y_min_m_; }
    double resolution_m() const noexcept { return resolution_m_; }
    std::size_t nx() const noexcept { return nx_; }
    std::size_t ny() const noexcept { return ny_; }

    bool SetCell(
        std::size_t ix,
        std::size_t iy,
        double height_m,
        bool known = true) noexcept
    {
        if (ix >= nx_ || iy >= ny_ || !std::isfinite(height_m))
            return false;
        cells_[iy * nx_ + ix] = {height_m, known};
        return true;
    }

    bool CellAt(
        std::size_t ix,
        std::size_t iy,
        TerrainCell &cell) const noexcept
    {
        if (ix >= nx_ || iy >= ny_)
            return false;
        cell = cells_[iy * nx_ + ix];
        return true;
    }

    bool WorldToCell(
        double x_m,
        double y_m,
        std::size_t &ix,
        std::size_t &iy) const noexcept
    {
        if (!IsValid() || !std::isfinite(x_m) || !std::isfinite(y_m))
            return false;
        const double fx = (x_m - x_min_m_) / resolution_m_;
        const double fy = (y_m - y_min_m_) / resolution_m_;
        if (fx < 0.0 || fy < 0.0 || fx >= static_cast<double>(nx_) ||
            fy >= static_cast<double>(ny_))
        {
            return false;
        }
        ix = static_cast<std::size_t>(std::floor(fx));
        iy = static_cast<std::size_t>(std::floor(fy));
        return ix < nx_ && iy < ny_;
    }

    bool SampleHeight(
        double x_m,
        double y_m,
        double &height_m,
        bool &known) const noexcept
    {
        std::size_t ix = 0;
        std::size_t iy = 0;
        TerrainCell cell{};
        if (!WorldToCell(x_m, y_m, ix, iy) || !CellAt(ix, iy, cell))
            return false;
        height_m = cell.height_m;
        known = cell.known;
        return std::isfinite(height_m);
    }

    // Conservative local height for swept-volume checks. Sparse lidar cells
    // are sampled at arbitrary controller coordinates, so use the maximum
    // nearby known return instead of treating a single cell gap as terrain.
    bool SampleClearanceHeight(
        double x_m,
        double y_m,
        double radius_m,
        double &height_m,
        bool &known) const noexcept
    {
        known = false;
        height_m = 0.0;
        if (!IsValid() || !std::isfinite(radius_m) || radius_m < 0.0)
            return false;
        std::size_t centre_x = 0;
        std::size_t centre_y = 0;
        if (!WorldToCell(x_m, y_m, centre_x, centre_y))
            return false;
        const int radius_cells = static_cast<int>(
            std::ceil(radius_m / resolution_m_));
        double maximum = -std::numeric_limits<double>::infinity();
        int known_count = 0;
        for (int dy = -radius_cells; dy <= radius_cells; ++dy)
            for (int dx = -radius_cells; dx <= radius_cells; ++dx)
            {
                const double dx_m = static_cast<double>(dx) * resolution_m_;
                const double dy_m = static_cast<double>(dy) * resolution_m_;
                if (dx_m * dx_m + dy_m * dy_m >
                    radius_m * radius_m + 1.0e-12)
                    continue;
                const int ix = static_cast<int>(centre_x) + dx;
                const int iy = static_cast<int>(centre_y) + dy;
                if (ix < 0 || iy < 0 || ix >= static_cast<int>(nx_) ||
                    iy >= static_cast<int>(ny_))
                    continue;
                TerrainCell cell{};
                if (CellAt(static_cast<std::size_t>(ix),
                           static_cast<std::size_t>(iy), cell) &&
                    cell.known && std::isfinite(cell.height_m))
                {
                    maximum = std::max(maximum, cell.height_m);
                    ++known_count;
                }
            }
        if (known_count == 0)
            return false;
        height_m = maximum;
        known = true;
        return true;
    }

    // Check a circular support patch. A foot must land on a known, nearly
    // planar patch rather than on a cell edge, riser, or unknown region.
    bool SampleSupportPatch(
        double x_m,
        double y_m,
        double radius_m,
        double &height_m,
        double &height_range_m,
        double &slope_rad) const noexcept
    {
        if (!IsValid() || !std::isfinite(radius_m) || radius_m < 0.0)
            return false;
        std::size_t centre_x = 0;
        std::size_t centre_y = 0;
        if (!WorldToCell(x_m, y_m, centre_x, centre_y))
            return false;

        const int radius_cells = static_cast<int>(
            std::ceil(radius_m / resolution_m_));
        double min_height = std::numeric_limits<double>::infinity();
        double max_height = -std::numeric_limits<double>::infinity();
        double sum_height = 0.0;
        int count = 0;
        for (int dy = -radius_cells; dy <= radius_cells; ++dy)
        {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx)
            {
                const double dx_m = static_cast<double>(dx) * resolution_m_;
                const double dy_m = static_cast<double>(dy) * resolution_m_;
                if (dx_m * dx_m + dy_m * dy_m >
                    radius_m * radius_m + 1.0e-12)
                {
                    continue;
                }
                const int ix = static_cast<int>(centre_x) + dx;
                const int iy = static_cast<int>(centre_y) + dy;
                if (ix < 0 || iy < 0 || ix >= static_cast<int>(nx_) ||
                    iy >= static_cast<int>(ny_))
                {
                    return false;
                }
                TerrainCell cell{};
                if (!CellAt(
                        static_cast<std::size_t>(ix),
                        static_cast<std::size_t>(iy),
                        cell) ||
                    !cell.known || !std::isfinite(cell.height_m))
                {
                    return false;
                }
                min_height = std::min(min_height, cell.height_m);
                max_height = std::max(max_height, cell.height_m);
                sum_height += cell.height_m;
                ++count;
            }
        }
        if (count == 0)
            return false;

        height_m = sum_height / static_cast<double>(count);
        height_range_m = max_height - min_height;

        double hx_minus = 0.0;
        double hx_plus = 0.0;
        double hy_minus = 0.0;
        double hy_plus = 0.0;
        bool known = false;
        if (!SampleHeight(x_m - resolution_m_, y_m, hx_minus, known) ||
            !known || !SampleHeight(x_m + resolution_m_, y_m, hx_plus, known) ||
            !known || !SampleHeight(x_m, y_m - resolution_m_, hy_minus, known) ||
            !known || !SampleHeight(x_m, y_m + resolution_m_, hy_plus, known) ||
            !known)
        {
            return false;
        }
        const double grad_x =
            (hx_plus - hx_minus) / (2.0 * resolution_m_);
        const double grad_y =
            (hy_plus - hy_minus) / (2.0 * resolution_m_);
        slope_rad = std::atan(std::hypot(grad_x, grad_y));
        return std::isfinite(height_m) && std::isfinite(height_range_m) &&
               std::isfinite(slope_rad);
    }

private:
    double x_min_m_ = 0.0;
    double y_min_m_ = 0.0;
    double resolution_m_ = 0.0;
    std::size_t nx_ = 0;
    std::size_t ny_ = 0;
    std::vector<TerrainCell> cells_;
};

struct StaircaseSpec
{
    double start_x_m = 0.8;
    double tread_depth_m = 0.24;
    double riser_height_m = 0.08;
    int step_count = 4;
    double width_m = 0.75;
    double base_height_m = 0.0;
};

inline bool Valid(const StaircaseSpec &spec) noexcept
{
    return std::isfinite(spec.start_x_m) &&
           std::isfinite(spec.tread_depth_m) && spec.tread_depth_m > 0.0 &&
           std::isfinite(spec.riser_height_m) && spec.riser_height_m > 0.0 &&
           spec.step_count > 0 && std::isfinite(spec.width_m) &&
           spec.width_m > 0.2 && std::isfinite(spec.base_height_m);
}

inline double StaircaseHeightAt(
    const StaircaseSpec &spec,
    double x_m,
    double y_m) noexcept
{
    if (!Valid(spec) || !std::isfinite(x_m) || !std::isfinite(y_m) ||
        std::abs(y_m) > 0.5 * spec.width_m)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double end_x =
        spec.start_x_m + spec.tread_depth_m * static_cast<double>(spec.step_count);
    if (x_m < spec.start_x_m)
        return spec.base_height_m;
    if (x_m >= end_x)
    {
        return spec.base_height_m +
               spec.riser_height_m * static_cast<double>(spec.step_count);
    }
    const int step_index = static_cast<int>(
        std::floor((x_m - spec.start_x_m) / spec.tread_depth_m));
    return spec.base_height_m +
           spec.riser_height_m * static_cast<double>(step_index + 1);
}

inline bool FillStaircaseHeightMap(
    const StaircaseSpec &spec,
    HeightMap &map) noexcept
{
    if (!Valid(spec) || !map.IsValid())
        return false;
    for (std::size_t iy = 0; iy < map.ny(); ++iy)
    {
        for (std::size_t ix = 0; ix < map.nx(); ++ix)
        {
            const double x = map.x_min_m() +
                (static_cast<double>(ix) + 0.5) * map.resolution_m();
            const double y = map.y_min_m() +
                (static_cast<double>(iy) + 0.5) * map.resolution_m();
            const double height = StaircaseHeightAt(spec, x, y);
            if (std::isfinite(height))
            {
                if (!map.SetCell(ix, iy, height, true))
                    return false;
            }
            else if (!map.SetCell(ix, iy, 0.0, false))
            {
                return false;
            }
        }
    }
    return true;
}

struct TerrainFootholdPlannerParams
{
    double support_radius_m = 0.035;
    double max_surface_height_range_m = 0.020;
    double max_slope_rad = 0.30;
    double max_step_up_m = 0.14;
    double max_step_down_m = 0.18;
    double nominal_xy_weight = 1.0;
    double height_weight = 0.25;
    double slope_weight = 0.10;
    double height_reference_m = 0.0;
    std::array<double, 5> candidate_dx_m{{-0.12, -0.06, 0.0, 0.06, 0.12}};
    std::array<double, 3> candidate_dy_m{{-0.06, 0.0, 0.06}};
};

struct TerrainFootholdRequest
{
    go2::Leg leg = go2::Leg::FR;
    double base_world_x_m = 0.0;
    double base_world_y_m = 0.0;
    double base_world_z_m = 0.35;
    double nominal_body_x_m = 0.0;
    double nominal_body_y_m = 0.0;
    double nominal_body_z_m = -0.32;
    double reference_foot_world_z_m = 0.0;
};

struct TerrainFootholdOutput
{
    TerrainPlanStatus status = TerrainPlanStatus::kInvalidInput;
    go2::Vec3 body_foot{};
    double world_x_m = 0.0;
    double world_y_m = 0.0;
    double world_z_m = 0.0;
    double surface_height_range_m = 0.0;
    double slope_rad = 0.0;
    double cost = std::numeric_limits<double>::infinity();
};

inline bool PlanTerrainFoothold(
    const TerrainFootholdPlannerParams &params,
    const HeightMap &map,
    const TerrainFootholdRequest &request,
    TerrainFootholdOutput &output) noexcept
{
    output = {};
    output.status = TerrainPlanStatus::kInvalidInput;
    if (!map.IsValid() || !std::isfinite(params.support_radius_m) ||
        params.support_radius_m <= 0.0 ||
        !std::isfinite(params.max_surface_height_range_m) ||
        params.max_surface_height_range_m < 0.0 ||
        !std::isfinite(params.max_slope_rad) || params.max_slope_rad < 0.0 ||
        !std::isfinite(request.base_world_x_m) ||
        !std::isfinite(request.base_world_y_m) ||
        !std::isfinite(request.base_world_z_m) ||
        !std::isfinite(request.nominal_body_x_m) ||
        !std::isfinite(request.nominal_body_y_m) ||
        !std::isfinite(request.nominal_body_z_m) ||
        !std::isfinite(request.reference_foot_world_z_m))
    {
        return false;
    }

    bool saw_unknown = false;
    bool saw_bad_patch = false;
    bool saw_bad_step = false;
    bool saw_unreachable = false;
    for (const double dx : params.candidate_dx_m)
    {
        for (const double dy : params.candidate_dy_m)
        {
            const double body_x = request.nominal_body_x_m + dx;
            const double body_y = request.nominal_body_y_m + dy;
            const double world_x = request.base_world_x_m + body_x;
            const double world_y = request.base_world_y_m + body_y;
            double support_z = 0.0;
            double height_range = 0.0;
            double slope = 0.0;
            if (!map.SampleSupportPatch(
                    world_x,
                    world_y,
                    params.support_radius_m,
                    support_z,
                    height_range,
                    slope))
            {
                saw_unknown = true;
                continue;
            }
            if (height_range > params.max_surface_height_range_m)
            {
                saw_bad_patch = true;
                continue;
            }
            if (slope > params.max_slope_rad)
            {
                saw_bad_patch = true;
                continue;
            }
            const double step_delta =
                support_z - request.reference_foot_world_z_m;
            if (step_delta > params.max_step_up_m ||
                step_delta < -params.max_step_down_m)
            {
                saw_bad_step = true;
                continue;
            }

            const go2::Vec3 body_foot{
                body_x,
                body_y,
                support_z - request.base_world_z_m};
            go2::LegJointPositions joints{};
            if (!go2::LegInverseKinematics(request.leg, body_foot, joints))
            {
                saw_unreachable = true;
                continue;
            }
            const double xy_error = dx * dx + dy * dy;
            const double height_error =
                support_z - params.height_reference_m;
            const double cost =
                params.nominal_xy_weight * xy_error +
                params.height_weight * height_error * height_error +
                params.slope_weight * slope * slope;
            if (cost < output.cost)
            {
                output.status = TerrainPlanStatus::kValid;
                output.body_foot = body_foot;
                output.world_x_m = world_x;
                output.world_y_m = world_y;
                output.world_z_m = support_z;
                output.surface_height_range_m = height_range;
                output.slope_rad = slope;
                output.cost = cost;
            }
        }
    }
    if (output.status == TerrainPlanStatus::kValid)
        return true;
    if (saw_unreachable)
        output.status = TerrainPlanStatus::kUnreachable;
    else if (saw_bad_step)
        output.status = TerrainPlanStatus::kStepTooHigh;
    else if (saw_bad_patch)
        output.status = TerrainPlanStatus::kNoSupportPatch;
    else if (saw_unknown)
        output.status = TerrainPlanStatus::kUnknownSurface;
    return false;
}

enum class StaircasePhase
{
    kApproach,
    kClimb,
    kExit,
};

struct TerrainMotionReference
{
    double vx_mps = 0.0;
    double ground_height_m = 0.0;
    double pitch_rad = 0.0;
    double step_length_m = 0.0;
    double foot_lift_m = 0.06;
    StaircasePhase phase = StaircasePhase::kApproach;
};

inline StaircasePhase ClassifyStaircasePhase(
    const StaircaseSpec &spec,
    double base_x_m,
    double preview_distance_m) noexcept
{
    if (!Valid(spec) || !std::isfinite(base_x_m) ||
        !std::isfinite(preview_distance_m))
    {
        return StaircasePhase::kApproach;
    }
    const double end_x =
        spec.start_x_m + spec.tread_depth_m * static_cast<double>(spec.step_count);
    if (base_x_m + preview_distance_m < spec.start_x_m)
        return StaircasePhase::kApproach;
    if (base_x_m < end_x)
        return StaircasePhase::kClimb;
    return StaircasePhase::kExit;
}

inline bool PlanStaircaseReference(
    const StaircaseSpec &spec,
    double base_x_m,
    double preview_distance_m,
    double nominal_vx_mps,
    double nominal_step_length_m,
    double nominal_foot_lift_m,
    TerrainMotionReference &output) noexcept
{
    output = {};
    if (!Valid(spec) || !std::isfinite(base_x_m) ||
        !std::isfinite(preview_distance_m) || preview_distance_m < 0.0 ||
        !std::isfinite(nominal_vx_mps) || nominal_vx_mps < 0.0 ||
        !std::isfinite(nominal_step_length_m) || nominal_step_length_m <= 0.0 ||
        !std::isfinite(nominal_foot_lift_m) || nominal_foot_lift_m < 0.0)
    {
        return false;
    }
    output.phase = ClassifyStaircasePhase(spec, base_x_m, preview_distance_m);
    output.vx_mps = nominal_vx_mps;
    output.ground_height_m = spec.base_height_m;
    output.pitch_rad = 0.0;
    output.step_length_m = nominal_step_length_m;
    output.foot_lift_m = nominal_foot_lift_m;

    if (output.phase == StaircasePhase::kApproach)
        return true;

    const double stair_pitch = std::atan2(
        spec.riser_height_m, spec.tread_depth_m);
    const double end_x =
        spec.start_x_m + spec.tread_depth_m * static_cast<double>(spec.step_count);
    if (output.phase == StaircasePhase::kClimb)
    {
        const int step_index = std::clamp(
            static_cast<int>(std::floor(
                std::max(0.0, base_x_m - spec.start_x_m) /
                spec.tread_depth_m)),
            0,
            spec.step_count - 1);
        output.ground_height_m = spec.base_height_m +
            spec.riser_height_m * static_cast<double>(step_index + 1);
        output.pitch_rad = stair_pitch;
        output.vx_mps = std::min(nominal_vx_mps, 0.35);
        output.foot_lift_m = std::max(
            nominal_foot_lift_m, spec.riser_height_m + 0.04);
    }
    else
    {
        output.ground_height_m = spec.base_height_m +
            spec.riser_height_m * static_cast<double>(spec.step_count);
        output.vx_mps = std::min(nominal_vx_mps, 0.25);
        output.foot_lift_m = std::max(nominal_foot_lift_m, 0.07);
        output.pitch_rad = 0.0;
        (void)end_x;
    }
    return true;
}

struct TerrainMotionSlewLimits
{
    double vx_mps2 = 0.8;
    double ground_height_mps = 0.20;
    double pitch_radps = 0.80;
    double step_length_mps = 0.20;
    double foot_lift_mps = 0.30;
};

inline bool SlewTerrainMotionReference(
    const TerrainMotionReference &current,
    const TerrainMotionReference &target,
    const TerrainMotionSlewLimits &limits,
    double dt_s,
    TerrainMotionReference &output) noexcept
{
    if (!std::isfinite(dt_s) || dt_s <= 0.0 ||
        !std::isfinite(limits.vx_mps2) || limits.vx_mps2 <= 0.0 ||
        !std::isfinite(limits.ground_height_mps) ||
        limits.ground_height_mps <= 0.0 ||
        !std::isfinite(limits.pitch_radps) || limits.pitch_radps <= 0.0 ||
        !std::isfinite(limits.step_length_mps) ||
        limits.step_length_mps <= 0.0 ||
        !std::isfinite(limits.foot_lift_mps) || limits.foot_lift_mps <= 0.0)
    {
        return false;
    }
    output = current;
    const auto slew = [dt_s](double value, double target_value, double rate) {
        return value + std::clamp(
            target_value - value, -rate * dt_s, rate * dt_s);
    };
    output.vx_mps = slew(current.vx_mps, target.vx_mps, limits.vx_mps2);
    output.ground_height_m = slew(
        current.ground_height_m,
        target.ground_height_m,
        limits.ground_height_mps);
    output.pitch_rad = slew(current.pitch_rad, target.pitch_rad, limits.pitch_radps);
    output.step_length_m = slew(
        current.step_length_m,
        target.step_length_m,
        limits.step_length_mps);
    output.foot_lift_m = slew(
        current.foot_lift_m,
        target.foot_lift_m,
        limits.foot_lift_mps);
    output.phase = target.phase;
    return std::isfinite(output.vx_mps) &&
           std::isfinite(output.ground_height_m) &&
           std::isfinite(output.pitch_rad) &&
           std::isfinite(output.step_length_m) &&
           std::isfinite(output.foot_lift_m);
}

}  // namespace go2_control::terrain
