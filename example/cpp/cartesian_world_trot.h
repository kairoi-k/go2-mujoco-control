// World-frame no-slip stance + Cartesian swing for a running diagonal trot.
// Body-frame IK conveyor is not used: stance feet stay planted in world XY,
// swing is a world spline to a Raibert foothold. Speed comes from GRF and
// placement, not from assuming v_cmd in the stance sweep.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "go2_forward_kinematics.h"

namespace go2_control
{

struct RunningTrotSchedule
{
    double period_s = 0.60;
    double duty_factor = 0.75;
    double step_length_m = 0.091;
    double foot_lift_m = 0.022;
    double stance_time_s = 0.45;
};

// Keep v * T_stance inside the hip workspace (~0.22 m). Cadence and duty
// fall together so a 2 m/s trot is a short-stance running gait, not a
// stretched walking trot.
inline RunningTrotSchedule ScheduleRunningTrot(double v_cmd_mps)
{
    const double v = std::clamp(v_cmd_mps, 0.10, 2.60);
    RunningTrotSchedule out;
    if (v <= 0.30)
    {
        out.period_s = 0.40;
        out.duty_factor = 0.55;
        out.stance_time_s = out.duty_factor * out.period_s;
        out.step_length_m = v * out.period_s;
        out.foot_lift_m = 0.036;
        return out;
    }
    const double t = std::clamp((v - 0.30) / (2.00 - 0.30), 0.0, 1.0);
    out.duty_factor = 0.50 + t * (0.38 - 0.50);
    out.stance_time_s = std::clamp(0.10 / v, 0.09, 0.11);
    out.period_s = std::clamp(out.stance_time_s / out.duty_factor, 0.22, 0.45);
    out.stance_time_s = out.duty_factor * out.period_s;
    out.step_length_m = v * out.period_s;
    out.foot_lift_m = std::clamp(0.036 + 0.012 * t, 0.036, 0.052);
    return out;
}

inline double CartesianWorkspaceSpeedCap(double duty, double period_s)
{
    const double t_st = std::max(0.08, duty * period_s);
    return 0.24 / t_st;
}

inline go2::Vec3 RotateByQuat(
    const std::array<double, 4> &q,
    const go2::Vec3 &v)
{
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];
    return {
        (1.0 - 2.0 * (y * y + z * z)) * v.x +
            2.0 * (x * y - z * w) * v.y +
            2.0 * (x * z + y * w) * v.z,
        2.0 * (x * y + z * w) * v.x +
            (1.0 - 2.0 * (x * x + z * z)) * v.y +
            2.0 * (y * z - x * w) * v.z,
        2.0 * (x * z - y * w) * v.x +
            2.0 * (y * z + x * w) * v.y +
            (1.0 - 2.0 * (x * x + y * y)) * v.z};
}

inline go2::Vec3 WorldToBody(
    const go2::Vec3 &base,
    const std::array<double, 4> &quat_world_from_body,
    const go2::Vec3 &p_world)
{
    const std::array<double, 4> inv = {
        quat_world_from_body[0],
        -quat_world_from_body[1],
        -quat_world_from_body[2],
        -quat_world_from_body[3]};
    const go2::Vec3 delta = {
        p_world.x - base.x, p_world.y - base.y, p_world.z - base.z};
    return RotateByQuat(inv, delta);
}

inline go2::Vec3 BodyToWorld(
    const go2::Vec3 &base,
    const std::array<double, 4> &quat_world_from_body,
    const go2::Vec3 &p_body)
{
    const go2::Vec3 rotated = RotateByQuat(quat_world_from_body, p_body);
    return {base.x + rotated.x, base.y + rotated.y, base.z + rotated.z};
}

inline void ClampFootToHipWorkspace(go2::Leg leg, go2::Vec3 &p_body)
{
    const go2::LegGeometry g = go2::Geometry(leg);
    const double y0 = g.hip_y + g.hip_link_y;
    p_body.x = std::clamp(p_body.x, g.hip_x - 0.16, g.hip_x + 0.16);
    p_body.y = std::clamp(p_body.y, y0 - 0.06, y0 + 0.06);
    p_body.z = std::clamp(p_body.z, -0.42, -0.16);
}

inline double Quintic01(double x)
{
    const double t = std::clamp(x, 0.0, 1.0);
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

inline double Quintic01Dot(double x)
{
    const double t = std::clamp(x, 0.0, 1.0);
    return 30.0 * t * t * (t - 1.0) * (t - 1.0);
}

struct WorldTouchdownInput
{
    go2::Vec3 hip_world{};
    double vx_world = 0.0;
    double vy_world = 0.0;
    double vx_des_world = 0.0;
    double vy_des_world = 0.0;
    double stance_time_s = 0.12;
    double velocity_gain_s = 0.08;
    double max_adjustment_m = 0.14;
    double ground_z = 0.02;
};

inline go2::Vec3 PlanWorldTouchdown(const WorldTouchdownInput &in)
{
    const double t_st = std::max(0.05, in.stance_time_s);
    const double vx_mid = 0.65 * in.vx_des_world + 0.35 * in.vx_world;
    const double vy_mid = 0.65 * in.vy_des_world + 0.35 * in.vy_world;
    const double dx_nom = 0.5 * vx_mid * t_st;
    const double dy_nom = 0.5 * vy_mid * t_st;
    double dx_adj = in.velocity_gain_s * (in.vx_world - in.vx_des_world);
    double dy_adj = in.velocity_gain_s * (in.vy_world - in.vy_des_world);
    const double adj = std::hypot(dx_adj, dy_adj);
    if (adj > in.max_adjustment_m && adj > 1.0e-9)
    {
        const double s = in.max_adjustment_m / adj;
        dx_adj *= s;
        dy_adj *= s;
    }
    return {in.hip_world.x + dx_nom + dx_adj,
            in.hip_world.y + dy_nom + dy_adj,
            in.ground_z};
}

struct CartesianWorldState
{
    std::array<go2::Vec3, go2::kLegCount> stance_anchor_world{};
    std::array<go2::Vec3, go2::kLegCount> swing_start_world{};
    std::array<go2::Vec3, go2::kLegCount> swing_target_world{};
    std::array<go2::Vec3, go2::kLegCount> target_world{};
    std::array<go2::Vec3, go2::kLegCount> target_world_vel{};
    std::array<bool, go2::kLegCount> stance_valid{};
    std::array<bool, go2::kLegCount> in_stance{};
    std::array<bool, go2::kLegCount> prev_stance{};
    bool have_prev = false;
};

struct CartesianWorldInput
{
    go2::Vec3 base{};
    std::array<double, 4> quaternion{1.0, 0.0, 0.0, 0.0};
    std::array<go2::Vec3, go2::kLegCount> actual_world_feet{};
    std::array<go2::Vec3, go2::kLegCount> stand_body_feet{};
    double phase = 0.0;
    double duty_factor = 0.75;
    double period_s = 0.60;
    double v_cmd_mps = 0.15;
    double vx_world = 0.0;
    double vy_world = 0.0;
    double yaw_rad = 0.0;
    double roll_rad = 0.0;
    double gyro_x = 0.0;
    double raibert_gain_s = 0.08;
    double raibert_max_adj_m = 0.14;
    double foot_lift_m = 0.028;
    double blend = 1.0;
};

inline bool LegScheduledStance(std::size_t leg, double phase, double duty)
{
    const bool pair_b =
        leg == static_cast<std::size_t>(go2::Leg::FL) ||
        leg == static_cast<std::size_t>(go2::Leg::RR);
    double leg_phase = phase + (pair_b ? 0.5 : 0.0);
    leg_phase -= std::floor(leg_phase);
    if (leg_phase < 0.0)
        leg_phase += 1.0;
    return leg_phase < duty;
}

inline double LegSwingPhase(std::size_t leg, double phase, double duty)
{
    const bool pair_b =
        leg == static_cast<std::size_t>(go2::Leg::FL) ||
        leg == static_cast<std::size_t>(go2::Leg::RR);
    double leg_phase = phase + (pair_b ? 0.5 : 0.0);
    leg_phase -= std::floor(leg_phase);
    if (leg_phase < 0.0)
        leg_phase += 1.0;
    const double swing = 1.0 - duty;
    if (swing <= 1.0e-6 || leg_phase < duty)
        return 0.0;
    return std::clamp((leg_phase - duty) / swing, 0.0, 1.0);
}

inline go2::Vec3 HipWorld(
    const go2::Vec3 &base,
    const std::array<double, 4> &quat,
    go2::Leg leg)
{
    const go2::LegGeometry g = go2::Geometry(leg);
    return BodyToWorld(base, quat, {g.hip_x, g.hip_y, 0.0});
}

inline go2::Vec3 SwingWorldTarget(
    const go2::Vec3 &start,
    const go2::Vec3 &end,
    double swing_phase,
    double lift_m)
{
    const double s = Quintic01(std::min(1.0, swing_phase / 0.80));
    go2::Vec3 p = {
        start.x + (end.x - start.x) * s,
        start.y + (end.y - start.y) * s,
        start.z + (end.z - start.z) * s};
    const double z_phase = std::min(1.0, swing_phase / 0.85);
    p.z += lift_m * std::sin(3.14159265358979323846 * z_phase) *
           std::sin(3.14159265358979323846 * z_phase);
    return p;
}

inline go2::Vec3 SwingWorldVelocity(
    const go2::Vec3 &start,
    const go2::Vec3 &end,
    double swing_phase,
    double lift_m,
    double swing_duration_s)
{
    const double t_sw = std::max(0.05, swing_duration_s);
    const double dphase_dt = 1.0 / t_sw;
    const double u = std::min(1.0, swing_phase / 0.80);
    const double ds_dphase =
        swing_phase >= 0.80 ? 0.0 : Quintic01Dot(u) / 0.80;
    const double ds_dt = ds_dphase * dphase_dt;
    go2::Vec3 v = {
        (end.x - start.x) * ds_dt,
        (end.y - start.y) * ds_dt,
        (end.z - start.z) * ds_dt};
    const double z_phase = std::min(1.0, swing_phase / 0.85);
    const double dz_dphase = swing_phase >= 0.85 ? 0.0 : (1.0 / 0.85);
    constexpr double kPi = 3.14159265358979323846;
    v.z += lift_m * kPi * std::sin(2.0 * kPi * z_phase) * dz_dphase *
           dphase_dt;
    return v;
}

inline void ApplyCartesianWorldTrot(
    const CartesianWorldInput &in,
    CartesianWorldState &state,
    std::array<go2::Vec3, go2::kLegCount> &body_feet)
{
    const double yaw = in.yaw_rad;
    const double vx_des = in.v_cmd_mps * std::cos(yaw);
    const double vy_des = in.v_cmd_mps * std::sin(yaw);
    const double t_st = std::max(0.05, in.duty_factor * in.period_s);
    const double blend = std::clamp(in.blend, 0.0, 1.0);

    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const bool stance = LegScheduledStance(leg, in.phase, in.duty_factor);
        const bool entering_stance = stance && (!state.have_prev || !state.prev_stance[leg]);
        const bool entering_swing = !stance && (!state.have_prev || state.prev_stance[leg]);

        if (entering_stance || (stance && !state.stance_valid[leg]))
        {
            state.stance_anchor_world[leg] = in.actual_world_feet[leg];
            state.stance_valid[leg] = true;
        }
        if (entering_swing)
        {
            state.swing_start_world[leg] = state.stance_valid[leg]
                ? state.stance_anchor_world[leg]
                : in.actual_world_feet[leg];
            WorldTouchdownInput td;
            td.hip_world = HipWorld(
                in.base, in.quaternion, static_cast<go2::Leg>(leg));
            td.vx_world = in.vx_world;
            td.vy_world = in.vy_world;
            td.vx_des_world = vx_des;
            td.vy_des_world = vy_des;
            td.stance_time_s = t_st;
            td.velocity_gain_s = in.raibert_gain_s;
            td.max_adjustment_m = in.raibert_max_adj_m;
            td.ground_z = state.swing_start_world[leg].z;
            // Capture-point: step into a roll so GRF rights the body.
            td.vy_world += 0.22 * in.roll_rad / std::max(0.08, t_st) +
                           0.08 * in.gyro_x;
            state.swing_target_world[leg] = PlanWorldTouchdown(td);
            const bool left =
                leg == static_cast<std::size_t>(go2::Leg::FL) ||
                leg == static_cast<std::size_t>(go2::Leg::RL);
            const double widen =
                0.014 * std::clamp(in.v_cmd_mps / 0.80, 0.0, 1.4);
            state.swing_target_world[leg].y += left ? widen : -widen;
            state.stance_valid[leg] = false;
        }
        else if (!stance)
        {
            WorldTouchdownInput td;
            td.hip_world = HipWorld(
                in.base, in.quaternion, static_cast<go2::Leg>(leg));
            td.vx_world = in.vx_world;
            td.vy_world = in.vy_world +
                          0.22 * in.roll_rad / std::max(0.08, t_st) +
                          0.08 * in.gyro_x;
            td.vx_des_world = vx_des;
            td.vy_des_world = vy_des;
            td.stance_time_s = t_st;
            td.velocity_gain_s = in.raibert_gain_s;
            td.max_adjustment_m = in.raibert_max_adj_m;
            td.ground_z = state.swing_start_world[leg].z;
            state.swing_target_world[leg] = PlanWorldTouchdown(td);
            {
                const bool left =
                    leg == static_cast<std::size_t>(go2::Leg::FL) ||
                    leg == static_cast<std::size_t>(go2::Leg::RL);
                const double widen =
                    0.014 * std::clamp(in.v_cmd_mps / 0.80, 0.0, 1.4);
                state.swing_target_world[leg].y += left ? widen : -widen;
            }
        }

        go2::Vec3 p_world = in.actual_world_feet[leg];
        go2::Vec3 v_world{0.0, 0.0, 0.0};
        if (stance && state.stance_valid[leg])
        {
            p_world = state.stance_anchor_world[leg];
        }
        else if (!stance)
        {
            const double swing_phase =
                LegSwingPhase(leg, in.phase, in.duty_factor);
            p_world = SwingWorldTarget(
                state.swing_start_world[leg],
                state.swing_target_world[leg],
                swing_phase,
                in.foot_lift_m);
            v_world = SwingWorldVelocity(
                state.swing_start_world[leg],
                state.swing_target_world[leg],
                swing_phase,
                in.foot_lift_m,
                std::max(0.05, (1.0 - in.duty_factor) * in.period_s));
        }
        state.target_world[leg] = p_world;
        state.target_world_vel[leg] = v_world;
        state.in_stance[leg] = stance;
        state.prev_stance[leg] = stance;

        go2::Vec3 p_body = WorldToBody(in.base, in.quaternion, p_world);
        ClampFootToHipWorkspace(static_cast<go2::Leg>(leg), p_body);
        body_feet[leg].x =
            (1.0 - blend) * in.stand_body_feet[leg].x + blend * p_body.x;
        body_feet[leg].y =
            (1.0 - blend) * in.stand_body_feet[leg].y + blend * p_body.y;
        body_feet[leg].z =
            (1.0 - blend) * in.stand_body_feet[leg].z + blend * p_body.z;
    }
    state.have_prev = true;
}

}  // namespace go2_control
