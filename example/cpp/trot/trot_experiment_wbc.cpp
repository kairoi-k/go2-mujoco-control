#include "trot_experiment.h"
#include "trot_true_dynamics.h"
#include "full2_campaign_env.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include "centroidal_wbc.h"
#include "contact_wrench_lexicographic_allocator.h"
#include "contact_wrench_projected_allocator.h"
#include "contact_wrench_qp.h"
#include "contact_state_filter.h"
#include "go2_contact_torque_mapping.h"
#include "go2_inverse_kinematics.h"
#include "inverse_dynamics_wbc.h"
#include "motion_frame_utils.h"
#include "preview_footstep_horizon.h"
#include "srbd_mpc.h"

using namespace unitree::common;
using namespace unitree::robot;
using namespace go2_trot;

namespace
{

go2_control::RigidBodyState MakeRigidBodyState(
    const unitree_go::msg::dds_::LowState_ &low,
    const unitree_go::msg::dds_::SportModeState_ &high,
    const Eigen::Vector3d &linear_vel_world)
{
    const WorldPose pose = ComputeWorldPose(low, high);
    go2_control::RigidBodyState state;
    state.position_world = Eigen::Vector3d(pose.base.x, pose.base.y, pose.base.z);
    state.quat_world_from_body = Eigen::Quaterniond(
        pose.quaternion[0], pose.quaternion[1],
        pose.quaternion[2], pose.quaternion[3]);
    state.linear_vel_world = linear_vel_world;
    state.angular_vel_body = Eigen::Vector3d(
        low.imu_state().gyroscope()[0],
        low.imu_state().gyroscope()[1],
        low.imu_state().gyroscope()[2]);
    for (int i = 0; i < kMotorCount; ++i)
    {
        state.q[i] = low.motor_state()[i].q();
        state.dq[i] = low.motor_state()[i].dq();
    }
    return state;
}

Eigen::Vector3d ClampVec3(const Eigen::Vector3d &v, double lim)
{
    Eigen::Vector3d out = v;
    for (int i = 0; i < 3; ++i)
        out[i] = std::clamp(out[i], -lim, lim);
    return out;
}

// The nominal gait phase is only a prediction.  At sprint cadence a foot can
// touch down a few milliseconds early/late, so a QP contact set made solely
// from the phase can ask ID-WBC to support a leg that is in flight, or to swing
// a leg that is already carrying load.  Keep this opt-in for experiments: use
// the hysteretic force state when it is trustworthy, but retain scheduled
// contacts until at least a diagonal pair is available so a noisy force sample
// cannot make the floating-base problem underconstrained.
std::array<bool, go2::kLegCount> MergeHighSpeedContact(
    const std::array<bool, go2::kLegCount> &scheduled,
    const std::array<bool, go2::kLegCount> &measured,
    int mode)
{
    if (mode <= 0)
        return scheduled;
    // Mode 2 is a conservative union: never remove a scheduled stance foot
    // merely because its force sensor is late, but allow an early measured
    // touchdown to enter the QP immediately.  This keeps the contact plant
    // compatible with the planned swing trajectory while absorbing early
    // touchdown, unlike the old measured-only merge which could replace a
    // valid diagonal pair with a transient one-leg sample.
    if (mode >= 2)
    {
        std::array<bool, go2::kLegCount> merged = scheduled;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            merged[leg] = merged[leg] || measured[leg];
        return merged;
    }
    std::array<bool, go2::kLegCount> merged = measured;
    int active = 0;
    for (bool contact : merged)
        active += contact ? 1 : 0;
    if (active < 2)
    {
        for (std::size_t leg = 0; leg < go2::kLegCount && active < 2; ++leg)
        {
            if (!scheduled[leg] || merged[leg])
                continue;
            merged[leg] = true;
            ++active;
        }
    }
    return merged;
}

}  // namespace

void TrotExperiment::UpdateWbcFull(
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot)
{
    wbc_shadow_diagnostics_.enabled = true;
    wbc_shadow_contact_state_valid_ = false;
    if (!rigid_body_ || !rigid_body_->loaded())
        return;
    const double pitch_abs = std::abs(
        static_cast<double>(state_snapshot.imu_state().rpy()[1]));
    const double roll_abs = std::abs(
        static_cast<double>(state_snapshot.imu_state().rpy()[0]));
    const double attitude_fade = std::clamp(
        1.0 - std::max(0.0, std::max(pitch_abs, roll_abs) - 0.06) / 0.10,
        0.0, 1.0);
    const double v_body = have_filtered_body_velocity_
        ? std::abs(latest_filtered_body_velocity_[0])
        : 0.0;
    const bool high_speed_curriculum =
        Full2EnvDouble("TROT_HS_DISABLE", 0.0) <= 0.5 &&
        ((!params_.runtime_velocity_command &&
         std::abs(wbc_speed_cmd_mps_) > 1.25 ||
         std::abs(kernel_nominal_velocity_x_mps_) > 1.25));
    const double cycle_lock = params_.cartesian_world
        ? Smoothstep((static_cast<double>(completed_cycles_) - 8.0) / 8.0)
        : Smoothstep((static_cast<double>(completed_cycles_) - 24.0) / 20.0);
    const double force_blend = UpdateCartesianForceBlend();
    const double tst_gate = params_.cartesian_world
        ? Smoothstep((0.40 - cartesian_stance_s_) / 0.12)
        : 1.0;
    double cart_lock = 0.0;
    if (params_.cartesian_world)
    {
        if (Full2EnvDouble("FULL2_NO_LOCK", 0.0) > 0.5)
            cart_lock = 0.0;
        else if (Full2EnvDouble("FULL2_LOCK_DA84", 0.0) > 0.5)
        {
            // 233312: min(cycle, speed) * attitude. Current G2 multiplies
            // force_blend * tst_gate and locks from cycle 8.
            const double speed_lock = Smoothstep((v_body - 0.40) / 0.60);
            const double cycle_lock_da84 =
                Smoothstep((static_cast<double>(completed_cycles_) - 24.0) /
                           20.0);
            cart_lock = std::min(cycle_lock_da84, speed_lock) * attitude_fade;
        }
        else
            cart_lock = cycle_lock * attitude_fade * force_blend * tst_gate;
    }
    Eigen::Vector3d linear_vel_world(
        high_state_snapshot.velocity()[0],
        high_state_snapshot.velocity()[1],
        high_state_snapshot.velocity()[2]);
    go2_control::RigidBodyDynamics dyn;
    if (!rigid_body_->Evaluate(
            MakeRigidBodyState(
                state_snapshot, high_state_snapshot, linear_vel_world),
            dyn))
    {
        return;
    }
    if (!dynamics_logged_)
    {
        dynamics_logged_ = true;
        std::cout << "WBC-FULL rigid body mass=" << dyn.mass_kg
                  << " com_z=" << dyn.com_world.z()
                  << " bias_z=" << dyn.bias[2] << "\n";
    }

    std::array<bool, go2::kLegCount> measured_contact{};
    const go2_control::HystereticContactParams contact_params{
        kShadowContactOnForceN, kShadowContactOffForceN};
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const double force = state_snapshot.foot_force()[leg];
        bool next_contact = false;
        if (!go2_control::UpdateHystereticContact(
                wbc_shadow_contact_state_[leg], force, contact_params,
                next_contact))
            return;
        wbc_shadow_contact_state_[leg] = next_contact;
        measured_contact[leg] = next_contact;
    }
    wbc_shadow_contact_state_valid_ = true;
    const double terrain_now_s =
        static_cast<double>(state_snapshot.tick()) * 1.0e-3;
    const auto terrain_contact_plan =
        params_.terrain_actuation && !params_.terrain_sensor_only
            ? (terrain_execution_plan_ &&
                       terrain_execution_plan_->usable_at(terrain_now_s)
                   ? terrain_execution_plan_
                   : terrain_plan_store_.LoadUsable(terrain_now_s))
            : nullptr;
    const int high_speed_contact_merge_mode = high_speed_curriculum
        ? std::clamp(static_cast<int>(std::llround(Full2EnvDouble(
              "TROT_HS_HYBRID_CONTACT", 0.0))), 0, 2)
        : 0;
    // During ordinary trot the force sensors stay high through lift-off, so
    // the validated path uses the gait schedule.  Sprint experiments may opt
    // into a measured/scheduled merge to absorb early or late touchdown.
    std::array<bool, go2::kLegCount> qp_contact = measured_contact;
    std::array<bool, go2::kLegCount> scheduled_contact = measured_contact;
    const double gait_period =
        kernel_period_s_ > 0.05 ? kernel_period_s_ : params_.period_s;
    const double gait_duty =
        kernel_duty_factor_ > 0.05 ? kernel_duty_factor_ : params_.duty_factor;
    const bool gait_contact_schedule_active =
        task_.gait_started_ &&
        (task_.motion_stage_ == 2 || WbcStopHoldActive());
    if (gait_contact_schedule_active)
    {
        std::array<std::array<bool, go2::kLegCount>,
                   go2_control::kSrbdMaxHorizon> scheduled{};
        go2_control::FillTrotContactSchedulePhase(
            current_phase_, gait_period, gait_duty, 1, 0.0, scheduled,
            params_.gait_pattern);
        scheduled_contact = scheduled[0];
        if (terrain_contact_plan &&
            terrain_contact_plan->contact_schedule.valid(
                terrain_contact_plan->horizon_knots))
        {
            std::array<std::size_t, go2_terrain::kTerrainPlanMaxKnots>
                plan_contact_index{};
            if (go2_terrain::BuildTerrainPlanHorizonIndices(
                    *terrain_contact_plan, terrain_now_s,
                    terrain_planner_.config().knot_dt_s,
                    terrain_planner_.config().knot_dt_s, 1,
                    plan_contact_index))
                scheduled_contact = terrain_contact_plan->contact_schedule
                    .planned_contact[plan_contact_index[0]];
        }
    }
    // During the brake, keep the scheduled running contacts.  The body is
    // still carrying sprint momentum, so declaring all four feet fixed after
    // an arbitrary 0.40 s creates the same hidden plant switch we are trying
    // to avoid.  The brake-complete gate promotes this to a four-contact WBC
    // hold only once measured speed and attitude are both safe.
    const bool high_speed_stop_support = high_speed_stop_hold_active_;
    if (gait_contact_schedule_active)
    {
        if (WbcStopHoldActive() || high_speed_stop_support)
            qp_contact.fill(true);
        else if (motion_event_response_enabled_ && EmergencyStopHoldReady())
            qp_contact.fill(true);
        else
        {
            qp_contact = MergeHighSpeedContact(
                scheduled_contact, measured_contact,
                high_speed_contact_merge_mode);
        }
    }

    // A terrain foothold is not a support contact at the predicted gait
    // boundary.  Keep the support set that was loaded when the sensor-derived
    // swing started, then promote a target only after the live force filter
    // and the live foot pose agree at the immutable endpoint.  This preserves
    // the planned/measured distinction while allowing the fixed gait cadence
    // to wait for a real touchdown instead of dropping its old support pair.
    bool terrain_transfer_has_target = false;
    bool terrain_transfer_complete = true;
    if (terrain_surface_transition_active_ &&
        terrain_transfer_hold_active_)
    {
        terrain_transfer_has_target = true;
        terrain_transfer_complete = false;
    }
    const bool terrain_execution_pending = std::any_of(
        terrain_swing_execution_.begin(), terrain_swing_execution_.end(),
        [](const TerrainSwingExecution &execution) {
            return execution.valid && execution.terrain_target_required &&
                !execution.measured_touchdown;
        });
    if (!terrain_surface_transition_active_ &&
        terrain_transfer_hold_active_ &&
        !terrain_transfer_has_target && !terrain_execution_pending)
    {
        if (scheduled_contact == terrain_transfer_hold_contact_)
        {
            terrain_transfer_hold_contact_.fill(false);
            terrain_transfer_hold_active_ = false;
        }
        else
        {
            terrain_transfer_has_target = true;
            terrain_transfer_complete = false;
        }
    }
    const double terrain_touchdown_tolerance_m = std::max(
        0.020,
        1.5 * terrain_planner_.config().feasibility.foot_patch_radius_m);
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        auto &execution = terrain_swing_execution_[leg];
        if (!execution.valid || !execution.terrain_target_required ||
            execution.measured_touchdown ||
            (!execution.in_flight && !execution.endpoint_held))
            continue;
        terrain_transfer_has_target = true;
        if (!execution.endpoint_held)
        {
            terrain_transfer_complete = false;
            continue;
        }
        const Eigen::Vector3d endpoint_error =
            dyn.foot_pos_world[leg] - Eigen::Vector3d(
                execution.target_world.x,
                execution.target_world.y,
                execution.target_world.z);
        execution.wbc_endpoint_error_m = endpoint_error.allFinite()
            ? endpoint_error.norm()
            : std::numeric_limits<double>::infinity();
        const bool at_endpoint = endpoint_error.allFinite() &&
            endpoint_error.norm() <= terrain_touchdown_tolerance_m;
        execution.wbc_at_endpoint = at_endpoint;
        execution.wbc_measured_contact = measured_contact[leg];
        if (measured_contact[leg] && at_endpoint)
        {
            execution.measured_touchdown = true;
            if (terrain_surface_transition_active_ &&
                terrain_surface_transition_required_[leg])
            {
                terrain_surface_transition_committed_[leg] = true;
                terrain_transfer_complete = false;
            }
        }
        else
            terrain_transfer_complete = false;
    }

    if (!terrain_transfer_has_target)
    {
        terrain_transfer_hold_contact_.fill(false);
        terrain_transfer_hold_active_ = false;
    }
    else
    {
        if (!terrain_transfer_hold_active_)
        {
            terrain_transfer_hold_contact_ = scheduled_contact;
            int scheduled_support_count = 0;
            for (bool contact : terrain_transfer_hold_contact_)
                scheduled_support_count += contact ? 1 : 0;
            if (scheduled_support_count < 2)
            {
                terrain_transfer_hold_contact_ = qp_contact;
                scheduled_support_count = 0;
                for (bool contact : terrain_transfer_hold_contact_)
                    scheduled_support_count += contact ? 1 : 0;
            }
            terrain_transfer_hold_active_ = scheduled_support_count >= 2;
        }
        if (terrain_transfer_hold_active_ && !terrain_transfer_complete)
        {
            // Keep the support captured at the first target boundary;
            // only a confirmed target may be removed from that support
            // set while its terrain swing is in flight. This keeps a
            // later target from replacing the transaction's support pair.
            qp_contact = terrain_transfer_hold_contact_;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                const auto &execution = terrain_swing_execution_[leg];
                const bool active_target = execution.valid &&
                    execution.terrain_target_required &&
                    !execution.measured_touchdown &&
                    (execution.in_flight || execution.endpoint_held);
                if (active_target)
                    qp_contact[leg] = false;
                if (execution.measured_touchdown)
                    qp_contact[leg] = true;
        }
        }
        else if (terrain_transfer_complete)
        {
            // The measured target is now a real support contact. Promote the
            // same retimed plan set on this tick instead of spending one
            // cycle on the stale transfer-hold mask.
            qp_contact = scheduled_contact;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                if (terrain_swing_execution_[leg].measured_touchdown)
                    qp_contact[leg] = true;
            }
            terrain_transfer_hold_contact_.fill(false);
            terrain_transfer_hold_active_ = false;
        }
    }

    bool terrain_surface_transition_complete =
        terrain_surface_transition_active_;
    if (terrain_surface_transition_active_)
    {
        int required_mask = 0;
        int committed_mask = 0;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (terrain_surface_transition_required_[leg])
                required_mask |= 1 << static_cast<int>(leg);
            if (terrain_surface_transition_committed_[leg])
            {
                committed_mask |= 1 << static_cast<int>(leg);
                if (measured_contact[leg])
                    qp_contact[leg] = true;
            }
            if (terrain_surface_transition_required_[leg] &&
                !terrain_surface_transition_committed_[leg])
                terrain_surface_transition_complete = false;
        }
        if (terrain_surface_transition_complete)
        {
            terrain_surface_transition_last_required_mask_ = required_mask;
            terrain_surface_transition_last_committed_mask_ = committed_mask;
            ++terrain_surface_transition_completions_;
            terrain_surface_transition_active_ = false;
            terrain_surface_transition_required_.fill(false);
            terrain_surface_transition_committed_.fill(false);
            terrain_surface_transition_source_valid_.fill(false);
        }
    }

    const auto contact_mask_for = [](
        const std::array<bool, go2::kLegCount> &contact) {
        int mask = 0;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            if (contact[leg])
                mask |= 1 << static_cast<int>(leg);
        return mask;
    };
    int contact_mask = 0;
    int active = 0;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (qp_contact[leg])
        {
            ++active;
            contact_mask |= 1 << static_cast<int>(leg);
        }
    }
    wbc_shadow_diagnostics_.active_contacts = active;
    wbc_shadow_diagnostics_.contact_mask = contact_mask;
    wbc_shadow_diagnostics_.measured_contact_mask =
        contact_mask_for(measured_contact);
    wbc_shadow_diagnostics_.scheduled_contact_mask =
        contact_mask_for(scheduled_contact);

    go2_control::SrbdMpcParams mpc_params;
    mpc_params.horizon = 8;
    mpc_params.dt_s = std::clamp(gait_period / 8.0, 0.020, 0.05);
    mpc_params.mass_kg = dyn.mass_kg;
    mpc_params.inertia_com_world = dyn.inertia_com_world;
    mpc_params.friction_mu = kShadowWbcFrictionCoefficient;
    if (params_.cartesian_world)
    {
        mpc_params.w_vel_xy = 80.0 + 40.0 * cart_lock + 40.0 * force_blend * tst_gate;
        mpc_params.w_pos_xy = 6.0;
        mpc_params.w_ori = 120.0 + 30.0 * force_blend * tst_gate;
        mpc_params.w_vel_z = 12.0;
        mpc_params.w_omega = 8.0;
            mpc_params.w_force = 3.0e-4;
            // 121733: any paper-kp force_track/weight slew sagged in
            // ~10 cycles. 105123 held 0.35 with these MPC weights
            // unchanged. Transmit GRF later, after a hold.
    }
    else if (!params_.step_plan.empty())
    {
        mpc_params.w_vel_xy = 80.0;
        mpc_params.w_pos_xy = 20.0;
    }
    if (high_speed_curriculum)
    {
        // The ordinary trot weights are intentionally conservative.  Keep
        // the release defaults unchanged, but allow a sprint campaign to
        // make the preview layer value velocity tracking enough to request
        // the reaction force needed for a real acceleration, rather than
        // leaving the whole burden to the one-step ID-WBC task.
        const double hs_mpc_w_vel = Full2EnvDouble(
            "TROT_HS_MPC_W_VEL", -1.0);
        const double hs_mpc_w_force_xy = Full2EnvDouble(
            "TROT_HS_MPC_W_FORCE_XY", -1.0);
        const double hs_mpc_w_ori = Full2EnvDouble(
            "TROT_HS_MPC_W_ORI", -1.0);
        if (hs_mpc_w_vel > 0.0)
            mpc_params.w_vel_xy = hs_mpc_w_vel;
        if (hs_mpc_w_force_xy >= 0.0)
            mpc_params.w_force_trot_xy = hs_mpc_w_force_xy;
        if (hs_mpc_w_ori > 0.0)
            mpc_params.w_ori = hs_mpc_w_ori;
    }
    const bool straight_line_reference =
        params_.world_feedback && have_world_reference_ &&
        !task_.goal_enabled_ &&
        std::abs(params_.turn_rate_radps) < 1.0e-4 &&
        (!motion_event_response_enabled_ ||
         std::abs(motion_reference_.yaw_rate_radps) < 1.0e-4);
    // At the validated trot cadence, 20 Hz MPC is sufficient.  A sprint
    // cycle can be shorter than that 50 ms hold, so reuse of the old SRBD
    // force plan becomes a visible phase lag.  Refresh at 100 Hz for the
    // high-speed plant while keeping the established rates elsewhere.
    const int mpc_period_ticks = high_speed_curriculum
        ? 5
        : (params_.cartesian_world ? 10 : 25);
    const auto terrain_plan = terrain_contact_plan;
    std::array<std::size_t, go2_terrain::kTerrainPlanMaxKnots>
        terrain_plan_knot{};
    bool terrain_plan_contact_coherent = true;
    const bool terrain_plan_active =
        terrain_plan && task_.gait_started_ && task_.motion_stage_ == 2;
    if (terrain_plan_active)
    {
        terrain_plan_contact_coherent =
            terrain_plan->contact_schedule.valid(
                terrain_plan->horizon_knots) &&
            go2_terrain::BuildTerrainPlanHorizonIndices(
                *terrain_plan, terrain_now_s,
                terrain_planner_.config().knot_dt_s,
                mpc_params.dt_s,
                static_cast<std::size_t>(mpc_params.horizon),
                terrain_plan_knot);
        if (!terrain_plan_contact_coherent)
            ++terrain_plan_contact_rejections_;
    }
    wbc_shadow_diagnostics_.terrain_plan_id =
        terrain_plan_active ? terrain_plan->plan_id : 0;
    wbc_shadow_diagnostics_.terrain_contact_coherent =
        terrain_plan_active && terrain_plan_contact_coherent;
    wbc_shadow_diagnostics_.terrain_planned_contact_mask =
        terrain_plan_active && terrain_plan_contact_coherent
            ? contact_mask_for(
                  terrain_plan->contact_schedule.planned_contact[
                      terrain_plan_knot[0]])
            : 0;
    const bool run_mpc =
        (wbc_full_ticks_ % mpc_period_ticks) == 0 || !last_srbd_.ok;
    if (run_mpc)
    {
        go2_control::SrbdMpcInput mpc_in;
        mpc_in.state[0] = state_snapshot.imu_state().rpy()[0];
        mpc_in.state[1] = state_snapshot.imu_state().rpy()[1];
        mpc_in.state[2] = state_snapshot.imu_state().rpy()[2];
        mpc_in.state[3] = dyn.com_world.x();
        mpc_in.state[4] = dyn.com_world.y();
        mpc_in.state[5] = dyn.com_world.z();
        mpc_in.state[6] = state_snapshot.imu_state().gyroscope()[0];
        mpc_in.state[7] = state_snapshot.imu_state().gyroscope()[1];
        mpc_in.state[8] = state_snapshot.imu_state().gyroscope()[2];
        mpc_in.state[9] = linear_vel_world.x();
        mpc_in.state[10] = linear_vel_world.y();
        mpc_in.state[11] = linear_vel_world.z();
        mpc_in.reference = mpc_in.state;
        mpc_in.reference[0] = 0.0;
        mpc_in.reference[1] = 0.0;
        mpc_in.reference[4] = 0.0;
        const double base_height_ref =
            (high_speed_curriculum &&
             Full2EnvDouble("TROT_HS_BASE_HEIGHT", -1.0) > 0.0)
                ? std::clamp(
                      Full2EnvDouble("TROT_HS_BASE_HEIGHT", -1.0),
                      0.32, 0.48)
                : kWbcPrimaryBaseHeightM;
        mpc_in.reference[5] = base_height_ref;
        mpc_in.reference[6] = 0.0;
        mpc_in.reference[7] = 0.0;
        mpc_in.reference[8] = motion_event_response_enabled_
            ? motion_reference_.yaw_rate_radps
            : (task_.goal_enabled_
            ? task_.TurnEnable(running_time_) * task_.commanded_turn_rate_radps_
            : params_.turn_rate_radps);
        const double v_cmd =
            task_.gait_started_ && task_.motion_stage_ == 2
                ? (std::isfinite(kernel_nominal_velocity_x_mps_) &&
                           std::abs(kernel_nominal_velocity_x_mps_) > 1.0e-6
                       ? kernel_nominal_velocity_x_mps_
                       : (params_.runtime_velocity_command
                              ? params_.direction_sign *
                                    velocity_command_state_.applied_mps
                              : params_.direction_sign *
                                    params_.step_length_m /
                                    params_.period_s))
                : 0.0;
        const double yaw =
            static_cast<double>(state_snapshot.imu_state().rpy()[2]);
        if (params_.cartesian_world)
        {
            mpc_in.reference[2] = world_reference_yaw_rad_;
            mpc_in.reference[4] = world_reference_y_m_;
            const double yaw_err = WrapAngle(
                yaw - world_reference_yaw_rad_);
            mpc_in.reference[8] = Clamp(-1.2 * yaw_err, -0.30, 0.30);
            double v_ref = v_cmd;
            if (cartesian_kp_frozen_ && cartesian_latched_kp_ <= 5.0)
            {
                // 105123 MPC+0.35 still asked ~74 N because w_vel_xy
                // was 160; ground GRF x mean 0 N. Lead 0.20 with
                // w_vel_xy=40 is a moderate hole; WBIC applies it.
                v_ref = std::copysign(
                    std::min(std::abs(v_cmd), v_body + 0.35), v_cmd);
            }
            else if (force_blend > 0.05 && tst_gate > 0.30 &&
                v_body > v_cmd && v_body < 2.20)
                v_ref = v_body;
            mpc_in.reference[9] =
                v_ref * std::cos(world_reference_yaw_rad_);
            mpc_in.reference[10] =
                v_ref * std::sin(world_reference_yaw_rad_);
            mpc_in.reference[3] =
                dyn.com_world.x() + mpc_in.reference[9] * mpc_params.dt_s *
                                        0.5 * mpc_params.horizon;
        }
        else
        {
            if (straight_line_reference)
            {
                const double yaw_error = WrapAngle(
                    yaw - world_reference_yaw_rad_);
                mpc_in.reference[2] = world_reference_yaw_rad_;
                mpc_in.reference[4] = world_reference_y_m_;
                const double straight_yaw_gain = std::clamp(
                    Full2EnvDouble("TROT_STRAIGHT_YAW_GAIN", 2.0),
                    0.0, 5.0);
                mpc_in.reference[8] = Clamp(
                    -straight_yaw_gain * yaw_error, -0.30, 0.30);
            }
            double v_ref = v_cmd;
            if (task_.goal_enabled_ && !task_.reached_goal_)
            {
                const WorldPose pose =
                    ComputeWorldPose(state_snapshot, high_state_snapshot);
                v_ref *= task_.CommandedStepScale(pose.base.x, pose.base.y);
                const double path_yaw = std::atan2(
                    task_.goal_y_ - world_reference_y_m_,
                    task_.goal_x_ - world_reference_x_m_);
                mpc_in.reference[2] = path_yaw;
                mpc_in.reference[4] = dyn.com_world.y();
                mpc_in.reference[8] = Clamp(
                    -1.2 * WrapAngle(yaw - path_yaw), -0.30, 0.30);
                mpc_in.reference[9] = v_ref * std::cos(path_yaw);
                mpc_in.reference[10] = v_ref * std::sin(path_yaw);
            }
            else
            {
                mpc_in.reference[9] = v_ref;
                mpc_in.reference[10] = motion_event_response_enabled_
                    ? motion_reference_.vy_mps : 0.0;
            }
        }
        mpc_in.reference[11] = 0.0;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            mpc_in.foot_from_com_world[leg] =
                dyn.foot_pos_world[leg] - dyn.com_world;
        if (task_.gait_started_ &&
            (task_.motion_stage_ == 2 || WbcStopHoldActive()))
        {
            if (WbcStopHoldActive() || high_speed_stop_support)
                for (int k = 0; k < mpc_params.horizon; ++k)
                    mpc_in.contact[k].fill(true);
            else if (motion_event_response_enabled_ && EmergencyStopHoldReady())
                for (int k = 0; k < mpc_params.horizon; ++k)
                    mpc_in.contact[k].fill(true);
            else
            {
                go2_control::FillTrotContactSchedulePhase(
                    current_phase_, gait_period, gait_duty,
                    mpc_params.horizon, mpc_params.dt_s, mpc_in.contact,
                    params_.gait_pattern);
                if ((high_speed_contact_merge_mode > 0 ||
                     terrain_transfer_hold_active_ ||
                     terrain_surface_transition_active_) &&
                    mpc_params.horizon > 0)
                    mpc_in.contact[0] = qp_contact;
            }
        }
        else
        {
            for (int k = 0; k < mpc_params.horizon; ++k)
                mpc_in.contact[k] = qp_contact;
        }

        if (terrain_plan && terrain_plan_contact_coherent &&
            task_.gait_started_ &&
            task_.motion_stage_ == 2 && !WbcStopHoldActive())
        {
            // The accepted planner snapshot is the sole source for future
            // terrain contacts.  A partial snapshot is rejected rather than
            // mixed with the legacy current-foot anchor.
            const auto fallback_mpc_contact = mpc_in.contact;
            bool complete_foot_horizon = true;
            const WorldPose current_pose = ComputeWorldPose(
                state_snapshot, high_state_snapshot);
            std::array<bool, go2::kLegCount> active_transfer_target{};
            std::array<bool, go2::kLegCount> effective_transfer_hold{};
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                const auto &execution = terrain_swing_execution_[leg];
                active_transfer_target[leg] = execution.valid &&
                    execution.terrain_target_required &&
                    !execution.measured_touchdown &&
                    (execution.in_flight || execution.endpoint_held);
                effective_transfer_hold[leg] =
                    terrain_surface_transition_active_ &&
                    terrain_surface_transition_committed_[leg] &&
                    measured_contact[leg];
                if (terrain_transfer_hold_active_ &&
                    terrain_transfer_has_target &&
                    !terrain_transfer_complete)
                    effective_transfer_hold[leg] =
                        effective_transfer_hold[leg] ||
                        terrain_transfer_hold_contact_[leg];
            }
            for (int k = 0; k < mpc_params.horizon; ++k)
            {
                const std::size_t plan_knot =
                    terrain_plan_knot[static_cast<std::size_t>(k)];
                mpc_in.contact[k] = terrain_plan->contact_schedule.planned_contact[
                    plan_knot];
                if (terrain_surface_transition_active_ ||
                    (terrain_transfer_hold_active_ &&
                     terrain_transfer_has_target &&
                     !terrain_transfer_complete))
                {
                    // The nominal plan is a prediction.  While a terrain
                    // target is still seeking measured touchdown, keep the
                    // loaded support anchors in the preview and do not let
                    // the nominal contact boundary inject unsupported force
                    // into the current transfer.  Once measured touchdown
                    // is promoted, the atomic plan regains ownership of the
                    // future knots.
                    mpc_in.contact[k] =
                        go2_terrain::TerrainTransferPreviewContact(
                            mpc_in.contact[k],
                            effective_transfer_hold,
                            active_transfer_target);
                }
                if (k == 0 && (terrain_transfer_hold_active_ ||
                               terrain_surface_transition_active_))
                    mpc_in.contact[k] = qp_contact;
                mpc_in.reference_horizon[static_cast<std::size_t>(k)] =
                    mpc_in.reference;
                const auto &body = terrain_plan->body_reference[
                    plan_knot];
                if (body.valid)
                {
                    // Horizontal speed remains owned by the approved Phase-1
                    // v_cmd path already present in mpc_in.reference.  The
                    // planner's measured-state extrapolation is not a second
                    // longitudinal velocity authority.  Stage B consumes the
                    // terrain-conditioned vertical body reference here.
                    mpc_in.reference_horizon[static_cast<std::size_t>(k)][5] =
                        mpc_in.reference[5] +
                        body.position.z - current_pose.base.z;
                    mpc_in.reference_horizon[static_cast<std::size_t>(k)][11] =
                        body.linear_velocity.z;
                }
                for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                {
                    const auto &foot = terrain_plan->predicted_foothold[
                        plan_knot][leg];
                    if (mpc_in.contact[k][leg])
                    {
                        const auto &execution =
                            terrain_swing_execution_[leg];
                        if (effective_transfer_hold[leg])
                        {
                            // A held support foot may already be past its
                            // nominal schedule boundary.  Its live
                            // kinematic anchor is the only valid lever arm
                            // until the new measured contact is promoted.
                            mpc_in.foot_from_com_world_horizon[
                                static_cast<std::size_t>(k)][leg] =
                                dyn.foot_pos_world[leg] - dyn.com_world;
                            mpc_in.foot_valid[
                                static_cast<std::size_t>(k)][leg] = true;
                            continue;
                        }
                        if (k == 0 && execution.valid &&
                            execution.measured_touchdown &&
                            std::isfinite(execution.target_world.x) &&
                            std::isfinite(execution.target_world.y) &&
                            std::isfinite(execution.target_world.z))
                        {
                            mpc_in.foot_from_com_world_horizon[
                                static_cast<std::size_t>(k)][leg] =
                                Eigen::Vector3d(
                                    execution.target_world.x -
                                        dyn.com_world.x(),
                                    execution.target_world.y -
                                        dyn.com_world.y(),
                                    execution.target_world.z -
                                        dyn.com_world.z());
                            mpc_in.foot_valid[
                                static_cast<std::size_t>(k)][leg] = true;
                            continue;
                        }
                        if (k == 0 && effective_transfer_hold[leg])
                        {
                            mpc_in.foot_from_com_world_horizon[
                                static_cast<std::size_t>(k)][leg] =
                                dyn.foot_pos_world[leg] - dyn.com_world;
                            mpc_in.foot_valid[
                                static_cast<std::size_t>(k)][leg] = true;
                            continue;
                        }
                        if (!foot.valid)
                        {
                            // A live contact promoted during the current
                            // transfer can be ahead of the planner knot.
                            // Keep its measured anchor at knot zero; future
                            // knots must still come from the atomic plan.
                            if (k == 0 && qp_contact[leg])
                            {
                                mpc_in.foot_from_com_world_horizon[
                                    static_cast<std::size_t>(k)][leg] =
                                    dyn.foot_pos_world[leg] - dyn.com_world;
                                mpc_in.foot_valid[
                                    static_cast<std::size_t>(k)][leg] = true;
                                continue;
                            }
                            complete_foot_horizon = false;
                            continue;
                        }
                        mpc_in.foot_from_com_world_horizon[
                            static_cast<std::size_t>(k)][leg] =
                            Eigen::Vector3d(
                                foot.position_world.x - dyn.com_world.x(),
                                foot.position_world.y - dyn.com_world.y(),
                                foot.position_world.z - dyn.com_world.z());
                        mpc_in.foot_valid[static_cast<std::size_t>(k)][leg] =
                            true;
                    }
                }
            }
            if (complete_foot_horizon)
            {
                mpc_in.has_time_indexed_footholds = true;
                mpc_in.has_time_indexed_reference = true;
                mpc_in.plan_id = terrain_plan->plan_id;
                mpc_in.plan_epoch = terrain_plan->plan_epoch;
                mpc_in.has_terrain_plan = true;
                mpc_in.terrain_plan.plan_id = terrain_plan->plan_id;
                mpc_in.terrain_plan.plan_epoch = terrain_plan->plan_epoch;
                mpc_in.terrain_plan.map_epoch = terrain_plan->map_epoch;
                mpc_in.terrain_plan.generated_at_s = terrain_plan->generated_at_s;
                mpc_in.terrain_plan.valid_until_s = terrain_plan->valid_until_s;
                mpc_in.measured_contact = measured_contact;
                mpc_in.measured_contact_valid = true;
                ++terrain_mpc_plan_consumed_count_;
            }
            else
                mpc_in.contact = fallback_mpc_contact;
        }
        wbc_shadow_diagnostics_.mpc_update_count =
            ++terrain_mpc_update_count_;
        wbc_shadow_diagnostics_.mpc_contact_mask_k0 =
            mpc_params.horizon > 0 ? contact_mask_for(mpc_in.contact[0]) : 0;
        wbc_shadow_diagnostics_.mpc_min_contact_count =
            static_cast<int>(go2::kLegCount);
        for (int k = 0; k < mpc_params.horizon; ++k)
        {
            const int count = static_cast<int>(std::count(
                mpc_in.contact[k].begin(), mpc_in.contact[k].end(), true));
            wbc_shadow_diagnostics_.mpc_min_contact_count = std::min(
                wbc_shadow_diagnostics_.mpc_min_contact_count, count);
        }
        const auto &first_reference = mpc_in.has_time_indexed_reference
            ? mpc_in.reference_horizon[0] : mpc_in.reference;
        const auto &last_reference = mpc_in.has_time_indexed_reference
            ? mpc_in.reference_horizon[
                  static_cast<std::size_t>(mpc_params.horizon - 1)]
            : mpc_in.reference;
        wbc_shadow_diagnostics_.mpc_reference_x_first_m = first_reference[3];
        wbc_shadow_diagnostics_.mpc_reference_x_last_m = last_reference[3];
        wbc_shadow_diagnostics_.mpc_reference_vx_first_mps =
            first_reference[9];
        wbc_shadow_diagnostics_.mpc_reference_vx_last_mps = last_reference[9];
        go2_control::SrbdMpcOutput mpc_out;
        if (go2_control::SolveSrbdMpc(mpc_params, mpc_in, mpc_out) && mpc_out.ok)
            last_srbd_ = mpc_out;
    }
    ++wbc_full_ticks_;
    wbc_shadow_diagnostics_.srbd_ok = last_srbd_.ok;

    go2_control::IdWbcInput wbc_in;
    wbc_in.dynamics = dyn;
    wbc_in.contact = qp_contact;
    if (terrain_plan_active && terrain_plan_contact_coherent)
    {
        wbc_in.has_terrain_plan = true;
        wbc_in.terrain_plan.plan_id = terrain_plan->plan_id;
        wbc_in.terrain_plan.plan_epoch = terrain_plan->plan_epoch;
        wbc_in.terrain_plan.map_epoch = terrain_plan->map_epoch;
        wbc_in.terrain_plan.generated_at_s = terrain_plan->generated_at_s;
        wbc_in.terrain_plan.valid_until_s = terrain_plan->valid_until_s;
        wbc_in.measured_contact = measured_contact;
        wbc_in.measured_contact_valid = true;
        wbc_in.planned_contact =
            terrain_plan->contact_schedule.planned_contact[
                terrain_plan_knot[0]];
        wbc_in.planned_contact_valid =
            terrain_plan->contact_schedule.planned_valid;
    }
    if (last_srbd_.ok)
    {
        wbc_in.desired_linear_acc_world = last_srbd_.first_linear_acc;
        wbc_shadow_diagnostics_.full_srbd_acc_x_mps2 =
            last_srbd_.first_linear_acc.x();
        const Eigen::Quaterniond quat(
            state_snapshot.imu_state().quaternion()[0],
            state_snapshot.imu_state().quaternion()[1],
            state_snapshot.imu_state().quaternion()[2],
            state_snapshot.imu_state().quaternion()[3]);
        wbc_in.desired_angular_acc_body =
            quat.normalized().toRotationMatrix().transpose() *
            last_srbd_.first_angular_acc;
        if (high_speed_curriculum &&
            Full2EnvDouble("TROT_HS_USE_LEAN", 1.0) > 0.5 &&
            task_.motion_stage_ == 2 && task_.gait_started_ &&
            !task_.stop_requested_ && !motion_event_response_enabled_)
        {
            // ID-WBC returns before the legacy wrench path below, so the
            // sprint lean must be expressed directly in its angular-
            // acceleration task. A small forward pitch lets the contact
            // forces create propulsion without an unbounded torque overlay.
            double pitch_ref = Clamp(
                0.035 + 0.040 * std::max(
                    0.0, std::abs(wbc_speed_cmd_mps_) - 0.50),
                0.035, 0.14);
            const double pitch_ref_override = Full2EnvDouble(
                "TROT_HS_PITCH_REF", -1.0);
            if (pitch_ref_override >= 0.0)
                pitch_ref = Clamp(pitch_ref_override, 0.0, 0.35);
            const double pitch = static_cast<double>(
                state_snapshot.imu_state().rpy()[1]);
            const double gyro_y = static_cast<double>(
                state_snapshot.imu_state().gyroscope()[1]);
            const double pitch_gain = std::max(
                0.0, Full2EnvDouble("TROT_HS_PITCH_GAIN", 12.0));
            const double pitch_damp = std::max(
                0.0, Full2EnvDouble("TROT_HS_PITCH_DAMP", 2.0));
            const double pitch_acc_limit = std::clamp(
                Full2EnvDouble("TROT_HS_PITCH_ACC_LIMIT", 4.0),
                0.5, 8.0);
            wbc_in.desired_angular_acc_body.y() = Clamp(
                pitch_gain * (pitch_ref - pitch) - pitch_damp * gyro_y,
                -pitch_acc_limit, pitch_acc_limit);
            const double roll_gain = std::max(
                0.0, Full2EnvDouble("TROT_HS_ROLL_GAIN", 0.0));
            const double roll_damp = std::max(
                0.0, Full2EnvDouble("TROT_HS_ROLL_DAMP", 6.0));
            const double roll_acc_limit = std::clamp(
                Full2EnvDouble("TROT_HS_ROLL_ACC_LIMIT", 8.0),
                1.0, 12.0);
            const double roll = static_cast<double>(
                state_snapshot.imu_state().rpy()[0]);
            const double gyro_x = static_cast<double>(
                state_snapshot.imu_state().gyroscope()[0]);
            wbc_in.desired_angular_acc_body.x() = Clamp(
                -roll_gain * roll - roll_damp * gyro_x,
                -roll_acc_limit, roll_acc_limit);
        }
        // At sprint speed the acceleration task alone can be satisfied by
        // redistributing qdd into the floating base while the contact-force
        // solution quietly loses its forward component. Keep the MPC GRF as
        // an optional, explicitly weighted reference for this experiment.
        if (high_speed_curriculum &&
            Full2EnvDouble("TROT_HS_FORCE_TRACK", 0.0) > 0.0 &&
            task_.motion_stage_ == 2 && task_.gait_started_ &&
            !task_.stop_requested_ && !motion_event_response_enabled_)
        {
            wbc_in.have_force_ref = true;
            wbc_in.force_ref = last_srbd_.first_force;
        }
        if (high_speed_curriculum &&
            Full2EnvDouble("TROT_HS_USE_VEL_TASK", 1.0) > 0.5 &&
            task_.motion_stage_ == 2 &&
            task_.gait_started_ &&
            !task_.stop_requested_ &&
            !motion_event_response_enabled_ &&
            have_filtered_body_velocity_)
        {
            // The preview MPC remains the posture/contact planner, but at
            // sprint speeds the ID-WBC needs an explicit velocity task in
            // the same acceleration space to actually generate propulsion.
            const double target_vx =
                std::isfinite(kernel_nominal_velocity_x_mps_)
                    ? kernel_nominal_velocity_x_mps_
                    : params_.direction_sign * params_.step_length_m /
                          params_.period_s;
            const double velocity_error =
                target_vx - latest_filtered_body_velocity_[0];
            const double speed_acc = Clamp(
                Full2EnvDouble("TROT_HS_ACC_GAIN", 8.0) * velocity_error,
                -Full2EnvDouble("TROT_HS_ACC_LIMIT", 3.0),
                Full2EnvDouble("TROT_HS_ACC_LIMIT", 3.0));
            wbc_shadow_diagnostics_.full_velocity_target_x_mps = target_vx;
            wbc_in.desired_linear_acc_world.x() = speed_acc;
        }
        const bool stop_balance =
            EmergencyStopHoldReady() ||
            WbcStopHoldActive() ||
            high_speed_stop_brake_active_;
        if (stop_balance)
        {
            // The short emergency-stop window is a four-contact balance
            // problem, not another gait step.  Keep a bounded measured-
            // velocity damper in the WBC task so a stale MPC force cannot
            // push the body forward while the feet are being frozen.
            const double kStopVelocityKp =
                WbcStopHoldActive()
                    ? 5.0
                    : (high_speed_stop_brake_active_
                           ? std::max(6.0, Full2EnvDouble(
                                 "TROT_HS_BRAKE_VEL_KP", 8.0))
                           : 6.0);
            const double kStopAxLimit =
                WbcStopHoldActive()
                    ? 4.0
                    : (high_speed_stop_brake_active_
                           ? std::clamp(Full2EnvDouble(
                                 "TROT_HS_BRAKE_AX_LIMIT", 5.0), 2.5, 8.0)
                           : 2.5);
            const double kStopAyLimit =
                WbcStopHoldActive()
                    ? 3.0
                    : (high_speed_stop_brake_active_
                           ? std::clamp(Full2EnvDouble(
                                 "TROT_HS_BRAKE_AY_LIMIT", 4.0), 2.0, 6.0)
                           : 2.0);
            wbc_in.desired_linear_acc_world.x() = Clamp(
                -kStopVelocityKp * linear_vel_world.x(),
                -kStopAxLimit, kStopAxLimit);
            wbc_in.desired_linear_acc_world.y() = Clamp(
                -kStopVelocityKp * linear_vel_world.y(),
                -kStopAyLimit, kStopAyLimit);
            const double roll = static_cast<double>(
                state_snapshot.imu_state().rpy()[0]);
            const double pitch = static_cast<double>(
                state_snapshot.imu_state().rpy()[1]);
            const double gyro_x = static_cast<double>(
                state_snapshot.imu_state().gyroscope()[0]);
            const double gyro_y = static_cast<double>(
                state_snapshot.imu_state().gyroscope()[1]);
            wbc_in.desired_angular_acc_body.x() = Clamp(
                -40.0 * roll - 8.0 * gyro_x, -4.0, 4.0);
            wbc_in.desired_angular_acc_body.y() = Clamp(
                -40.0 * pitch - 8.0 * gyro_y, -4.0, 4.0);
        }
        if (have_filtered_body_velocity_ && task_.gait_started_ &&
            task_.motion_stage_ == 2 &&
            (params_.cartesian_world || !params_.step_plan.empty()))
        {
            const double v_des =
                std::isfinite(kernel_nominal_velocity_x_mps_)
                    ? kernel_nominal_velocity_x_mps_
                    : 0.0;
            const double v_err =
                v_des - latest_filtered_body_velocity_[0];
            const double yaw =
                static_cast<double>(state_snapshot.imu_state().rpy()[2]);
            const double pitch =
                static_cast<double>(state_snapshot.imu_state().rpy()[1]);
            const double gyro_y =
                static_cast<double>(
                    state_snapshot.imu_state().gyroscope()[1]);
            const double pitch_fade = Clamp(
                1.0 - std::max(0.0, std::abs(pitch) - 0.06) / 0.12,
                0.15, 1.0);
            // 233312-era cartesian ax: 1 m/s² / gain 2, faded by pitch.
            // Later paper_push/blend formulas either exploded (143932)
            // or left CoM ax at 0 while wx glued the feet (141715).
            double ax_lim = params_.cartesian_world ? 1.0 : 3.0;
            double ax_gain = params_.cartesian_world ? 2.0 : 4.0;
            const double ax_lim_ov = Full2EnvDouble("FULL2_AX_LIM", -1.0);
            if (ax_lim_ov > 0.0)
                ax_lim = ax_lim_ov;
            const double ax_gain_ov = Full2EnvDouble("FULL2_AX_GAIN", -1.0);
            if (ax_gain_ov > 0.0)
                ax_gain = ax_gain_ov;
            const double ax_foot = Full2EnvDouble("FULL2_AX_FOOT", 0.0);
            const double ax_foot_v = Full2EnvDouble("FULL2_AX_FOOT_V", 0.0);
            const double v_now = latest_filtered_body_velocity_[0];
            if (ax_foot > 0.0 && cartesian_last_foot_error_m_ > ax_foot &&
                (ax_foot_v <= 0.0 || v_now >= ax_foot_v))
                ax_lim = 0.0;
            const double ax_body =
                pitch_fade * Clamp(ax_gain * v_err, -ax_lim, ax_lim);
            wbc_in.desired_linear_acc_world.x() += ax_body * std::cos(yaw);
            wbc_in.desired_linear_acc_world.y() += ax_body * std::sin(yaw);
            const double roll =
                static_cast<double>(state_snapshot.imu_state().rpy()[0]);
            const double gyro_x =
                static_cast<double>(
                    state_snapshot.imu_state().gyroscope()[0]);
            double roll_kp = params_.cartesian_world ? 40.0 : 20.0;
            const double roll_ov = Full2EnvDouble("FULL2_ROLL", -1.0);
            if (roll_ov > 0.0)
                roll_kp = roll_ov;
            wbc_in.desired_angular_acc_body.x() +=
                -roll_kp * roll -
                (params_.cartesian_world ? 5.0 : 2.5) * gyro_x;
            double pitch_kp = 12.0;
            const double pitch_ov = Full2EnvDouble("FULL2_PITCH", -1.0);
            if (pitch_ov > 0.0)
                pitch_kp = pitch_ov;
            wbc_in.desired_angular_acc_body.y() +=
                -pitch_kp * pitch - 1.5 * gyro_y - 0.25 * ax_body;
            if (params_.cartesian_world)
            {
                const double yaw_err = WrapAngle(
                    yaw - world_reference_yaw_rad_);
                const double gyro_z = static_cast<double>(
                    state_snapshot.imu_state().gyroscope()[2]);
                wbc_in.desired_angular_acc_body.z() +=
                    -8.0 * yaw_err - 2.0 * gyro_z;
            }
        }
    }
    if (have_commanded_body_feet_)
    {
        const WorldPose pose =
            ComputeWorldPose(state_snapshot, high_state_snapshot);
        const Eigen::Quaterniond quat(
            pose.quaternion[0], pose.quaternion[1],
            pose.quaternion[2], pose.quaternion[3]);
        const Eigen::Matrix3d R = quat.normalized().toRotationMatrix();
        Eigen::Matrix<double, go2_control::kGo2Nv, 1> qvel =
            Eigen::Matrix<double, go2_control::kGo2Nv, 1>::Zero();
        qvel.head<3>() = linear_vel_world;
        qvel.segment<3>(3) = Eigen::Vector3d(
            state_snapshot.imu_state().gyroscope()[0],
            state_snapshot.imu_state().gyroscope()[1],
            state_snapshot.imu_state().gyroscope()[2]);
        for (int i = 0; i < kMotorCount; ++i)
        {
            const int dof = rigid_body_->MotorDof(i);
            if (dof >= 6 && dof < go2_control::kGo2Nv)
                qvel[dof] = state_snapshot.motor_state()[i].dq();
        }
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            const Eigen::Vector3d v = dyn.foot_jac_world[leg] * qvel;
            if (qp_contact[leg])
            {
                if (params_.cartesian_world && have_commanded_world_feet_)
                {
                    const Eigen::Vector3d p_des(
                        commanded_world_feet_[leg].x,
                        commanded_world_feet_[leg].y,
                        commanded_world_feet_[leg].z);
                    if (cartesian_kp_frozen_ && cartesian_latched_kp_ <= 5.0)
                    {
                        // Mini Cheetah WBIC: J qdd + Jdot qd ≈ 0, not a
                        // cartesian spring. 132034 zeroed a_des.x and
                        // dropped hard no-slip; last-8s 0.32 with more
                        // Y crab. Keep the keeper damper; anisotropy
                        // is in the no-slip weights.
                        wbc_in.stance_acc_world[leg] = ClampVec3(-8.0 * v, 4.0);
                    }
                    else
                    {
                        const double kp_foot =
                            40.0 + 120.0 * cart_lock + 100.0 * force_blend * tst_gate;
                        const double kd_foot =
                            10.0 + 10.0 * cart_lock + 12.0 * force_blend * tst_gate;
                        wbc_in.stance_acc_world[leg] = ClampVec3(
                            kp_foot * (p_des - dyn.foot_pos_world[leg]) -
                                kd_foot * v,
                            8.0 + 12.0 * cart_lock + 10.0 * force_blend * tst_gate);
                    }
                    wbc_in.have_stance_acc = true;
                }
                continue;
            }
            Eigen::Vector3d p_des =
                Eigen::Vector3d(pose.base.x, pose.base.y, pose.base.z) +
                R * Eigen::Vector3d(
                        commanded_body_feet_[leg].x,
                        commanded_body_feet_[leg].y,
                        commanded_body_feet_[leg].z);
            Eigen::Vector3d v_des = Eigen::Vector3d::Zero();
            if (have_commanded_body_feet_velocity_ &&
                !params_.cartesian_world)
            {
                const Eigen::Vector3d omega_body(
                    state_snapshot.imu_state().gyroscope()[0],
                    state_snapshot.imu_state().gyroscope()[1],
                    state_snapshot.imu_state().gyroscope()[2]);
                const Eigen::Vector3d r_body(
                    commanded_body_feet_[leg].x,
                    commanded_body_feet_[leg].y,
                    commanded_body_feet_[leg].z);
                const Eigen::Vector3d rel_v_body(
                    commanded_body_feet_velocity_[leg].x,
                    commanded_body_feet_velocity_[leg].y,
                    commanded_body_feet_velocity_[leg].z);
                // Swing tracking must carry the reference velocity.  A
                // zero v_des makes the WBC fight a fast foot trajectory with
                // pure damping, which is the dominant error at sprint speed.
                v_des = ClampVec3(
                    linear_vel_world + R *
                        (omega_body.cross(r_body) + rel_v_body),
                    12.0);
            }
            if (params_.cartesian_world && have_commanded_world_feet_)
            {
                p_des = Eigen::Vector3d(
                    commanded_world_feet_[leg].x,
                    commanded_world_feet_[leg].y,
                    commanded_world_feet_[leg].z);
                v_des = Eigen::Vector3d(
                    cartesian_state_.target_world_vel[leg].x,
                    cartesian_state_.target_world_vel[leg].y,
                    cartesian_state_.target_world_vel[leg].z);
            }
            const Eigen::Vector3d p = dyn.foot_pos_world[leg];
            const double swing_kp = Full2EnvDouble("FULL2_SWING_KP", 180.0);
            const double swing_kd = Full2EnvDouble("FULL2_SWING_KD", 16.0);
            const double swing_acc_lim = Full2EnvDouble("FULL2_SWING_ACC", 50.0);
            wbc_in.swing_acc_world[leg] = ClampVec3(
                swing_kp * (p_des - p) + swing_kd * (v_des - v),
                swing_acc_lim);
        }
    }

    go2_control::IdWbcOutput wbc_out;
    go2_control::IdWbcParams id_params;
    const int n_contact =
        (qp_contact[0] ? 1 : 0) + (qp_contact[1] ? 1 : 0) +
        (qp_contact[2] ? 1 : 0) + (qp_contact[3] ? 1 : 0);
    id_params.w_stance_no_slip =
        params_.cartesian_world ? (50.0 + 90.0 * cart_lock) : 8.0;
    const double w_no_slip_x_ov = Full2EnvDouble("FULL2_WX_X", -1.0);
    if (w_no_slip_x_ov >= 0.0)
        id_params.w_stance_no_slip_x = w_no_slip_x_ov;
    id_params.w_base_lin = params_.cartesian_world ? 80.0 : 80.0;
    const double w_lin_ov = Full2EnvDouble("FULL2_W_LIN", -1.0);
    if (w_lin_ov > 0.0)
        id_params.w_base_lin = w_lin_ov;
    const double w_lin_x_ov = Full2EnvDouble("FULL2_W_LIN_X", -1.0);
    if (w_lin_x_ov >= 0.0)
        id_params.w_base_lin_x = w_lin_x_ov;
    id_params.w_base_ang = params_.cartesian_world
        ? (80.0 + 30.0 * cart_lock)
        : 40.0;
    const double w_ang_ov = Full2EnvDouble("FULL2_W_ANG", -1.0);
    if (w_ang_ov > 0.0)
        id_params.w_base_ang = w_ang_ov;
    id_params.w_swing = params_.cartesian_world ? 80.0 : 80.0;
    const double w_sw_ov = Full2EnvDouble("FULL2_W_SWING", -1.0);
    if (w_sw_ov > 0.0)
        id_params.w_swing = w_sw_ov;
    const double w_sw_x_ov = Full2EnvDouble("FULL2_W_SWING_X", -1.0);
    if (w_sw_x_ov >= 0.0)
        id_params.w_swing_x = w_sw_x_ov;
    const double force_track_ov = Full2EnvDouble(
        "TROT_HS_FORCE_TRACK", 0.0);
    if (high_speed_curriculum && force_track_ov > 0.0)
        id_params.w_force_track = std::clamp(force_track_ov, 0.0, 1.0);
    id_params.tau_limit_nm = 35.0;
    const double tau_ov = Full2EnvDouble("FULL2_TAU", -1.0);
    if (tau_ov > 0.0)
        id_params.tau_limit_nm = tau_ov;
    if (params_.cartesian_world)
    {
        // Original cartesian-world: soft no-slip only. Hard equality
        // and freeze wx-open both produced either a 0.33 sit or a
        // one-cycle 0.40 then roll (152821/161952).
        id_params.hard_stance_no_slip = false;
        id_params.w_force = 1.0e-6;
        id_params.w_force_track = cart_lock * 0.008;
        if (last_srbd_.ok)
        {
            wbc_in.have_force_ref = true;
            wbc_in.force_ref = last_srbd_.first_force;
        }
    }
    bool solved =
        go2_control::SolveInverseDynamicsWbc(id_params, wbc_in, wbc_out) &&
        wbc_out.ok;
    if (!solved && id_params.hard_stance_no_slip)
    {
        id_params.hard_stance_no_slip = false;
        solved =
            go2_control::SolveInverseDynamicsWbc(id_params, wbc_in, wbc_out) &&
            wbc_out.ok;
    }
    if (solved)
    {
        last_id_wbc_ = wbc_out;
        have_last_id_wbc_ = true;
    }
    else if (have_last_id_wbc_)
    {
        wbc_out = last_id_wbc_;
    }
    else
    {
        return;
    }

    // Sprint-only pitch moment trim.  The ID-WBC task can lose the small
    // front/rear normal-force split needed to hold the torso while the
    // diagonal pair is accelerating.  Redistribute a bounded amount of
    // normal force between the active front and rear feet; total support
    // force is unchanged, and the QP torque limits remain the final gate.
    if (high_speed_curriculum && task_.motion_stage_ == 2 &&
        task_.gait_started_ && !task_.stop_requested_)
    {
        const double force_diff_max = std::clamp(
            Full2EnvDouble("TROT_HS_PITCH_FORCE_DIFF_MAX", 0.0),
            0.0, 30.0);
        if (force_diff_max > 0.0)
        {
            const double pitch_ref = std::clamp(
                Full2EnvDouble("TROT_HS_PITCH_REF", 0.0), -0.30, 0.30);
            const double pitch = static_cast<double>(
                state_snapshot.imu_state().rpy()[1]);
            const double gyro_y = static_cast<double>(
                state_snapshot.imu_state().gyroscope()[1]);
            const double gain = std::max(
                0.0, Full2EnvDouble("TROT_HS_PITCH_FORCE_DIFF_GAIN", 24.0));
            const double damp = std::max(
                0.0, Full2EnvDouble("TROT_HS_PITCH_FORCE_DIFF_DAMP", 2.0));
            double diff = std::clamp(
                gain * (pitch - pitch_ref) + damp * gyro_y,
                -force_diff_max, force_diff_max);
            std::array<int, 2> front{};
            std::array<int, 2> rear{};
            int n_front = 0;
            int n_rear = 0;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                if (!qp_contact[leg])
                    continue;
                if (leg < 2)
                    front[n_front++] = static_cast<int>(leg);
                else
                    rear[n_rear++] = static_cast<int>(leg);
            }
            if (n_front > 0 && n_rear > 0 && std::abs(diff) > 1.0e-9)
            {
                const double front_share = diff / n_front;
                const double rear_share = -diff / n_rear;
                double scale = 1.0;
                for (int i = 0; i < n_front; ++i)
                {
                    const int leg = front[i];
                    const double fz = wbc_out.force[3 * leg + 2];
                    if (front_share > 0.0)
                        scale = std::min(scale, (180.0 - fz) /
                                                     std::max(front_share, 1.0e-9));
                    else
                        scale = std::min(scale, (fz - 2.0) /
                                                     std::max(-front_share, 1.0e-9));
                }
                for (int i = 0; i < n_rear; ++i)
                {
                    const int leg = rear[i];
                    const double fz = wbc_out.force[3 * leg + 2];
                    if (rear_share > 0.0)
                        scale = std::min(scale, (180.0 - fz) /
                                                     std::max(rear_share, 1.0e-9));
                    else
                        scale = std::min(scale, (fz - 2.0) /
                                                     std::max(-rear_share, 1.0e-9));
                }
                scale = std::clamp(scale, 0.0, 1.0);
                for (int i = 0; i < n_front; ++i)
                {
                    const int leg = front[i];
                    Eigen::Vector3d delta = Eigen::Vector3d::Zero();
                    delta.z() = scale * front_share;
                    wbc_out.tau -=
                        dyn.foot_jac_world[leg].rightCols<12>().transpose() * delta;
                    wbc_out.force.segment<3>(3 * leg) += delta;
                }
                for (int i = 0; i < n_rear; ++i)
                {
                    const int leg = rear[i];
                    Eigen::Vector3d delta = Eigen::Vector3d::Zero();
                    delta.z() = scale * rear_share;
                    wbc_out.tau -=
                        dyn.foot_jac_world[leg].rightCols<12>().transpose() * delta;
                    wbc_out.force.segment<3>(3 * leg) += delta;
                }
            }
        }
    }

    // Optional sprint-only propulsion bias.  The regular ID-WBC solution
    // can spend its horizontal wrench budget on the swing-foot task when the
    // commanded speed is high.  Keep this disabled by default; when enabled
    // it adds a bounded, contact-distributed J^T f term so experiments can
    // distinguish a propulsion bottleneck from a foot-placement bottleneck.
    if (!params_.impulse && high_speed_curriculum &&
        have_filtered_body_velocity_ && task_.gait_started_ &&
        task_.motion_stage_ == 2 && !task_.stop_requested_)
    {
        const double force_max = Full2EnvDouble(
            "TROT_HS_DIRECT_FORCE_MAX", 0.0);
        if (force_max > 0.0)
        {
            const double target_v =
                std::isfinite(kernel_nominal_velocity_x_mps_)
                    ? kernel_nominal_velocity_x_mps_
                    : params_.direction_sign * params_.step_length_m /
                          params_.period_s;
            const double gain = std::max(
                0.0, Full2EnvDouble("TROT_HS_DIRECT_FORCE_GAIN", 0.0));
            const double push = std::clamp(
                gain * (target_v - latest_filtered_body_velocity_[0]),
                -force_max, force_max);
            if (std::abs(push) > 1.0e-9 && active > 0)
            {
                const double per_contact = push / static_cast<double>(active);
                for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
                {
                    if (!qp_contact[leg])
                        continue;
                    Eigen::Vector3d f = Eigen::Vector3d::Zero();
                    f.x() = per_contact;
                    // In the inverse-dynamics convention used above,
                    // tau = Mj*qdd + h_j - J_j^T*f.  A forward ground-force
                    // overlay therefore enters with the same minus sign;
                    // adding J^T*f would command a backward push.
                    wbc_out.tau -=
                        dyn.foot_jac_world[leg].rightCols<12>().transpose() * f;
                    wbc_out.force.segment<3>(3 * static_cast<int>(leg)) += f;
                }
            }
        }
    }

    if (params_.cartesian_world && have_commanded_world_feet_ &&
        task_.gait_started_ && task_.motion_stage_ == 2 &&
        (cart_lock > 0.05) &&
        !(cartesian_kp_frozen_ && cartesian_latched_kp_ <= 5.0))
    {
        Eigen::Matrix<double, 12, 1> tau_pd =
            Eigen::Matrix<double, 12, 1>::Zero();
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            if (!qp_contact[leg])
                continue;
            const Eigen::Vector3d v = dyn.foot_jac_world[leg] * dyn.qvel;
            const Eigen::Vector3d dp =
                Eigen::Vector3d(
                    commanded_world_feet_[leg].x,
                    commanded_world_feet_[leg].y,
                    commanded_world_feet_[leg].z) -
                dyn.foot_pos_world[leg];
            Eigen::Vector3d f_hold = Eigen::Vector3d::Zero();
            f_hold.x() = std::clamp(
                cart_lock * (80.0 * dp.x() - 28.0 * v.x()), -18.0, 18.0);
            f_hold.y() = std::clamp(
                cart_lock * (80.0 * dp.y() - 28.0 * v.y()), -18.0, 18.0);
            const Eigen::Vector3d f_pd = (1.0 - force_blend) * f_hold;
            tau_pd +=
                dyn.foot_jac_world[leg].rightCols<12>().transpose() * f_pd;
        }
        wbc_out.tau += tau_pd;
        for (int i = 0; i < 12; ++i)
            wbc_out.tau[i] = std::clamp(wbc_out.tau[i], -35.0, 35.0);
    }

    wbc_shadow_diagnostics_.solver_ok = true;
    wbc_shadow_diagnostics_.full_requested_acc_x_mps2 =
        wbc_in.desired_linear_acc_world.x();
    wbc_shadow_diagnostics_.full_id_qdd_x_mps2 = wbc_out.qdd[0];
    wbc_shadow_diagnostics_.full_id_contact_force_x_n = 0.0;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        wbc_shadow_diagnostics_.full_id_contact_force_x_n +=
            wbc_out.force[3 * static_cast<int>(leg)];
    wbc_shadow_diagnostics_.mapping_ok = true;
    wbc_shadow_diagnostics_.id_wbc_ok = solved;
    wbc_shadow_diagnostics_.wrench_satisfied = wbc_out.eq_residual < 1.0;
    wbc_shadow_diagnostics_.constraint_feasible = true;
    wbc_shadow_diagnostics_.task_satisfied = wbc_out.eq_residual < 1.0;
    wbc_shadow_diagnostics_.residual_norm = wbc_out.eq_residual;
    wbc_shadow_diagnostics_.id_eq_residual = wbc_out.eq_residual;
    wbc_shadow_diagnostics_.task_residual_norm = wbc_out.rne_residual;
    wbc_shadow_diagnostics_.iterations = wbc_out.iterations;
    wbc_shadow_diagnostics_.active_contacts = active;
    wbc_shadow_diagnostics_.contact_mask = contact_mask;
    wbc_shadow_diagnostics_.max_abs_tau = wbc_out.tau.cwiseAbs().maxCoeff();
    wbc_shadow_diagnostics_.desired_force_x_n =
        last_srbd_.ok ? last_srbd_.first_force[0] + last_srbd_.first_force[3] +
                            last_srbd_.first_force[6] + last_srbd_.first_force[9]
                      : 0.0;
    double min_fz = 1.0e9;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const Eigen::Vector3d f = wbc_out.force.segment<3>(3 * static_cast<int>(leg));
        for (int j = 0; j < 3; ++j)
        {
            const int motor = static_cast<int>(3 * leg + j);
            const int dof = rigid_body_->MotorDof(motor);
            const int joint_row = dof - 6;
            wbc_shadow_candidate_torques_[leg][j] =
                (joint_row >= 0 && joint_row < 12) ? wbc_out.tau[joint_row] : 0.0;
        }
        if (qp_contact[leg])
            min_fz = std::min(min_fz, f.z());
    }
    wbc_shadow_diagnostics_.min_contact_normal_force_n =
        std::isfinite(min_fz) ? min_fz : 0.0;
}

// --- TrotExperiment::UpdateWbcShadow ---
void TrotExperiment::UpdateWbcShadow(
    const unitree_go::msg::dds_::LowState_ &state_snapshot,
    bool have_state,
    const unitree_go::msg::dds_::SportModeState_ &high_state_snapshot,
    bool have_high_state)
{
    wbc_shadow_diagnostics_ = WbcShadowDiagnostics{};
    wbc_shadow_contact_state_valid_ = false;
    const bool high_speed_curriculum =
        Full2EnvDouble("TROT_HS_DISABLE", 0.0) <= 0.5 &&
        (params_.gait_pattern != go2_control::GaitPattern::kDiagonalTrot ||
         std::abs(wbc_speed_cmd_mps_) > 1.25 ||
         std::abs(kernel_nominal_velocity_x_mps_) > 1.25);
    const auto shadow_start = std::chrono::steady_clock::now();
    const auto finish_shadow_timing = [&]() {
        wbc_shadow_diagnostics_.elapsed_us =
            std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - shadow_start).count();
        wbc_shadow_diagnostics_.within_budget =
            wbc_shadow_diagnostics_.elapsed_us <= kShadowWbcBudgetUs;
    };
    if (params_.wbc_full && have_state && have_high_state)
    {
        UpdateWbcFull(state_snapshot, high_state_snapshot);
        finish_shadow_timing();
        return;
    }
    const TrueDynamics true_dyn = ExtractTrueDynamics(state_snapshot);
    if (true_dyn.valid && !dynamics_logged_)
    {
        dynamics_logged_ = true;
        std::cout << "TrueDynamics: M00=" << true_dyn.base_mass_matrix[0]
                  << " M05=" << true_dyn.base_mass_matrix[5]
                  << " bias_z=" << true_dyn.base_qfrc_bias[2]
                  << " bias_pitch=" << true_dyn.base_qfrc_bias[4]
                  << "\n";
    }
    wbc_shadow_candidate_torques_ = {};
    wbc_shadow_diagnostics_.enabled = params_.wbc_shadow;
    if (!params_.wbc_shadow || !have_state)
    {
        finish_shadow_timing();
        return;
    }

    go2_control::ProjectedContactWrenchRequest request;
    request.wrench.contact.fill(false);
    request.force_constraints.friction_coefficient =
        kShadowWbcFrictionCoefficient;
    request.force_constraints.max_normal_force =
        kShadowWbcMaxNormalForce;

    go2_control::JointAngles joint_angles{};
    int active_contacts = 0;
    int contact_mask = 0;
    const go2_control::HystereticContactParams shadow_contact_params{
        kShadowContactOnForceN, kShadowContactOffForceN};
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const int motor = static_cast<int>(3 * leg);
        joint_angles[leg] = {
            static_cast<double>(state_snapshot.motor_state()[motor + 0].q()),
            static_cast<double>(state_snapshot.motor_state()[motor + 1].q()),
            static_cast<double>(state_snapshot.motor_state()[motor + 2].q())};
        request.wrench.contact_positions_body[leg] =
            go2::FootPosition(
                static_cast<go2::Leg>(leg),
                joint_angles[leg][0],
                joint_angles[leg][1],
                joint_angles[leg][2]);
        const double foot_force =
            static_cast<double>(state_snapshot.foot_force()[leg]);
        bool next_contact = false;
        if (!go2_control::UpdateHystereticContact(
                wbc_shadow_contact_state_[leg],
                foot_force,
                shadow_contact_params,
                next_contact))
        {
            finish_shadow_timing();
            return;
        }
        wbc_shadow_contact_state_[leg] = next_contact;
        request.wrench.contact[leg] = wbc_shadow_contact_state_[leg];
        if (request.wrench.contact[leg])
        {
            ++active_contacts;
            contact_mask |= 1 << static_cast<int>(leg);
        }
    }
    wbc_shadow_contact_state_valid_ = true;
    wbc_shadow_diagnostics_.active_contacts = active_contacts;
    const bool reduced_contact_task =
        params_.wbc_reduced_contact_task &&
        !params_.wbc_full &&
        active_contacts < go2::kLegCount;
    if (reduced_contact_task)
        request.wrench.task_weights = {1.0, 1.0, 1.0, 0.0, 0.0, 0.0};
    else if (params_.wbc_full)
        request.wrench.task_weights = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    else if (params_.impulse)
    {
        // [wrench-fix] 冲量模式: 姿态力矩权重优先于推力,
        // 防止推力产生的前倾力矩压倒姿态任务(分配器优先保姿态)。
        request.wrench.task_weights = {0.5, 0.5, 1.0, 2.0, 2.0, 1.0};
    }
    wbc_shadow_diagnostics_.reduced_contact_task = reduced_contact_task;

    double desired_force_x_n = 0.0;
    double desired_force_y_n = 0.0;
    const bool disable_high_speed_velocity_wrench =
        high_speed_curriculum &&
        Full2EnvDouble("TROT_HS_DISABLE_VELOCITY_WRENCH", 0.0) > 0.5;
    if (params_.wbc_velocity_wrench &&
        !disable_high_speed_velocity_wrench &&
        task_.motion_stage_ == 2 &&
        task_.gait_started_ &&
        !task_.stop_requested_ &&
        have_filtered_body_velocity_)
    {
        const double target_velocity_x_mps =
            motion_event_response_enabled_
                ? motion_reference_.vx_mps
                : (std::isfinite(kernel_nominal_velocity_x_mps_) &&
                           std::abs(kernel_nominal_velocity_x_mps_) > 1.0e-6
                       ? kernel_nominal_velocity_x_mps_
                       : params_.direction_sign * params_.step_length_m /
                             params_.period_s);
        const double velocity_error_x_mps =
            target_velocity_x_mps - latest_filtered_body_velocity_[0];
        desired_force_x_n = Clamp(
            kShadowWbcMassKg * params_.wbc_velocity_gain_s_inv *
                velocity_error_x_mps,
            -params_.wbc_max_forward_force_n,
            params_.wbc_max_forward_force_n);
        const double target_velocity_y_mps =
            motion_event_response_enabled_ ? motion_reference_.vy_mps : 0.0;
        const double velocity_error_y_mps =
            target_velocity_y_mps - latest_filtered_body_velocity_[1];
        desired_force_y_n = Clamp(
            kShadowWbcMassKg * params_.wbc_velocity_gain_s_inv *
                velocity_error_y_mps,
            -params_.wbc_max_forward_force_n,
            params_.wbc_max_forward_force_n);
    }
    wbc_shadow_diagnostics_.desired_force_x_n = desired_force_x_n;
    double desired_force_z_n = 0.0;  // [增量式] z 力交给位置伺服(避免重复补偿)
    double desired_tau_x_nm = 0.0;
    double desired_tau_y_nm = 0.0;
    double desired_tau_z_nm = 0.0;
    std::array<double, 3> desired_force{
        desired_force_x_n, desired_force_y_n, desired_force_z_n};
    std::array<double, 3> desired_torque{
        desired_tau_x_nm, desired_tau_y_nm, desired_tau_z_nm};
    const bool enhanced_wrench_active =
        params_.wbc_primary && have_high_state &&
        task_.gait_started_ &&
        (running_time_ - task_.gait_start_time_s_) >=
            kWbcPrimaryWrenchEnableS;
    if (enhanced_wrench_active)
    {
        // 基座高度 PD(目标 0.42 m)
        const WorldPose pose =
            ComputeWorldPose(state_snapshot, high_state_snapshot);
        const double height_error_m =
            kWbcPrimaryBaseHeightM - pose.base.z;
        const double base_vel_z_mps =
            static_cast<double>(high_state_snapshot.velocity()[2]);
        // [wrench-fix] z 目标 = 纯重力基底(支撑归一化)。
        // 位置环(kp=63)已扛重力+高度, wrench 再要全重力会双倍补偿;
        // 但 z=0 又让摩擦锥无法产生水平力。折中: 只给重力基底,
        // 高度修正完全交给位置环, wrench z 明确=需要的法向支撑。
        if (params_.impulse)
        {
            // [wrench-fix] 重力基底加到 desired_force[2](数组), 不是标量
            // (数组在块外拷贝, 标量改动不会反映到 wrench)。
            const double gravity_base_n =
                kShadowWbcMassKg * kShadowWbcGravityMps2 *
                (static_cast<double>(active_contacts) /
                 static_cast<double>(go2::kLegCount));
            desired_force[2] += gravity_base_n;
        }
        else
        {
            desired_force[2] +=
                kWbcPrimaryHeightKp * height_error_m -
                kWbcPrimaryHeightKd * base_vel_z_mps;
        }
        // 姿态 PD + 角速度阻尼(imu rpy 与 gyro)
        const double roll_rad =
            static_cast<double>(state_snapshot.imu_state().rpy()[0]);
        const double pitch_rad =
            static_cast<double>(state_snapshot.imu_state().rpy()[1]);
        const double gyro_x_radps =
            static_cast<double>(state_snapshot.imu_state().gyroscope()[0]);
        const double gyro_y_radps =
            static_cast<double>(state_snapshot.imu_state().gyroscope()[1]);
        const double gyro_z_radps =
            static_cast<double>(state_snapshot.imu_state().gyroscope()[2]);
        desired_tau_x_nm =
            -kWbcPrimaryRollKp * roll_rad -
            kWbcPrimaryRollKd * gyro_x_radps;
        desired_tau_y_nm =
            -kWbcPrimaryPitchKp * pitch_rad -
            kWbcPrimaryPitchKd * gyro_y_radps;
        // [Phase6] 真动力学基座任务层:wrench = M_base x a_desired + 0*bias(增量式)
        const TrueDynamics dyn = ExtractTrueDynamics(state_snapshot);
        if (dyn.valid)
        {
            std::array<double, 6> a_desired{};
            if (params_.impulse)
            {
                // 冲量主控 v1: 线动量任务(加速度域)
                // a = Kp*(v_target - v) + Kd*(-a_meas)
                const double target_vx =
                    motion_event_response_enabled_
                        ? motion_reference_.vx_mps
                        : params_.direction_sign * params_.step_length_m /
                              params_.period_s;
                const double target_vy = 0.0;
                if (have_filtered_body_velocity_)
                {
                    const double v_err_x =
                        target_vx - latest_filtered_body_velocity_[0];
                    const double v_err_y =
                        target_vy - latest_filtered_body_velocity_[1];
                    double damp_x = 0.0;
                    double damp_y = 0.0;
                    if (have_body_acceleration_)
                    {
                        damp_x = latest_body_acceleration_[0];
                        damp_y = latest_body_acceleration_[1];
                    }
                    a_desired[0] = Clamp(
                        kImpulseLinVelKpS * v_err_x -
                            kImpulseLinVelKd * damp_x,
                        -kImpulseLinAccMaxMps2, kImpulseLinAccMaxMps2);
                    // [impulse] y 向弱增益(直线保持): 高速时 y 扰动
                    // 被强增益放大导致侧偏/roll 累积, y 只需弱纠正。
                    a_desired[1] = Clamp(
                        kImpulseLinVelKpS * 0.3 * v_err_y -
                            kImpulseLinVelKd * 0.5 * damp_y,
                        -kImpulseLinAccMaxMps2 * 0.5,
                        kImpulseLinAccMaxMps2 * 0.5);
                }
            }
            else
            {
                a_desired[0] = Clamp(
                    kWbcPrimaryVelGain1S * desired_force_x_n /
                        kShadowWbcMassKg,
                    -3.0, 3.0);
                a_desired[1] = Clamp(
                    kWbcPrimaryVelGain1S * desired_force_y_n /
                        kShadowWbcMassKg,
                    -2.0, 2.0);
                if (params_.wbc_full && !motion_event_response_enabled_ &&
                    !high_speed_curriculum &&
                    have_preview_terminal_velocity_ &&
                    !WbcStopHoldActive())
                {
                    if (std::isfinite(preview_planned_acc_x_mps2_))
                        a_desired[0] = Clamp(
                            preview_planned_acc_x_mps2_, -3.0, 3.0);
                    else
                    {
                        double preview_acc_x = 0.0;
                        if (go2_control::PreviewTerminalAcceleration(
                                params_.direction_sign *
                                    params_.step_length_m /
                                    params_.period_s,
                                preview_terminal_velocity_x_mps_,
                                preview_n_steps_,
                                params_.period_s,
                                preview_acc_x))
                        {
                            a_desired[0] = Clamp(preview_acc_x, -3.0, 3.0);
                        }
                    }
                }
            }
            const double bounce_phase =
                2.0 * kPi * 2.0 / params_.period_s *
                (running_time_ - task_.gait_start_time_s_);
            const double bounce_acc = WbcStopHoldActive()
                ? 0.0
                : params_.bounce_acc_amp * std::sin(bounce_phase);
            a_desired[2] = params_.impulse
                ? 0.0  // [wrench-fix] 高度完全交给位置环, wrench z=纯重力
                : Clamp(
                      kWbcPrimaryHeightAccKp * height_error_m -
                          kWbcPrimaryHeightAccKd * base_vel_z_mps +
                          bounce_acc,
                      -3.0, 3.0);
            const double attitude_acc_lim =
                params_.impulse ? 8.0 : 4.0;
            // [impulse] pitch 前倾参考 3°: 动态 trot 自然前倾,
            // 推力产生的前倾不被姿态任务强行拉回(减少对抗),
            // 但限幅防过度前倾。
            double pitch_ref_rad =
                params_.impulse ? 3.0 * kPi / 180.0 : 0.0;
            if (!params_.impulse && high_speed_curriculum &&
                Full2EnvDouble("TROT_HS_USE_LEAN", 1.0) > 0.5)
            {
                // A bound/gallop needs a small forward lean to turn the
                // horizontal centroidal force into a stable ground reaction.
                const double speed_cmd =
                    std::abs(wbc_speed_cmd_mps_ >= 0.0
                                 ? wbc_speed_cmd_mps_
                                 : kernel_nominal_velocity_x_mps_);
                pitch_ref_rad = Clamp(
                    0.035 + 0.040 * std::max(0.0, speed_cmd - 0.50),
                    0.035, 0.14);
                const double pitch_ref_override = Full2EnvDouble("TROT_HS_PITCH_REF", -1.0);
                if (pitch_ref_override >= 0.0)
                    pitch_ref_rad = Clamp(pitch_ref_override, 0.0, 0.35);
            }
            a_desired[3] = Clamp(
                -kWbcPrimaryRollAccKp * roll_rad -
                    kWbcPrimaryRollAccKd * gyro_x_radps,
                -attitude_acc_lim, attitude_acc_lim);
            a_desired[4] = Clamp(
                kWbcPrimaryPitchAccKp * (pitch_ref_rad - pitch_rad) -
                    kWbcPrimaryPitchAccKd * gyro_y_radps,
                -attitude_acc_lim, attitude_acc_lim);
            const double turn_rate = motion_event_response_enabled_
                ? motion_reference_.yaw_rate_radps
                : (task_.goal_enabled_
                    ? task_.TurnEnable(running_time_) *
                          task_.commanded_turn_rate_radps_
                    : params_.turn_rate_radps);
            const double turn_enable_w = motion_event_response_enabled_
                ? 1.0
                : (task_.goal_enabled_
                    ? 1.0
                    : task_.TurnEnable(running_time_));
            a_desired[5] = Clamp(
                kWbcPrimaryTurnYawAccKp *
                        (turn_enable_w * turn_rate -
                         gyro_z_radps) -
                    kWbcPrimaryYawAccKd * gyro_z_radps,
                -4.0, 4.0);
            if (params_.wbc_full)
            {
                go2_control::CentroidalMass mass;
                mass.mass_matrix = dyn.base_mass_matrix;
                mass.bias = dyn.base_qfrc_bias;
                mass.include_bias = true;
                go2_control::CentroidalTask task;
                task.desired_acc = a_desired;
                go2_control::CentroidalWrench built;
                if (go2_control::BuildCentroidalWrench(mass, task, built) &&
                    built.valid)
                {
                    desired_force = {
                        built.wrench[0], built.wrench[1], built.wrench[2]};
                    desired_torque = {
                        built.wrench[3], built.wrench[4], built.wrench[5]};
                }
            }
            else
            {
                for (int i = 0; i < 6; ++i)
                {
                    double w = 0.0;
                    for (int j = 0; j < 6; ++j)
                        w += dyn.base_mass_matrix[i * 6 + j] * a_desired[j];
                    if (i < 3)
                        desired_force[static_cast<std::size_t>(i)] += w;
                    else
                        desired_torque[static_cast<std::size_t>(i - 3)] += w;
                }
            }
        }
    }
    wbc_shadow_diagnostics_.desired_force_x_n = desired_force_x_n;
    request.wrench.desired_wrench = {
        desired_force[0], desired_force[1], desired_force[2],
        desired_torque[0], desired_torque[1], desired_torque[2]};
    wbc_shadow_diagnostics_.contact_mask = contact_mask;

    go2_control::ContactForces contact_forces{};
    if (params_.wbc_full)
    {
        go2_control::ContactWrenchQpAllocator qp_allocator;
        go2_control::ProjectedContactWrenchSolution qp_solution;
        if (!qp_allocator.Solve(request, qp_solution))
        {
            finish_shadow_timing();
            return;
        }
        contact_forces = qp_solution.forces;
        wbc_shadow_diagnostics_.solver_ok = true;
        wbc_shadow_diagnostics_.wrench_satisfied = qp_solution.wrench_satisfied;
        wbc_shadow_diagnostics_.constraint_feasible =
            qp_solution.constraint_report.feasible;
        wbc_shadow_diagnostics_.iterations = qp_solution.iterations;
        wbc_shadow_diagnostics_.residual_norm = qp_solution.residual_norm;
        wbc_shadow_diagnostics_.task_satisfied = qp_solution.task_satisfied;
        wbc_shadow_diagnostics_.task_residual_norm =
            qp_solution.task_residual_norm;
        wbc_shadow_diagnostics_.max_axis_friction_ratio =
            qp_solution.max_axis_friction_ratio;
        wbc_shadow_diagnostics_.max_radial_friction_ratio =
            qp_solution.max_radial_friction_ratio;
        wbc_shadow_diagnostics_.min_contact_normal_force_n =
            qp_solution.min_contact_normal_force;
    }
    else
    {
        go2_control::ContactWrenchProjectedAllocator allocator;
        go2_control::ProjectedContactWrenchSolution wrench_solution;
        if (!allocator.Solve(request, wrench_solution))
        {
            finish_shadow_timing();
            return;
        }
        contact_forces = wrench_solution.forces;
        wbc_shadow_diagnostics_.solver_ok = true;
        wbc_shadow_diagnostics_.wrench_satisfied =
            wrench_solution.wrench_satisfied;
        wbc_shadow_diagnostics_.constraint_feasible =
            wrench_solution.constraint_report.feasible;
        wbc_shadow_diagnostics_.iterations = wrench_solution.iterations;
        wbc_shadow_diagnostics_.residual_norm =
            wrench_solution.residual_norm;
        wbc_shadow_diagnostics_.task_satisfied =
            wrench_solution.task_satisfied;
        wbc_shadow_diagnostics_.task_residual_norm =
            wrench_solution.task_residual_norm;
        wbc_shadow_diagnostics_.max_axis_friction_ratio =
            wrench_solution.max_axis_friction_ratio;
        wbc_shadow_diagnostics_.max_radial_friction_ratio =
            wrench_solution.max_radial_friction_ratio;
        wbc_shadow_diagnostics_.min_contact_normal_force_n =
            wrench_solution.min_contact_normal_force;
    }

    go2_control::ContactTorqueMapRequest torque_request;
    torque_request.joint_angles = joint_angles;
    torque_request.contact_forces = contact_forces;
    torque_request.contact = request.wrench.contact;
    go2_control::ContactTorqueMapSolution torque_solution;
    if (!go2_control::MapContactForcesToJointTorques(
            torque_request, torque_solution))
    {
        finish_shadow_timing();
        return;
    }

    wbc_shadow_diagnostics_.mapping_ok = true;
    wbc_shadow_diagnostics_.max_abs_tau =
        torque_solution.max_abs_torque;
    wbc_shadow_candidate_torques_ = torque_solution.torques;
    finish_shadow_timing();
}

// --- TrotExperiment::PrepareWbcTorqueFeedforward ---
bool TrotExperiment::PrepareWbcTorqueFeedforward(
    std::array<double, kMotorCount> &torque_ff)
{
    torque_ff.fill(0.0);
    wbc_shadow_diagnostics_.feedforward_ready = false;
    wbc_shadow_diagnostics_.feedforward_applied = false;
    wbc_shadow_diagnostics_.feedforward_reduced_task_gate = false;
    wbc_shadow_diagnostics_.feedforward_max_abs_tau = 0.0;
    wbc_shadow_diagnostics_.feedforward_gate_code =
        static_cast<int>(go2_control::WbcFeedforwardGateCode::kDisabled);

    const bool reduced_task_gate =
        params_.wbc_task_torque_feedforward &&
        params_.wbc_reduced_contact_task &&
        wbc_shadow_diagnostics_.reduced_contact_task &&
        wbc_shadow_diagnostics_.task_satisfied;
    wbc_shadow_diagnostics_.feedforward_reduced_task_gate =
        reduced_task_gate && !wbc_shadow_diagnostics_.wrench_satisfied;

    go2_control::WbcFeedforwardGateInput gate_input;
    gate_input.requested = params_.wbc_torque_feedforward;
    gate_input.shadow_enabled =
        params_.wbc_shadow && wbc_shadow_diagnostics_.enabled;
    gate_input.locomotion_active =
        (task_.motion_stage_ == 2 && task_.gait_started_ &&
         !task_.stop_requested_) || WbcStopHoldActive();
    gate_input.solver_ok = wbc_shadow_diagnostics_.solver_ok;
    gate_input.mapping_ok = wbc_shadow_diagnostics_.mapping_ok;
    gate_input.wrench_satisfied =
        wbc_shadow_diagnostics_.wrench_satisfied;
    gate_input.reduced_task_gate = reduced_task_gate;
    gate_input.constraint_feasible =
        wbc_shadow_diagnostics_.constraint_feasible;
    gate_input.active_contacts = wbc_shadow_diagnostics_.active_contacts;
    gate_input.minimum_contacts = kMinimumSupportContacts;
    gate_input.within_budget = wbc_shadow_diagnostics_.within_budget;
    gate_input.torque_scale = params_.wbc_torque_scale;
    gate_input.max_torque_scale = kWbcTorqueFeedforwardMaxScale;

    auto gate_result =
        go2_control::EvaluateWbcFeedforwardGate(gate_input);
    wbc_shadow_diagnostics_.feedforward_gate_code =
        static_cast<int>(gate_result.code);
    if (!gate_result.ready)
        return false;

    double max_abs_torque = 0.0;
    bool candidate_values_finite = true;
    bool scaled_candidate_within_limit = true;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        for (std::size_t joint = 0; joint < 3; ++joint)
        {
            const double candidate =
                wbc_shadow_candidate_torques_[leg][joint];
            const double scaled = params_.wbc_torque_scale * candidate;
            if (!std::isfinite(candidate))
                candidate_values_finite = false;
            if (!std::isfinite(scaled) ||
                std::abs(scaled) > kWbcTorqueFeedforwardMaxAbsNm)
                scaled_candidate_within_limit = false;
            if (!candidate_values_finite ||
                !scaled_candidate_within_limit)
                break;
            torque_ff[3 * leg + joint] = scaled;
            max_abs_torque = std::max(max_abs_torque, std::abs(scaled));
        }
        if (!candidate_values_finite ||
            !scaled_candidate_within_limit)
            break;
    }

    if (!candidate_values_finite || !scaled_candidate_within_limit)
    {
        gate_input.candidate_values_finite = candidate_values_finite;
        gate_input.scaled_candidate_within_limit =
            scaled_candidate_within_limit;
        gate_result =
            go2_control::EvaluateWbcFeedforwardGate(gate_input);
        wbc_shadow_diagnostics_.feedforward_gate_code =
            static_cast<int>(gate_result.code);
        torque_ff.fill(0.0);
        return false;
    }

    wbc_shadow_diagnostics_.feedforward_gate_code =
        static_cast<int>(go2_control::WbcFeedforwardGateCode::kReady);
    wbc_shadow_diagnostics_.feedforward_ready = true;
    wbc_shadow_diagnostics_.feedforward_max_abs_tau = max_abs_torque;
    return true;
}
