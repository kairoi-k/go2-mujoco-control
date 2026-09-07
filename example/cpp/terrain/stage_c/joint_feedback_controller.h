#pragma once
// One diagnostic feedback tick. This header has no thread, plant, scheduler,
// solver, or motor-write owner. It consumes an already selected proposal and
// an owner-sampled foot reference, then returns a torque only when every
// solver, independent ID-WBC, and motor-envelope gate passes.
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <Eigen/Dense>
#include "joint_feedback_reference.h"
#include "id_wbc_certificate.h"
#include "motor_command_certificate.h"
namespace go2_terrain {
namespace stage_c {
namespace joint_feedback_controller {
using MeasuredContactMask = std::array<bool, go2::kLegCount>;
using JointFeedbackReferenceConfig =
    joint_feedback_reference::ClosedLoopResearchConfig;
struct JointFeedbackTickResult {
  bool ok = false;
  bool tau_valid = false;
  bool solver_returned = false;
  bool id_certificate_valid = false;
  bool motor_mapping_valid = false;
  bool motor_envelope_valid = false;
  TimeNs time{};
  std::string failure = "not_run";
  int qp_iterations = 0;
  double com_error_m = std::numeric_limits<double>::quiet_NaN();
  double max_foot_error_m = std::numeric_limits<double>::quiet_NaN();
  std::size_t feedback_clipped_count = 0;
  bool orientation_clipped = false;
  Eigen::Vector3d orientation_acc_body =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Matrix<double, go2::kJointCount, 1> tau_joint_row =
      Eigen::Matrix<double, go2::kJointCount, 1>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  // Motor-name order, suitable for a caller's final command adapter only
  // after tau_valid is true. It is NaN on every failure path.
  Eigen::Matrix<double, go2::kJointCount, 1> tau =
      Eigen::Matrix<double, go2::kJointCount, 1>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  go2_control::IdWbcParams params{};
  go2_control::IdWbcInput wbc_input{};
  go2_control::IdWbcOutput wbc{};
  go2_control::IdWbcPhysicalCertificate id_certificate{};
  go2_trot::MotorCommandCertificate motor_certificate{};
};
inline bool ValidateTickProposal(
    const CentroidalJointProposal &proposal, TimeNs time,
    const go2_control::RigidBodyState &state, std::string &failure) {
  if (!proposal.selected_valid || !proposal.search.feasible) {
    failure = "proposal_not_valid";
    return false;
  }
  if (time.value < 0 || !joint_feedback_reference::FiniteState(state)) {
    failure = "tick_state_or_time_invalid";
    return false;
  }
  const auto &problem = proposal.selected_problem;
  const auto &result = proposal.selected_result;
  const auto &input = problem.request.input;
  if (!input.basic_valid() || !input.identity.valid() ||
      problem.schedule.empty() || problem.grid.size() < 2 ||
      result.states.size() != problem.grid.size() ||
      result.forces.size() + 1 != problem.grid.size() ||
      !result.certificate.feasible || !result.certificate.input_checked ||
      !result.certificate.original_dynamics_checked) {
    failure = "proposal_shape_or_certificate_invalid";
    return false;
  }
  if (time < problem.grid.front() || time >= problem.grid.back()) {
    failure = "tick_outside_proposal_coverage";
    return false;
  }
  return true;
}
inline JointFeedbackTickResult FeedbackTick(
    const CentroidalJointProposal &proposal,
    go2_control::Go2RigidBody &robot,
    const go2_control::RigidBodyState &actual_state,
    const FootTrajectorySample &feet,
    const MeasuredContactMask &measured_contact,
    TimeNs time, double initial_yaw,
    const JointFeedbackReferenceConfig &config = {},
    const go2_control::IdWbcPhysicalCertificateThresholds &thresholds = {}) {
  JointFeedbackTickResult out;
  out.time = time;
  auto fail = [&](const char *why) {
    out.failure = why;
    out.tau_valid = false;
    out.ok = false;
    out.tau_joint_row.setConstant(std::numeric_limits<double>::quiet_NaN());
    out.tau.setConstant(std::numeric_limits<double>::quiet_NaN());
    return out;
  };
  std::string failure;
  if (!joint_feedback_reference::ValidClosedLoopConfig(config) ||
      !std::isfinite(initial_yaw) ||
      !ValidateTickProposal(proposal, time, actual_state, failure)) {
    return fail(failure.empty() ? "invalid_tick_input" : failure.c_str());
  }
  if (!feet.valid || feet.time != time) return fail("foot_sample_time_mismatch");
  go2_control::RigidBodyPlanningKinematics actual;
  if (!robot.EvaluatePlanningKinematics(actual_state, actual) ||
      !actual.valid || !actual.dynamics.valid) {
    return fail("actual_kinematics_unavailable");
  }
  const auto &problem = proposal.selected_problem;
  const auto &result = proposal.selected_result;
  Eigen::Matrix<double, 6, 1> desired =
      Eigen::Matrix<double, 6, 1>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  Eigen::Matrix<double, 6, 1> weights =
      Eigen::Matrix<double, 6, 1>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  ContactForceInterval planned_force{};
  CentroidalState planned_state =
      CentroidalState::Constant(std::numeric_limits<double>::quiet_NaN());
  if (!joint_feedback_reference::BuildCentroidalReference(
          problem, result, time, actual, config, desired, weights,
          planned_force, planned_state, failure)) {
    return fail(failure.empty() ? "centroidal_reference_failed" : failure.c_str());
  }
  out.com_error_m = (planned_state.head<3>()-actual.dynamics.com_world).norm();
  out.max_foot_error_m = 0.0;
  for (std::size_t leg=0; leg<go2::kLegCount; ++leg)
    out.max_foot_error_m = std::max(out.max_foot_error_m,
        (joint_feedback_reference::Vec(feet.center_world[leg].value)-actual.dynamics.foot_pos_world[leg]).norm());
  const auto *nominal = joint_feedback_reference::FindSchedule(problem, time);
  if (nominal == nullptr) return fail("schedule_interval_missing");
  go2_control::IdWbcParams params;
  go2_control::IdWbcInput input;
  std::array<Eigen::Vector3d, go2::kLegCount> application_points{};
  std::size_t clipped = 0;
  Eigen::Vector3d orientation_acc =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  bool orientation_clipped = false;
  if (!joint_feedback_reference::BuildWbcReplayInput(
          problem, robot, actual_state, actual, feet, planned_force, desired,
          weights, *nominal, measured_contact, initial_yaw, config, params,
          input, application_points, clipped, orientation_acc,
          orientation_clipped, failure)) {
    return fail(failure.empty() ? "wbc_input_failed" : failure.c_str());
  }
  out.params = params;
  out.wbc_input = input;
  out.feedback_clipped_count = clipped;
  out.orientation_clipped = orientation_clipped;
  out.orientation_acc_body = orientation_acc;
  go2_control::IdWbcOutput wbc;
  const bool call_ok =
      go2_control::SolveInverseDynamicsWbc(params, input, wbc);
  out.wbc = wbc;
  out.qp_iterations = wbc.iterations;
  out.solver_returned = call_ok && wbc.ok;
  if (wbc.solution_finite) {
    out.id_certificate =
        go2_control::VerifyIdWbcPhysicalCertificate(
            params, input, wbc, thresholds);
    out.id_certificate_valid = out.id_certificate.feasible;
  }
  bool mapping_valid = false;
  std::array<go2_trot::MotorCommandSample, go2::kJointCount> commands{};
  if (wbc.solution_finite && wbc.tau.allFinite()) {
    mapping_valid = true;
    for (std::size_t motor = 0; motor < go2::kJointCount; ++motor) {
      const int dof = robot.MotorDof(static_cast<int>(motor));
      const int joint_row = dof - go2_control::kFloatingNv;
      if (joint_row < 0 ||
          joint_row >= static_cast<int>(go2::kJointCount)) {
        mapping_valid = false;
        break;
      }
      commands[motor].q = actual_state.q[motor];
      commands[motor].dq = actual_state.dq[motor];
      commands[motor].kp = 0.0;
      commands[motor].kd = 0.0;
      commands[motor].tau = wbc.tau[joint_row];
    }
  }
  out.motor_mapping_valid = mapping_valid;
  if (mapping_valid) {
    out.motor_certificate =
        go2_trot::VerifyMotorCommandComposition(robot, commands, actual_state);
    out.motor_envelope_valid =
        out.motor_certificate.input_valid &&
        out.motor_certificate.within_model_envelope;
  }
  // A finite solver iterate and even a feasible independent certificate do
  // not authorize a command when the solver call or motor mapping failed.
  if (!out.solver_returned) return fail("wbc_solver_failed");
  if (!out.id_certificate_valid) return fail("id_wbc_certificate_failed");
  if (!out.motor_mapping_valid || !out.motor_certificate.input_valid)
    return fail("motor_command_mapping_invalid");
  if (!out.motor_envelope_valid) return fail("motor_envelope_violation");
  out.tau_joint_row = wbc.tau;
  for (std::size_t motor = 0; motor < go2::kJointCount; ++motor)
    out.tau[static_cast<int>(motor)] =
        out.motor_certificate.requested_torque_nm[motor];
  if (!out.tau.allFinite() || !out.tau_joint_row.allFinite())
    return fail("final_torque_nonfinite");
  out.tau_valid = true;
  out.ok = true;
  out.failure = "ok";
  return out;
}
}  // namespace joint_feedback_controller
}  // namespace stage_c
}  // namespace go2_terrain
