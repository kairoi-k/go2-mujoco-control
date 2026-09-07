#pragma once
#include "go2_rigid_body.h"
#include <algorithm>
namespace go2_trot {
struct MotorCommandSample {
 double q=0,dq=0,kp=0,kd=0,tau=0;
};
struct MotorCommandCertificate {
 bool input_valid=false;
 bool within_model_envelope=false;
 std::array<double,12> requested_torque_nm{};
 std::array<double,12> predicted_applied_torque_nm{};
 double maximum_saturation_nm=0;
 // This predicts the bridge/model composition at the supplied state only.
 // It is not a measurement of a later simulator tick or a dynamics proof.
};
inline MotorCommandCertificate VerifyMotorCommandComposition(
 const go2_control::Go2RigidBody &robot,
 const std::array<MotorCommandSample,12> &commands,
 const go2_control::RigidBodyState &state) {
 MotorCommandCertificate out;
 if(!state.q.allFinite() || !state.dq.allFinite()) return out;
 for(int motor=0;motor<12;++motor) {
  const auto &c=commands[motor];double low=0,high=0;
  if(!std::isfinite(c.q) || !std::isfinite(c.dq) ||
     !std::isfinite(c.kp) || !std::isfinite(c.kd) || !std::isfinite(c.tau) ||
     c.kp<0 || c.kd<0 || !robot.MotorTorqueEnvelope(motor,low,high)) return out;
  const double requested=c.tau+c.kp*(c.q-state.q[motor])+c.kd*(c.dq-state.dq[motor]);
  if(!std::isfinite(requested))return out;
  const double applied=std::clamp(requested,low,high);
  out.requested_torque_nm[motor]=requested;
  out.predicted_applied_torque_nm[motor]=applied;
  out.maximum_saturation_nm=std::max(out.maximum_saturation_nm,std::abs(requested-applied));
 }
 out.input_valid=true;out.within_model_envelope=out.maximum_saturation_nm<=1e-9;
 return out;
}
} // namespace
