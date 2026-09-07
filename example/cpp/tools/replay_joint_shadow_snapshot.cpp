#include "stage_c/terminal_swing_binding.h"
#include "../trot/joint_planning_shadow.h"
#include "stage_c/joint_closed_loop_replay.h"
#include "stage_c/joint_trajectory.h"
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
struct Reader {
 std::ifstream stream;
 explicit Reader(const char *p):stream(p){if(!stream)throw std::runtime_error("snapshot unavailable");}
 std::string word(){std::string s;if(!(stream>>s))throw std::runtime_error("truncated snapshot");return s;}
 double number(){auto s=word();std::size_t end=0;double x=std::stod(s,&end);if(end!=s.size())throw std::runtime_error("bad number");return x;}
 std::uint64_t integer(){auto s=word();std::size_t end=0;if(s.empty()||s.front()=='-')throw std::runtime_error("bad integer");auto x=std::stoull(s,&end);if(end!=s.size())throw std::runtime_error("bad integer");return x;}
 bool boolean(){auto x=integer();if(x>1)throw std::runtime_error("bad bool");return x!=0;}
 template<class V>void vector(V&v,int n){for(int j=0;j<n;++j)v[j]=number();}
};
go2_terrain::TerrainModel ReadTerrainModel(Reader &r) {
 go2_terrain::TerrainModel m;m.frame_id=r.word();m.source=static_cast<go2_terrain::TerrainSource>(r.integer());m.epoch=r.integer();m.registered=r.boolean();m.map_sequence=r.integer();
 m.state_stamp_s=r.number();m.map_stamp_s=r.number();m.age_s=r.number();m.width=r.integer();m.height=r.integer();m.resolution_m=r.number();
 if(!m.width||!m.height||m.width>1000000/m.height)throw std::runtime_error("invalid cell count");
 r.vector(m.origin_m,2);r.vector(m.registration_position_world,3);m.registration_yaw_rad=r.number();r.vector(m.capture_position_world,3);m.capture_yaw_rad=r.number();
 m.cells.resize(m.width*m.height);for(auto&c:m.cells){c.known=r.boolean();c.height_m=r.number();c.has_height_bounds=r.boolean();c.height_min_m=r.number();c.height_max_m=r.number();c.age_s=r.number();c.slope_rad=r.number();c.roughness_m=r.number();c.variance_m2=r.number();r.vector(c.normal,3);}
 return m;
}
namespace articulated_audit_detail {
void Number(double value) {
 std::cout << std::setprecision(17);
 if (std::isfinite(value)) std::cout << value; else std::cout << "null";
}
void Bool(bool value) { std::cout << (value ? "true" : "false"); }
void String(const std::string &value) {
 std::cout << '"';
 for (const char c : value) {
  if (c == '\\' || c == '"') std::cout << '\\' << c;
  else if (c == '\n') std::cout << "\\n";
  else if (c == '\r') std::cout << "\\r";
  else std::cout << c;
 }
 std::cout << '"';
}
void PrintCertificate(const go2_terrain::stage_c::JointTrajectorySample &sample,
                      std::size_t index) {
 using namespace go2_terrain::stage_c;
 const auto &c = sample.model_certificate;
 const auto &d = c.dynamics;
 std::cout << "{\"index\":" << index << ",\"time_s\":";
 Number(sample.time.seconds());
 std::cout << ",\"force_interval_valid\":"; Bool(sample.force_interval_valid);
 std::cout << ",\"qacc_norm_mixed_units\":"; Number(sample.qacc.norm());
 std::cout << ",\"torque_max_nm\":"; Number(sample.torque.cwiseAbs().maxCoeff());
 std::cout << ",\"sample_feasible\":"; Bool(c.sample_feasible);
 std::cout << ",\"failure\":"; String(JointPlannerFailureName(c.failure));
 std::cout << ",\"dynamics_certificate\":{\"checked\":"; Bool(d.checked);
 std::cout << ",\"input_valid\":"; Bool(d.input_valid);
 std::cout << ",\"valid\":"; Bool(d.valid);
 std::cout << ",\"feasible\":"; Bool(d.feasible);
 std::cout << ",\"failure_bitmask\":" << d.failure_bitmask;
 std::cout << ",\"force_application_jacobian_used\":"; Bool(d.force_application_jacobian_used);
 std::cout << ",\"dynamics_residual_norm\":"; Number(d.dynamics_residual_norm);
 std::cout << ",\"max_dynamics_force_residual_N\":"; Number(d.max_dynamics_force_residual_N);
 std::cout << ",\"max_dynamics_moment_residual_Nm\":"; Number(d.max_dynamics_moment_residual_Nm);
 std::cout << ",\"max_joint_dynamics_residual_Nm\":"; Number(d.max_joint_dynamics_residual_Nm);
 std::cout << ",\"max_normal_violation_N\":"; Number(d.max_normal_violation_N);
 std::cout << ",\"max_friction_violation_N\":"; Number(d.max_friction_violation_N);
 std::cout << ",\"max_swing_force_violation_N\":"; Number(d.max_swing_force_violation_N);
 std::cout << ",\"max_tau_violation_Nm\":"; Number(d.max_tau_violation_Nm);
 std::cout << ",\"hard_stance_acc_residual_mps2\":"; Number(d.hard_stance_acc_residual_mps2);
 std::cout << "},\"max_surface_force_violation_N\":"; Number(c.max_surface_force_violation_n);
 std::cout << ",\"max_joint_position_violation_rad\":"; Number(c.max_joint_position_violation_rad);
 std::cout << ",\"max_joint_velocity_violation_radps\":"; Number(c.max_joint_velocity_violation_radps);
 std::cout << '}';
}

struct FootCoverageAudit {
 bool attempted=false;
 bool valid=false;
 std::string failure="not_attempted";
 double start_s=std::numeric_limits<double>::quiet_NaN();
 double end_s=std::numeric_limits<double>::quiet_NaN();
};
FootCoverageAudit CheckFootCoverage(const go2_terrain::stage_c::FootTrajectoryRequest &request) {
 using namespace go2_terrain::stage_c;
 FootCoverageAudit out;out.attempted=true;
 out.start_s=request.start.seconds();out.end_s=request.end.seconds();
 if(request.end<=request.start) { out.failure=JointPlannerFailureName(JointPlannerFailure::kInvalidInput);return out; }
 std::vector<TimeNs> times{request.start};
 if(request.end.value-request.start.value>1) times.push_back(TimeNs{request.end.value-1});
 const auto sampled=SampleFootTrajectory(request,times);
 out.valid=sampled.valid;
 out.failure=JointPlannerFailureName(sampled.failure);
 return out;
}
void PrintCoverage(const FootCoverageAudit &coverage) {
 std::cout << "{\"attempted\":";Bool(coverage.attempted);
 std::cout << ",\"valid\":";Bool(coverage.valid);
 std::cout << ",\"failure\":";String(coverage.failure);
 std::cout << ",\"start_s\":";Number(coverage.start_s);
 std::cout << ",\"end_s\":";Number(coverage.end_s);std::cout<<'}';
}
struct LiftAudit {
 bool attempted=false;
 bool actual_kinematics_valid=false;
 Eigen::Vector3d runtime_momentum_target=Eigen::Vector3d::Zero();
 Eigen::Vector3d diagnostic_momentum_target=Eigen::Vector3d::Zero();
 bool qacc_valid=false;
 std::string failure="not_attempted";
 int map_rank=0;
 double map_rcond=0.0;
 double residual_inf_mps2=std::numeric_limits<double>::infinity();
 Eigen::Vector3d angular_acc_body=Eigen::Vector3d::Zero();
 double joint_acc_norm_radps2=std::numeric_limits<double>::quiet_NaN();
 bool initial_state_available=false;
 Eigen::Vector3d initial_position_delta=Eigen::Vector3d::Zero();
 Eigen::Vector3d initial_velocity_delta=Eigen::Vector3d::Zero();
 bool sample_certificate_available=false;
 go2_terrain::stage_c::JointTrajectorySample sample{};
};
void PrintVector3(const Eigen::Vector3d &value) {
 std::cout << '[';for(int i=0;i<3;++i){if(i)std::cout<<',';Number(value[i]);}std::cout<<']';
}
LiftAudit BuildLiftAudit(
    go2_control::Go2RigidBody &robot,const go2_control::RigidBodyState &initial,
    const go2_terrain::stage_c::CentroidalProblem &problem,
    const go2_terrain::stage_c::FootTrajectoryRequest &feet,
    const go2_terrain::stage_c::CentroidalResult &centroidal) {
 using namespace go2_terrain::stage_c;
 using namespace go2_terrain::stage_c::joint_feedback_reference;
 LiftAudit out;out.attempted=true;
 if(problem.grid.empty()) {out.failure=JointPlannerFailureName(JointPlannerFailure::kCoverageIncomplete);return out;}
 go2_control::RigidBodyPlanningKinematics actual;
 if(!robot.EvaluatePlanningKinematics(initial,actual) || !actual.valid || !actual.dynamics.valid) {
  out.failure=JointPlannerFailureName(JointPlannerFailure::kObservationUnavailable);return out;
 }
 out.actual_kinematics_valid=true;
 const TimeNs time=problem.grid.front();
 Eigen::Matrix<double,6,1> desired=Eigen::Matrix<double,6,1>::Constant(std::numeric_limits<double>::quiet_NaN());
 Eigen::Matrix<double,6,1> weights=Eigen::Matrix<double,6,1>::Constant(std::numeric_limits<double>::quiet_NaN());
 ContactForceInterval planned_force{};CentroidalState planned_state{};std::string reference_failure;
 ClosedLoopResearchConfig config;
 (void)weights;
 if(!BuildCentroidalReference(problem,centroidal,time,actual,config,desired,weights,planned_force,planned_state,reference_failure)) {
  out.failure=reference_failure.empty()?JointPlannerFailureName(JointPlannerFailure::kObservationUnavailable):reference_failure;return out;
 }
 const auto f=SampleFootTrajectoryAt(feet,time);
 if(!f.valid || f.samples.size()!=1) {
  out.failure=JointPlannerFailureName(f.failure);return out;
 }
 const auto &fs=f.samples.front();
 BodyReconstruction reconstruction;
 reconstruction.state=initial;reconstruction.model=actual;reconstruction.kinematics_valid=true;
 out.initial_state_available=true;
 out.initial_position_delta.setZero();out.initial_velocity_delta.setZero();
 const auto interval=std::find_if(problem.schedule.begin(),problem.schedule.end(),
  [&](const FixedScheduleInterval &s){return s.start<=time && time<s.end;});
 if(interval==problem.schedule.end()) {out.failure=JointPlannerFailureName(JointPlannerFailure::kCoverageIncomplete);return out;}
 BodyAccelerationTarget target;target.com_acceleration_valid=true;
 target.angular_momentum_derivative_valid=true;
 target.com_acceleration_world=desired.head<3>();
 target.angular_momentum_derivative_world=desired.tail<3>();
 target.foot_acceleration_valid=fs.leg_valid;
 std::array<Eigen::Vector3d,4> points;
 std::array<ContactSurface,4> surfaces{};
 Eigen::Vector3d actual_moment=Eigen::Vector3d::Zero();
 for(int leg=0;leg<4;++leg) {
  points[leg]=actual.dynamics.foot_pos_world[leg];
  target.foot_acceleration_world[leg]=Eigen::Vector3d(fs.acceleration_world[leg].x,fs.acceleration_world[leg].y,fs.acceleration_world[leg].z);
  if(!planned_force.contact[leg]) continue;
  ContactSurface surface;Eigen::Vector3d surface_point;
  if(!ResolveScheduleSurface(problem,*interval,leg,planned_force.end,surface_point,surface)) {
   out.failure=JointPlannerFailureName(JointPlannerFailure::kObservationUnavailable);return out;
  }
  const double radius=actual.dynamics.foot_geometry[leg].collision_radius_m;
  const Eigen::Vector3d normal=surface.basis_world.col(2);
  if(!normal.allFinite() || !std::isfinite(radius) || radius<=0.0) {
   out.failure=JointPlannerFailureName(JointPlannerFailure::kObservationUnavailable);return out;
  }
  // This is the actual force application point used by the model map, not
  // the nominal surface target. The surface point is retained only for its
  // normal/friction metadata.
  points[leg]=actual.dynamics.foot_pos_world[leg]-radius*normal;
  surfaces[leg]=surface;
  const Eigen::Vector3d force(Vec(planned_force.force_world[leg]));
  if(!force.allFinite()) {out.failure=JointPlannerFailureName(JointPlannerFailure::kNumericalFailure);return out;}
  actual_moment+=(points[leg]-actual.dynamics.com_world).cross(force);
 }
 // BuildCentroidalReference supplies COM feedback and momentum feedback from
 // the same proposal. Recompute only the angular moment arm for the actual
 // application points used by LiftBodyAcceleration/VerifyArticulatedSample.
 target.angular_momentum_derivative_world=actual_moment+
  config.momentum_kp*(planned_state.tail<3>()-actual.angular_momentum_world);
 out.runtime_momentum_target=desired.tail<3>();
 out.diagnostic_momentum_target=target.angular_momentum_derivative_world;
 const auto lift=LiftBodyAcceleration(robot,reconstruction,target);
 out.failure=JointPlannerFailureName(lift.failure);out.map_rank=lift.map_rank;out.map_rcond=lift.map_rcond;
 out.residual_inf_mps2=lift.residual_inf;out.qacc_valid=lift.valid;
 if(!lift.valid) return out;
 out.angular_acc_body=lift.qacc.segment<3>(3);out.joint_acc_norm_radps2=lift.qacc.tail<12>().norm();
 out.sample_certificate_available=true;out.sample.time=time;out.sample.state=initial;
 out.sample.qacc=lift.qacc;out.sample.force=planned_force;out.sample.force_interval_valid=true;
 out.sample.model_certificate=VerifyArticulatedSample(robot,initial,lift.qacc,planned_force,points,surfaces,35.0,30.0);
 out.sample.torque=out.sample.model_certificate.torque;
 return out;
}
void PrintLift(const LiftAudit &lift) {
 std::cout << "{\"attempted\":";Bool(lift.attempted);
 std::cout << ",\"actual_state_lift_only\":true,\"source_state_unmodified\":true";
 std::cout << ",\"failure\":";String(lift.failure);
 std::cout << ",\"actual_kinematics_valid\":";Bool(lift.actual_kinematics_valid);
 std::cout << ",\"target_semantics\":\"actual lever arms replace runtime planned moment; nominal foot acceleration without feedback; not runtime task replay\"";
 std::cout << ",\"runtime_momentum_target_nm\":";PrintVector3(lift.runtime_momentum_target);
 std::cout << ",\"diagnostic_momentum_target_nm\":";PrintVector3(lift.diagnostic_momentum_target);
 std::cout << ",\"qacc_valid\":";Bool(lift.qacc_valid);
 std::cout << ",\"map_rank\":"<<lift.map_rank<<",\"map_rcond\":";Number(lift.map_rcond);
 std::cout << ",\"residual_inf_mps2\":";Number(lift.residual_inf_mps2);
 std::cout << ",\"angular_acc_body_radps2\":";PrintVector3(lift.angular_acc_body);
 std::cout << ",\"joint_acc_norm_radps2\":";Number(lift.joint_acc_norm_radps2);
 std::cout << ",\"initial_state_retained\":";
 if(!lift.initial_state_available) std::cout<<"null";
 else {std::cout<<"{\"position_delta_world_m\":";PrintVector3(lift.initial_position_delta);
  std::cout<<",\"linear_velocity_delta_world_mps\":";PrintVector3(lift.initial_velocity_delta);std::cout<<'}';}
 std::cout << ",\"articulated_sample_certificate\":";
 if(!lift.sample_certificate_available) std::cout<<"null";else PrintCertificate(lift.sample,0);
 std::cout<<'}';
}
void PrintAudit(const go2_control::RigidBodyState &initial, double source_time_s,
                const std::shared_ptr<const go2_terrain::stage_c::joint_execution::JointExecutionProposal> &proposal,
                const std::string &proposal_failure,
                const go2_terrain::stage_c::JointTrajectoryResult *result,
                const FootCoverageAudit *runtime_coverage=nullptr,
                const FootCoverageAudit *full_coverage=nullptr,
                const LiftAudit *lift=nullptr) {
 using namespace go2_terrain::stage_c;
 std::cout << "{\"articulated_audit\":true,\"scope\":\"same selected proposal and foot request; no mj_step; diagnostic only\"";
 std::cout << ",\"source_time_s\":"; Number(source_time_s);
 std::cout << ",\"proposal_valid\":"; Bool(static_cast<bool>(proposal));
 std::cout << ",\"proposal_failure\":"; String(proposal_failure);
 if (!proposal) { std::cout << ",\"result\":null}\n"; return; }
 std::cout << ",\"proposal_id\":" << proposal->proposal_id;
 std::cout << ",\"foot_request_start_s\":"; Number(proposal->foot_request.start.seconds());
 std::cout << ",\"foot_request_end_s\":"; Number(proposal->foot_request.end.seconds());
 std::cout << ",\"candidate_indices\":[";
 const auto &indices = proposal->selected->search.plan.candidate_indices;
 for (std::size_t i=0;i<indices.size();++i) { if(i) std::cout << ','; std::cout << indices[i]; }
 std::cout << ']';
 std::cout << ",\"runtime_foot_request_coverage\":";
 if(runtime_coverage==nullptr) std::cout<<"null";else PrintCoverage(*runtime_coverage);
 std::cout << ",\"full_grid_foot_request_coverage\":";
 if(full_coverage==nullptr) std::cout<<"null";else PrintCoverage(*full_coverage);
 if (result == nullptr) { std::cout << ",\"result\":null}\n"; return; }
 const auto &r = *result;
 std::cout << ",\"result\":{\"failure\":"; String(JointPlannerFailureName(r.failure));
 std::cout << ",\"centroidal_detail\":"; String(r.centroidal.detail);
 std::cout << ",\"model_samples_verified\":"; Bool(r.model_samples_verified);
 std::cout << ",\"execution_ready\":"; Bool(r.execution_ready);
 std::cout << ",\"sample_count\":" << r.samples.size();
 std::cout << ",\"centroidal_certificate\":{\"feasible\":"; Bool(r.centroidal.certificate.feasible);
 std::cout << ",\"input_checked\":"; Bool(r.centroidal.certificate.input_checked);
 std::cout << ",\"coverage_checked\":"; Bool(r.centroidal.certificate.coverage_checked);
 std::cout << ",\"original_dynamics_checked\":"; Bool(r.centroidal.certificate.original_dynamics_checked);
 std::cout << ",\"position_residual_m\":"; Number(r.centroidal.certificate.residual.position_m);
 std::cout << ",\"velocity_residual_mps\":"; Number(r.centroidal.certificate.residual.velocity_mps);
 std::cout << ",\"momentum_residual_nms\":"; Number(r.centroidal.certificate.residual.momentum_nms);
 std::cout << "},\"body_reconstruction\":{\"kinematics_valid\":"; Bool(r.body.kinematics_valid);
 std::cout << ",\"failure\":"; String(JointPlannerFailureName(r.body.failure));
 std::cout << ",\"knot_count\":" << r.body.knots.size();
 std::cout << ",\"max_position_residual_m\":"; Number(r.body.max_position_residual_m);
 std::cout << ",\"max_velocity_residual_mps\":"; Number(r.body.max_velocity_residual_mps);
 std::cout << ",\"max_momentum_residual_nms\":"; Number(r.body.max_momentum_residual_nms);
 std::cout << ",\"initial_state_retained\":";
 if (r.body.knots.empty()) { std::cout << "null"; }
 else {
  const auto &s = r.body.knots.front().state;
  const double position = (s.position_world-initial.position_world).lpNorm<Eigen::Infinity>();
  const double q = (s.q-initial.q).lpNorm<Eigen::Infinity>();
  const double linear = (s.linear_vel_world-initial.linear_vel_world).lpNorm<Eigen::Infinity>();
  const double angular = (s.angular_vel_body-initial.angular_vel_body).lpNorm<Eigen::Infinity>();
  const double dq = (s.dq-initial.dq).lpNorm<Eigen::Infinity>();
  const double quat = s.quat_world_from_body.angularDistance(initial.quat_world_from_body);
  std::cout << "{\"position_inf_m\":"; Number(position);
  std::cout << ",\"q_inf_rad\":"; Number(q);
  std::cout << ",\"linear_velocity_inf_mps\":"; Number(linear);
  std::cout << ",\"angular_velocity_inf_radps\":"; Number(angular);
  std::cout << ",\"dq_inf_radps\":"; Number(dq);
  std::cout << ",\"quaternion_angle_rad\":"; Number(quat);
  std::cout << ",\"within_reconstruction_initial_tolerances\":";
  Bool(position<=1e-7 && q<=1e-6 && linear<=1e-7 && angular<=1e-7 && dq<=1e-6 && quat<=1e-8);
  std::cout << '}';
 }
 std::cout << "},\"sample_certificate\":";
 int sample_index=-1;
 for (std::size_t i=0;i<r.samples.size();++i) if (!r.samples[i].model_certificate.sample_feasible) { sample_index=static_cast<int>(i); break; }
 if (sample_index<0 && !r.samples.empty()) sample_index=static_cast<int>(r.samples.size()-1);
 if (sample_index<0) std::cout << "null";
 else PrintCertificate(r.samples[static_cast<std::size_t>(sample_index)],static_cast<std::size_t>(sample_index));
 std::cout << ",\"lift_diagnostic\":";
 if(lift==nullptr) std::cout<<"null";else PrintLift(*lift);
 std::cout << "}}\n";
}
}  // namespace articulated_audit_detail
int main(int argc,char**argv){try {
 const bool roundtrip=argc==3 && std::string(argv[2])=="--roundtrip";
 const bool terminal_audit=argc==5 && std::string(argv[2])=="--articulated-tail-audit";
 const bool articulated_audit=argc==3 && std::string(argv[2])=="--articulated-audit";
 const bool closed_loop=argc==5 && std::string(argv[2])=="--closed-loop";
 if(argc!=2 && !roundtrip && !articulated_audit && !closed_loop && !terminal_audit)throw std::runtime_error("usage: replay_joint_shadow_snapshot EXTRACTED_SNAPSHOT [--roundtrip | --articulated-audit | --articulated-tail-audit PREDICTION_END_S CHOICES_CSV | --closed-loop SCENE NEW_OUTPUT_CSV]");
 Reader r(argv[1]);const auto schema=r.word();const bool has_history=schema=="joint-shadow-snapshot-v2";
 if(schema!="joint-shadow-snapshot-v1" && !has_history)throw std::runtime_error("unsupported snapshot");
 const auto id=r.integer();const auto pattern=r.integer();if(pattern>static_cast<unsigned>(go2_control::GaitPattern::kRunningTrot))throw std::runtime_error("invalid pattern");
 go2_terrain::TerrainPlannerInput input;input.state_stamp_s=r.number();input.gait_phase=r.number();input.gait_period_s=r.number();input.duty_factor=r.number();input.commanded_vx_mps=r.number();input.base_yaw_rad=r.number();
 go2_control::RigidBodyState state;r.vector(state.position_world,3);std::array<double,4>q;r.vector(q,4);state.quat_world_from_body=Eigen::Quaterniond(q[0],q[1],q[2],q[3]);
 r.vector(state.linear_vel_world,3);r.vector(state.angular_vel_body,3);r.vector(state.q,12);r.vector(state.dq,12);
 input.contact_schedule.measured_valid=r.boolean();for(auto&b:input.contact_schedule.measured_contact)b=r.boolean();
 input.touchdown_target_feet_valid=r.boolean();for(auto&f:input.touchdown_target_feet_base){f.x=r.number();f.y=r.number();f.z=r.number();}
 auto m=ReadTerrainModel(r);
 go2_terrain::stage_c::WorldTerrainSnapshot history;
 if(has_history){
  using namespace go2_terrain;using namespace stage_c;
  const auto epoch=r.integer();const double stamp=r.number();WorldTerrainSnapshotOptions options;
  options.stationary_terrain_assumption=r.boolean();options.height_conflict_tolerance_m=r.number();options.minimum_normal_dot=r.number();
  const auto count=r.integer();if(!count || count>kWorldTerrainSnapshotMaxCaptures)throw std::runtime_error("history count");
  std::vector<CaptureTerrainViewResult> views;
  for(std::uint64_t i=0;i<count;++i){CaptureTerrainViewResult view;view.model=ReadTerrainModel(r);
   const auto &v=view.model;auto &p=view.provenance;
   p.source_sequence=v.map_sequence;p.map_epoch=v.epoch;p.source_map_stamp_s=v.map_stamp_s;p.state_stamp_s=v.state_stamp_s;
   p.capture_position_world=v.capture_position_world;p.capture_yaw_rad=v.capture_yaw_rad;p.source=v.source;
   for(const auto&c:v.cells)view.source_known_cells+=c.known;view.view_known_cells=view.source_known_cells;
   view.coverage_preserved=true;view.valid=true;view.error=CaptureTerrainViewError::kNone;views.push_back(std::move(view));}
  const auto built=BuildWorldTerrainSnapshot(epoch,stamp,std::move(views),options);
  if(!built.ok())throw std::runtime_error("history metadata invalid");history=built.snapshot;
  std::ostringstream original,latest;original.precision(17);latest.precision(17);
  go2_trot::AppendJointTerrainModelJson(original,&m);go2_trot::AppendJointTerrainModelJson(latest,history.latest_model());
  if(original.str()!=latest.str() || stamp!=input.state_stamp_s)throw std::runtime_error("history latest/state mismatch");
 }
 std::string extra;if(r.stream>>extra)throw std::runtime_error("extra snapshot fields");input.terrain=&m;
 if(!state.position_world.allFinite() || !state.linear_vel_world.allFinite() || !state.angular_vel_body.allFinite() || !state.q.allFinite() || !state.dq.allFinite() || !state.quat_world_from_body.coeffs().allFinite() || state.quat_world_from_body.norm()<1e-12)throw std::runtime_error("invalid actual state");
 if(roundtrip){std::cout<<go2_trot::JointShadowSnapshotJson(state,input,id,static_cast<int>(pattern),has_history?&history:nullptr)<<"\n";return 0;}
 go2_trot::JointPlanningShadow shadow;if(!shadow.Load(GO2_MODEL_PATH))throw std::runtime_error("model load");
 shadow.Capture(state,input,id,static_cast<go2_control::GaitPattern>(pattern),has_history?&history:nullptr);
 if(articulated_audit || terminal_audit){
  using namespace go2_terrain::stage_c;
  std::string proposal_failure;
  const auto proposal=shadow.BuildExecutionProposal(proposal_failure);
  if(!proposal){articulated_audit_detail::PrintAudit(state,input.state_stamp_s,proposal,proposal_failure,nullptr);return 2;}
  const auto &selected_problem=proposal->selected->selected_problem;
  const auto runtime_coverage=articulated_audit_detail::CheckFootCoverage(proposal->foot_request);
  auto full_grid_request=proposal->foot_request;
  if(!selected_problem.grid.empty()) full_grid_request.end=selected_problem.grid.back();
  if(terminal_audit) {
   std::size_t parsed=0;const std::string until_text=argv[3];
   const double until_s=std::stod(until_text,&parsed);
   if(parsed!=until_text.size() || !std::isfinite(until_s) ||
      until_s<full_grid_request.end.seconds() || until_s>input.state_stamp_s+2.0)
    throw std::runtime_error("invalid explicit stationary terrain prediction end");
   std::vector<std::size_t> choices;std::istringstream csv(argv[4]);std::string part;
   while(std::getline(csv,part,',')) {
    if(part.empty() || part.find_first_not_of("0123456789")!=std::string::npos)
     throw std::runtime_error("invalid explicit candidate index");
    choices.push_back(std::stoull(part));
   }
   PhaseClock audit_clock;PhaseClockObservation observation;
   observation.observation_time=TimeNs::FromSeconds(input.state_stamp_s);
   observation.phase=input.gait_phase;observation.period_s=input.gait_period_s;
   observation.duty_factor=input.duty_factor;
   observation.leg_offsets=CaptureGaitOffsets(static_cast<go2_control::GaitPattern>(pattern));
   if(!audit_clock.Capture(observation).accepted)throw std::runtime_error("tail phase unavailable");
   const auto phase=audit_clock.snapshot();
   if(phase.epoch!=selected_problem.request.input.identity.schedule_epoch)
    throw std::runtime_error("tail phase epoch mismatch");
   FixedSchedulePreviewRequest timing{full_grid_request.start,
    TimeNs{full_grid_request.end.value+TimeNs::FromSeconds(phase.period_s).value},
    phase.origin_time,TimeNs::FromSeconds(phase.period_s),TimeNs{20000000},
    phase.epoch,phase.duty_factor,phase.leg_offsets};
   const auto extended=BuildFixedSchedulePreview(timing);
   if(!extended.complete)throw std::runtime_error("tail preview incomplete");
   TouchdownEventTable tails;
   for(const auto &event:extended.events.events)
    if(event.touchdown_time>=full_grid_request.end && event.liftoff_time<full_grid_request.end)
     tails.events.push_back(event);
   go2_control::Go2RigidBody tail_robot;
   go2_control::RigidBodyPlanningKinematics model;
   if(!tail_robot.Load(GO2_MODEL_PATH) || !tail_robot.EvaluatePlanningKinematics(state,model))
    throw std::runtime_error("tail actual model unavailable");
   const auto reference=go2_trot::BuildJointTerrainCandidateReference(state,input,model);
   TerrainCandidateConfig config;config.allow_registered_heading_frame=true;
   config.allow_contact_continuation_beyond_horizon=true;
   const auto candidates=GenerateTerrainCandidates(m,tails,full_grid_request.start,m.epoch,
    reference,TimeNs::FromSeconds(until_s),config,has_history?&history:nullptr);
   const auto binding=BindTerminalSwingCandidates(full_grid_request,tails,candidates,choices);
   std::cout<<"{\"terminal_binding\":true,\"conditional_choices_only\":true,"
    <<"\"stationary_prediction_end_s\":"<<std::setprecision(17)<<until_s
    <<",\"event_count\":"<<tails.events.size()<<",\"candidate_counts\":[";
   for(std::size_t e=0;e<candidates.sets.size();++e){if(e)std::cout<<',';std::cout<<candidates.sets[e].event_set.candidates.size();}
   std::cout<<"],\"choices\":[";
   for(std::size_t e=0;e<choices.size();++e){if(e)std::cout<<',';std::cout<<choices[e];}
   std::cout<<"],\"failure\":\""<<JointPlannerFailureName(binding)<<"\"}\n";
   if(binding!=JointPlannerFailure::kNone)return 2;
  }
  const auto full_coverage=articulated_audit_detail::CheckFootCoverage(full_grid_request);
  go2_control::Go2RigidBody articulated_robot;
  if(!articulated_robot.Load(GO2_MODEL_PATH))
   throw std::runtime_error("articulated audit model load");
  const auto articulated=go2_terrain::stage_c::SolveJointTrajectoryCandidate(
      articulated_robot,shadow.last_source_state(),selected_problem,
      full_grid_request,35.0,30.0,go2_terrain::stage_c::TimeNs{5000000});
  const auto lift=articulated_audit_detail::BuildLiftAudit(
      articulated_robot,shadow.last_source_state(),selected_problem,
      proposal->foot_request,articulated.centroidal);
  articulated_audit_detail::PrintAudit(state,input.state_stamp_s,proposal,"ok",&articulated,
      &runtime_coverage,&full_coverage,&lift);
  return articulated.model_samples_verified?0:2;
 }
 if(closed_loop){
  const auto replay=go2_terrain::stage_c::joint_closed_loop_detail::RunJointClosedLoopReplay(
   shadow.last_source_state(),shadow.last_proposal(),argv[3],argv[4]);
  std::cout<<"closed_loop completed="<<replay.completed<<" model_match="<<replay.model_match
   <<" rows="<<replay.rows<<" failure="<<replay.failure<<"\n";
  return replay.completed?0:2;
 }
 return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
