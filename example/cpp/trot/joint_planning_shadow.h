#pragma once
#include "stage_c/phase_clock.h"
#include "stage_c/event_schedule.h"
#include "stage_c/model_observation.h"
#include "stage_c/terrain_candidates.h"
#include "stage_c/prepare_joint_problem.h"
#include "stage_c/centroidal_joint_proposal.h"
#include "terrain_planner.h"
#include "joint_shadow_snapshot.h"
#include <chrono>
#include <iostream>
#include <sstream>
namespace go2_trot {
// Worker-owned model and absolute clock. This probe emits reduced-model
// proposals only; it cannot publish a TerrainMotionPlan or motor command.
class JointPlanningShadow {
public:
 bool Load(const std::string &path){return robot_.Load(path);}
 // Current capture only. Rejected captures clear this result; retaining an
 // accepted command across captures belongs to the single execution owner.
 const go2_terrain::stage_c::CentroidalJointProposal &last_proposal() const { return last_proposal_; }
 const go2_control::RigidBodyState &last_source_state() const { return last_source_state_; }
 void Capture(const go2_control::RigidBodyState &state,
              const go2_terrain::TerrainPlannerInput &legacy,std::uint64_t id,
              go2_control::GaitPattern pattern,
              const go2_terrain::stage_c::WorldTerrainSnapshot *terrain_history=nullptr) {
  using namespace go2_terrain;using namespace stage_c;
  const auto begin=std::chrono::steady_clock::now();
  last_proposal_={};last_source_state_=state;
  const TimeNs now=TimeNs::FromSeconds(legacy.state_stamp_s);
  if(now.value<0 || !legacy.contact_schedule.measured_valid) {
   std::cout<<"JointShadow id="<<id<<" detail=invalid_state_or_measured_contact command_authority=0\n";return;}
  PhaseClockObservation observation;observation.observation_time=now;
  observation.phase=legacy.gait_phase;observation.period_s=legacy.gait_period_s;
  observation.duty_factor=legacy.duty_factor;
  observation.leg_offsets=CaptureGaitOffsets(pattern);
  const auto clock=clock_.Capture(observation,false);
  JointPlannerFailure failure=JointPlannerFailure::kObservationUnavailable;
  std::string detail="clock_rejected";std::size_t events=0,combinations=0;
  bool feasible=false;double residual=0,max_anchor_gap=0;
  int history_initial_queries=0;std::size_t history_candidates=0,history_selected=0;std::string terrain_query_error="none";
  int qp_iterations=0,scp_iterations=0;std::string solver_detail="not_called";
  auto report=[&](){std::ostringstream line;line.precision(17);line<<"JointShadow id="<<id<<" state="<<legacy.state_stamp_s
   <<" phase="<<legacy.gait_phase<<" period="<<legacy.gait_period_s
   <<" clock="<<static_cast<int>(clock.failure)<<" epoch="<<clock.epoch
   <<" phase_residual_ns="<<clock.phase_residual.value
   <<" anchor_source=force_conditioned_geometry_estimate command_authority=0"
   <<" events="<<events<<" combinations="<<combinations<<" feasible="<<feasible
   <<" failure="<<JointPlannerFailureName(failure)<<" detail="<<detail
   <<" history_candidates="<<history_candidates<<" history_selected="<<history_selected
   <<" history_initial_queries="<<history_initial_queries<<" terrain_query_error="<<terrain_query_error
   <<" residual="<<residual<<" anchor_gap_m="<<max_anchor_gap
   <<" qp_iterations="<<qp_iterations<<" scp_iterations="<<scp_iterations<<" solver_detail="<<solver_detail
   <<" elapsed_us="<<std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count()<<"\n";std::cout<<line.str();};
  auto report_query=[&](const char *stage,int leg,std::size_t event,std::size_t candidate,const WorldTerrainQueryDiagnostic &q){
   std::ostringstream line;line.precision(17);
   line<<"JointTerrainQuery id="<<id<<" stage="<<stage<<" leg="<<leg<<" event="<<event<<" candidate="<<candidate
    <<" reason="<<WorldTerrainQueryFailureName(q.failure)<<" world_x="<<q.world_x<<" world_y="<<q.world_y
    <<" local_x="<<q.local_x<<" local_y="<<q.local_y<<" radius="<<q.radius_m
    <<" known="<<q.patch_known<<" total="<<q.patch_total<<" outside="<<q.patch_outside
    <<" age_max="<<q.cell_age_max_s<<" age_limit="<<q.max_cell_age_s<<"\n";std::cout<<line.str();};
  if(!clock.accepted){report();return;}
  const auto *terrain=legacy.terrain;
  if(!terrain || !terrain->valid() || !terrain->registered){detail="map_unavailable";report();return;}
  if(terrain_history && (!terrain_history->ok() || terrain_history->metadata.aggregate_epoch!=terrain->epoch ||
      std::abs(terrain_history->metadata.state_time_s-legacy.state_stamp_s)>1e-9)) {
   failure=JointPlannerFailure::kInvalidInput;detail="history_state_epoch_conflict";report();return;
  }
  if(!snapshot_dumped_ && legacy.gait_period_s<=.140000001) {
   std::cout<<"JointSnapshot "<<JointShadowSnapshotJson(state,legacy,id,static_cast<int>(pattern),terrain_history)<<"\n";
   snapshot_dumped_=true;
  }
  go2_control::RigidBodyPlanningKinematics model;
  if(!robot_.EvaluatePlanningKinematics(state,model)){detail="model_unavailable";report();return;}
  const TimeNs end{now.value+280000000};
  const auto phase=clock_.snapshot();
  FixedSchedulePreviewRequest timing{now,end,phase.origin_time,TimeNs::FromSeconds(phase.period_s),TimeNs{20000000},phase.epoch,phase.duty_factor,phase.leg_offsets};
  const auto preview=BuildFixedSchedulePreview(timing);events=preview.events.events.size();
  if(!preview.complete){failure=preview.failure;detail="preview_incomplete";report();return;}
  ContactEvidence measured;measured.valid=legacy.contact_schedule.measured_valid;measured.provenance=ContactProvenance::kMeasured;
  measured.source_time=now;measured.mask=legacy.contact_schedule.measured_contact;
  std::array<TimedPoint,4> anchors;std::array<ContactSurface,4> initial_surfaces;
  TerrainCandidateConfig candidate_config;candidate_config.allow_registered_heading_frame=true;
  candidate_config.allow_contact_continuation_beyond_horizon=true;
  for(int l=0;l<4;++l)if(measured.mask[l]) {
   const auto c=model.dynamics.foot_pos_world[l];
   const double radius=model.dynamics.foot_geometry[l].collision_radius_m;
   TerrainPatch patch;WorldTerrainQueryDiagnostic query;
   bool patch_ok=false;
   if(terrain_history){
    const auto observed=SampleWorldTerrainSnapshot(*terrain_history,c.x(),c.y(),radius,candidate_config.maximum_cell_age_s);
    patch_ok=observed.ok();patch=observed.patch;query=observed.diagnostic;
    terrain_query_error=WorldTerrainSnapshotErrorName(observed.error);
    if(patch_ok){history_initial_queries+=observed.history_used;
     std::ostringstream line;line.precision(17);line<<"JointTerrainHistory id="<<id<<" stage=initial leg="<<l
      <<" selected_sequence="<<observed.selected_source_sequence<<" source_time="<<observed.selected_source_time_s
      <<" history_used="<<observed.history_used<<" stationary="<<observed.stationary_terrain_assumption<<"\n";std::cout<<line.str();}
   }else patch_ok=SampleWorldTerrainPatch(*terrain,c.x(),c.y(),radius,candidate_config.maximum_cell_age_s,patch,&query);
   if(!patch_ok || !patch.valid || !patch.all_known) {
    report_query("initial",l,0,0,query);
    failure=terrain_query_error=="conflicting_history"?JointPlannerFailure::kInitialConditionConflict:JointPlannerFailure::kCoverageIncomplete;
    detail=terrain_query_error=="conflicting_history"?"initial_history_conflict":"initial_patch_unknown";report();return;
   }
   if(!terrain_candidate_detail::SingleSupportPatch(patch,radius,candidate_config)) {
    failure=JointPlannerFailure::kInitialConditionConflict;
    detail="initial_surface_not_single_patch";report();return;
   }
   auto &s=initial_surfaces[l];
   if(!terrain_candidate_detail::MakeBasis(patch.normal,s.basis_world)){detail="initial_normal_invalid";report();return;}
   s.frame=Frame::kWorld;s.coverage=MapCoverageState::kKnown;s.map_epoch=terrain->epoch;
   s.valid_until=end;s.friction_mu=.8;s.max_normal_n=180;
   // Model-derived surface point conditioned on measured force and a known
   // terrain normal. Keep its map-plane gap explicit; do not call it a
   // ground-truth contact location or silently project it onto the map.
   const Eigen::Vector3d p=c-radius*s.basis_world.col(2);
   anchors[l]={{p.x(),p.y(),p.z()},Frame::kWorld,now,true,PointRole::kSurfaceContactPoint};
   max_anchor_gap=std::max(max_anchor_gap,std::abs(p.z()-patch.center_height_m));
  }
  MapObservation map;map.metadata_valid=true;map.frame_id=terrain->frame_id;
  map.source=terrain->source;map.epoch=terrain->epoch;map.acquisition_time=TimeNs::FromSeconds(terrain->map_stamp_s);
  map.resolution_m=terrain->resolution_m;map.origin_m=terrain->origin_m;map.width=terrain->width;map.height=terrain->height;
  map.total_cells=terrain->cells.size();for(const auto &c:terrain->cells)map.known_cells+=c.known;
  Phase1CommandAuthority command;command.valid=true;command.command_epoch=phase.epoch;
  command.period_s=phase.period_s;command.duty_factor=phase.duty_factor;
  command.shaped_vx_mps=command.applied_vx_mps=legacy.commanded_vx_mps;
  PlanningBudget budget;budget.prediction_start=now;budget.prediction_end=end;
  budget.max_candidate_combinations=2;budget.max_solver_iterations=1200;
  PlanningIdentity identity{static_cast<std::uint64_t>(std::llround(legacy.state_stamp_s*1000)),now,terrain->epoch,phase.epoch,id};
  const auto input=CaptureModelPlanningObservation(robot_,state,identity,measured,anchors,map,command,budget,model,SupportAnchorProvenance::kForceConditionedGeometryEstimate);
  if(!input.ok){failure=input.failure;detail="observation_rejected";report();return;}
  TerrainCandidateReference reference;reference.com_world_valid=true;
  reference.com_world={model.dynamics.com_world.x(),model.dynamics.com_world.y(),model.dynamics.com_world.z()};
  reference.com_velocity_world_valid=true;
  reference.com_velocity_world={legacy.commanded_vx_mps*std::cos(legacy.base_yaw_rad),legacy.commanded_vx_mps*std::sin(legacy.base_yaw_rad),0};
  reference.nominal_offset_valid.fill(true);reference.foot_radius_valid.fill(true);
  for(int l=0;l<4;++l){
   Eigen::Vector3d offset=model.dynamics.foot_pos_world[l]-model.dynamics.com_world;
   if(legacy.touchdown_target_feet_valid){const auto &p=legacy.touchdown_target_feet_base[l];
    offset=state.position_world+state.quat_world_from_body.normalized()*Eigen::Vector3d(p.x,p.y,p.z)+
     model.dynamics.foot_pos_world[l]-model.dynamics.foot_site_world[l]-model.dynamics.com_world;}
   reference.nominal_foot_center_offset_world[l]={offset.x(),offset.y(),offset.z()};
   reference.foot_radius_m[l]=model.dynamics.foot_geometry[l].collision_radius_m;
  }
  auto candidates=GenerateTerrainCandidates(*terrain,preview.events,now,terrain->epoch,reference,end,candidate_config,terrain_history);
  history_candidates=candidates.history_used_candidates;
  for(const auto &q:candidates.rejected_query_diagnostics)
   report_query("candidate",static_cast<int>(q.leg),q.event_index,q.candidate_index,q.query);
  if(!candidates.valid){failure=candidates.failure;detail="candidate_coverage";report();return;}
  go2_control::SrbdMpcParams physical;physical.mass_kg=model.dynamics.mass_kg;physical.inertia_com_world=model.dynamics.inertia_com_world;
  std::vector<StateBox> bounds(preview.grid.size());std::vector<CentroidalState> refs;
  for(const auto t:preview.grid){CentroidalState ref=CentroidalState::Zero();
   const double dt=(t.value-now.value)*1e-9;ref.head<3>()=model.dynamics.com_world+dt*Eigen::Vector3d(reference.com_velocity_world.x,reference.com_velocity_world.y,0);
   ref.segment<3>(3)<<reference.com_velocity_world.x,reference.com_velocity_world.y,0;refs.push_back(ref);}
  auto prepared=PrepareJointProblem(input.input,preview,candidates,initial_surfaces,model,physical,bounds,refs);
  if(!prepared.ok){failure=prepared.failure;detail=prepared.detail;report();return;}
  last_proposal_=SearchCentroidalJointProposal(prepared.problem);
  const auto &result=last_proposal_.search;
  qp_iterations=static_cast<int>(last_proposal_.total_qp_iterations);
  scp_iterations=static_cast<int>(last_proposal_.total_scp_iterations);
  residual=last_proposal_.max_successful_residual;
  solver_detail=last_proposal_.last_solver_detail;
  feasible=result.feasible;failure=result.failure;combinations=result.diagnostics.combinations_considered;
  if(feasible)for(std::size_t e=0;e<result.plan.candidate_indices.size();++e) {
   const auto &match=candidates.sets[e].matched_surfaces[result.plan.candidate_indices[e]];
   history_selected+=match.history_used;
   const auto &event=preview.events.events[e];const auto &p=match.surface_world;
   std::ostringstream line;line.precision(17);line<<"JointSelectedFoot id="<<id<<" event="<<e<<" leg="<<static_cast<int>(event.id.leg)
    <<" touchdown="<<event.touchdown_time.seconds()<<" x="<<p.value.x<<" y="<<p.value.y<<" z="<<p.value.z
    <<" source_time="<<p.source_time.seconds()<<" selected_sequence="<<match.source_sequence<<" history_used="<<match.history_used<<"\n";std::cout<<line.str();
  }
  detail=feasible?"reduced_proposal_only":"joint_search_rejected";report();
 }
private:
 go2_terrain::stage_c::CentroidalJointProposal last_proposal_{};
 go2_control::RigidBodyState last_source_state_{};
 bool snapshot_dumped_=false;
 go2_control::Go2RigidBody robot_;
 go2_terrain::stage_c::PhaseClock clock_;
};
} // namespace
