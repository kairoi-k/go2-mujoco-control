#pragma once
#include "centroidal_subproblem.h"

namespace go2_terrain { namespace stage_c { namespace detail {
inline Eigen::Vector3d V(const go2::Vec3 &v) { return {v.x,v.y,v.z}; }
inline go2::Vec3 V(const Eigen::Vector3d &v) { return {v.x(),v.y(),v.z()}; }
struct Prepared {
    CentroidalState initial = CentroidalState::Zero();
    std::vector<ForceArray> feet;
    std::vector<std::array<ContactSurface,4>> surfaces;
    std::vector<std::array<bool,4>> contacts;
    std::vector<double> dt;
    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
};
inline bool Point(const TimedPoint &p) {
    return p.valid && p.frame == Frame::kWorld && V(p.value).allFinite();
}
inline bool SamePoint(const TimedPoint &a,const TimedPoint &b) {
    return Point(a) && Point(b) && (V(a.value)-V(b.value)).norm()==0.0;
}
inline JointPlannerFailure Prepare(const CentroidalProblem &p, Prepared &q,
                                   std::string &why) {
    using F=JointPlannerFailure;
    auto fail=[&](F f,const char *s) { why=s; return f; };
    const auto &r=p.request; const auto &in=r.input;
    if(!in.basic_valid() || !in.body.model_com_valid ||
       !Point(in.body.model_com_world) || !Point(in.body.base_position_world) ||
       !V(in.body.com_velocity_world).allFinite() ||
       !p.initial_momentum_valid || !p.initial_momentum_world.allFinite())
        return fail(F::kObservationUnavailable,"missing_finite_world_COM_or_momentum");
    if(p.grid.size()<2 || p.grid.size()>49 || p.bounds.size()!=p.grid.size() ||
       p.schedule.empty() || p.schedule.size()>48 ||
       p.schedule_epoch!=in.identity.schedule_epoch || p.schedule_epoch==0 ||
       in.body.model_com_world.source_time!=in.identity.source_state_time ||
       p.grid.front()!=in.identity.source_state_time ||
       in.measured_contact.source_time!=p.grid.front() ||
       !in.command.valid || in.command.command_epoch==0 ||
       !std::isfinite(in.command.applied_vx_mps) ||
       !std::isfinite(in.command.applied_vy_mps) ||
       !std::isfinite(in.command.shaped_vx_mps) || !std::isfinite(in.command.shaped_vy_mps) ||
       !std::isfinite(in.command.period_s) || in.command.period_s<=0 ||
       !std::isfinite(in.command.duty_factor) || in.command.duty_factor<=0 || in.command.duty_factor>=1 ||
       !std::isfinite(p.model.mass_kg) || p.model.mass_kg<=0 ||
       p.model.mass_kg!=in.body.mass_kg || !std::isfinite(p.model.gravity_mps2) ||
       p.model.gravity_mps2<=0 || !std::isfinite(p.force_trust_n) ||
       p.force_trust_n<=0 || p.max_scp_iterations<0 || p.max_scp_iterations>32 ||
       p.max_qp_iterations<0 || p.max_qp_iterations>10000)
        return fail(F::kInvalidInput,"state_time_schedule_model_or_shape_conflict");
    const auto weights=go2_control::SrbdQ(p.model).diagonal().eval();
    if(!std::isfinite(p.w_momentum) || p.w_momentum<0 || !weights.allFinite() || weights.minCoeff()<0 ||
       !std::isfinite(p.model.w_force) || p.model.w_force<=0 ||
       !std::isfinite(p.model.w_force_trot_xy) || p.model.w_force_trot_xy<=0)
        return fail(F::kInvalidInput,"invalid_objective_weights");
    for(std::size_t k=1;k<p.grid.size();++k)
        if(p.grid[k]<=p.grid[k-1]) return fail(F::kInvalidInput,"non_strict_time_grid");
    if(p.required_end<=p.required_start || p.required_start<p.grid.front() ||
       p.required_end>p.grid.back() || in.budget.prediction_start!=p.grid.front() ||
       in.budget.prediction_end!=p.grid.back() ||
       p.schedule.front().start!=p.grid.front() || p.schedule.back().end!=p.grid.back())
        return fail(F::kCoverageIncomplete,"absolute_consumer_interval_not_covered");
    if(in.map.epoch!=in.identity.map_epoch || !in.map.metadata_valid)
        return fail(F::kObservationUnavailable,"map_identity_missing");
    if(in.initial_support_margin_valid && !std::isfinite(in.initial_support_margin_m))
        return fail(F::kInvalidInput,"nonfinite_geometric_diagnostic");
    // The frozen geometric conflict is reported alongside dynamics, not used
    // to replace the dynamics question. Initial dynamic bounds remain hard.
    q.initial.head<3>()=V(in.body.model_com_world.value);
    q.initial.segment<3>(3)=V(in.body.com_velocity_world);
    q.initial.tail<3>()=p.initial_momentum_world;
    for(const auto &box:p.bounds)
        if(!box.lower.allFinite() || !box.upper.allFinite() ||
           (box.lower.array()>box.upper.array()).any())
            return fail(F::kInvalidInput,"invalid_state_box");
    if((q.initial.array()<p.bounds.front().lower.array()).any() ||
       (q.initial.array()>p.bounds.front().upper.array()).any())
        return fail(F::kInitialConditionConflict,"fixed_initial_state_outside_bounds");
    const auto &ev=r.events.events;
    if((!ev.empty() && !r.events.valid()) || ev.size()!=r.candidate_sets.size() ||
       ev.size()!=p.combination.size() || ev.size()!=p.event_surfaces.size())
        return fail(F::kInvalidInput,"invalid_event_combination");
    if(!r.accepted_commitments.events.empty() &&
       !r.accepted_commitments.committed_prefix_compatible(r.events))
        return fail(F::kCommitmentConflict,"event_prefix_changed");
    std::vector<Eigen::Vector3d> targets;
    for(std::size_t e=0;e<ev.size();++e) {
        const auto &set=r.candidate_sets[e];
        if(!(set.event_id==ev[e].id) || ev[e].id.schedule_epoch!=p.schedule_epoch ||
           static_cast<unsigned>(ev[e].id.leg)>=4 ||
           p.combination[e]>=set.candidates.size())
            return fail(F::kInvalidInput,"event_identity_or_candidate_index_conflict");
        const auto &c=set.candidates[p.combination[e]];
        if(!Point(c.target_world) || !std::isfinite(c.foothold_cost))
            return fail(F::kInvalidInput,"nonfinite_or_nonworld_candidate");
        if(c.coverage!=MapCoverageState::kKnown)
            return fail(F::kCoverageIncomplete,"unknown_candidate_patch");
        if(!c.geometry_hard_feasible)
            return fail(F::kNoFeasibleCandidateInSet,"candidate_geometry_rejected");
        if(ev[e].committed && !SamePoint(c.target_world,ev[e].target_world))
            return fail(F::kCommitmentConflict,"selected_committed_target_changed");
        if(ev[e].touchdown_time<p.grid.front() || ev[e].touchdown_time>=p.grid.back() ||
           ev[e].contact_interval_end<=ev[e].touchdown_time)
            return fail(F::kInvalidInput,"event_outside_prediction_or_empty_contact");
        if(ev[e].touchdown_time==p.grid.front() &&
           in.measured_contact.mask[static_cast<std::size_t>(ev[e].id.leg)]) {
            const auto &foot=in.feet[static_cast<std::size_t>(ev[e].id.leg)];
            if(!foot.measured_support_anchor_valid ||
               !SamePoint(c.target_world,foot.measured_support_anchor_world))
                return fail(F::kInitialConditionConflict,"already_measured_touchdown_target_changed");
        }
        targets.push_back(V(c.target_world.value));
    }
    auto surface_ok=[&](const ContactSurface &s,TimeNs end) {
        return s.frame==Frame::kWorld && s.coverage==MapCoverageState::kKnown &&
            s.map_epoch==in.identity.map_epoch && s.valid_until>=end &&
            s.basis_world.allFinite() &&
            (s.basis_world.transpose()*s.basis_world-Eigen::Matrix3d::Identity()).norm()<1e-10 &&
            std::abs(s.basis_world.determinant()-1.0)<1e-10 &&
            std::isfinite(s.friction_mu) && s.friction_mu>=0 &&
            std::isfinite(s.min_normal_n) && s.min_normal_n>=0 &&
            std::isfinite(s.max_normal_n) && s.max_normal_n>=s.min_normal_n;
    };
    std::array<bool,4> anchor_ended{};
    for(std::size_t j=0;j<p.schedule.size();++j) {
        const auto &s=p.schedule[j];
        if(s.start>=s.end || (j && p.schedule[j-1].end!=s.start))
            return fail(F::kCoverageIncomplete,"schedule_gap_or_overlap");
        if(!std::binary_search(p.grid.begin(),p.grid.end(),s.start) ||
           !std::binary_search(p.grid.begin(),p.grid.end(),s.end))
            return fail(F::kCoverageIncomplete,"grid_straddles_contact_event");
        for(int l=0;l<4;++l) {
            if(!s.contact[l]) { anchor_ended[l]=true; continue; }
            int e=s.event_index[l];
            if(j && p.schedule[j-1].contact[l] && p.schedule[j-1].event_index[l]!=e)
                return fail(F::kInvalidInput,"contact_target_changed_without_swing");
            if(e<0) {
                if(e!=-1 || anchor_ended[l] || !in.measured_contact.mask[l] ||
                   !in.feet[l].measured_support_anchor_valid ||
                   !Point(in.feet[l].measured_support_anchor_world) ||
                   in.feet[l].measured_support_anchor_world.source_time!=p.grid.front())
                    return fail(F::kObservationUnavailable,"initial_contact_anchor_unavailable");
            } else if(e>=static_cast<int>(ev.size()) ||
                      static_cast<int>(ev[e].id.leg)!=l ||
                      s.start<ev[e].touchdown_time || s.end>ev[e].contact_interval_end) {
                return fail(F::kInvalidInput,"contact_event_time_or_leg_conflict");
            }
        }
    }
    // Every scheduled touchdown is represented throughout its half-open stance.
    // Missing events cannot turn into fabricated aerial gaps or reused anchors.
    for(std::size_t e=0;e<ev.size();++e) {
        auto end=std::min(ev[e].contact_interval_end,p.grid.back());
        if(!std::binary_search(p.grid.begin(),p.grid.end(),ev[e].touchdown_time) ||
           !std::binary_search(p.grid.begin(),p.grid.end(),end))
            return fail(F::kCoverageIncomplete,"event_boundary_missing_from_grid");
        for(const auto &s:p.schedule) if(s.start<end && s.end>ev[e].touchdown_time) {
            auto l=static_cast<std::size_t>(ev[e].id.leg);
            if(!s.contact[l] || s.event_index[l]!=static_cast<int>(e))
                return fail(F::kInvalidInput,"event_not_preserved_by_schedule");
        }
    }
    std::size_t j=0;
    for(std::size_t k=0;k+1<p.grid.size();++k) {
        while(j+1<p.schedule.size() && p.grid[k]>=p.schedule[j].end) ++j;
        const auto &s=p.schedule[j];
        if(p.grid[k]<s.start || p.grid[k+1]>s.end)
            return fail(F::kCoverageIncomplete,"uncovered_dynamics_interval");
        ForceArray feet; for(auto &f:feet) f.setZero();
        std::array<ContactSurface,4> surfaces{};
        for(int l=0;l<4;++l) if(s.contact[l]) {
            int e=s.event_index[l];
            feet[l]=e<0?V(in.feet[l].measured_support_anchor_world.value):targets[e];
            surfaces[l]=e<0?p.initial_surfaces[l]:p.event_surfaces[e];
            if(!surface_ok(surfaces[l],p.grid[k+1]))
                return fail(F::kCoverageIncomplete,"unknown_expired_or_invalid_surface_frame");
        }
        q.feet.push_back(feet); q.surfaces.push_back(surfaces);
        q.contacts.push_back(s.contact);
        q.dt.push_back(static_cast<double>(p.grid[k+1].value-p.grid[k].value)*1e-9);
    }
    if(p.committed_states.size()!=p.committed_state_times.size() ||
       p.committed_states.size()>p.grid.size() || p.committed_forces.size()>q.dt.size())
        return fail(F::kCommitmentConflict,"invalid_continuous_prefix_size");
    for(std::size_t k=0;k<p.committed_states.size();++k)
        if(p.committed_state_times[k]!=p.grid[k] || !p.committed_states[k].allFinite() ||
           (p.committed_states[k].array()<p.bounds[k].lower.array()-kDynamicsTolerance).any() ||
           (p.committed_states[k].array()>p.bounds[k].upper.array()+kDynamicsTolerance).any() ||
           (k==0 && (p.committed_states[k]-q.initial).norm()!=0))
            return fail(F::kCommitmentConflict,"continuous_state_prefix_conflict");
    for(std::size_t k=0;k<p.committed_forces.size();++k) {
        const auto &f=p.committed_forces[k];
        if(f.start!=p.grid[k] || f.end!=p.grid[k+1] || f.contact!=q.contacts[k])
            return fail(F::kCommitmentConflict,"continuous_force_prefix_time_or_contact_changed");
        for(const auto &v:f.force_world) if(!V(v).allFinite())
            return fail(F::kCommitmentConflict,"nonfinite_committed_force");
    }
    auto model=p.model; model.dt_s=1.0;
    q.gravity=go2_control::SrbdGravity(model).tail<3>();
    return F::kNone;
}
}}}
