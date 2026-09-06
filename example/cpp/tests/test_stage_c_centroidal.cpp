#include "stage_c/centroidal_subproblem.h"
#include "terrain_planner.h"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace go2_terrain::stage_c;
namespace {
int checks=0;
void Check(bool ok,const char *name) {
    ++checks; if(!ok) throw std::runtime_error(name);
}
TimeNs T(double s) { return TimeNs::FromSeconds(s); }
TimedPoint Point(double x,double y,double z) { return {{x,y,z},Frame::kWorld,T(1),true}; }
ContactSurface Surface() {
    ContactSurface s; s.frame=Frame::kWorld; s.coverage=MapCoverageState::kKnown;
    s.map_epoch=7; s.valid_until=T(2); s.friction_mu=0.8; s.max_normal_n=180;
    return s;
}
CentroidalProblem Fixture(int n=1) {
    CentroidalProblem p; auto &i=p.request.input;
    i.identity={11,T(1),7,3,0};
    i.body.valid=i.body.model_com_valid=true;
    i.body.base_position_world=Point(0,0,0.42); i.body.model_com_world=Point(0,0,0.4);
    i.body.mass_kg=p.model.mass_kg=10;
    i.measured_contact.mask={{true,false,false,true}};
    i.measured_contact.valid=true; i.measured_contact.provenance=ContactProvenance::kMeasured;
    i.measured_contact.source_time=T(1);
    i.map.metadata_valid=true; i.map.epoch=7; i.map.width=i.map.height=8;
    i.map.total_cells=i.map.known_cells=64; i.map.coverage=MapCoverageState::kKnown;
    i.command.valid=true; i.command.command_epoch=4; i.command.period_s=.24; i.command.duty_factor=.4;
    i.feet[0].measured_support_anchor_world=Point(.2,-.1,0);
    i.feet[3].measured_support_anchor_world=Point(-.2,.1,0);
    i.feet[0].measured_support_anchor_valid=i.feet[3].measured_support_anchor_valid=true;
    i.initial_support_margin_valid=true; i.initial_support_margin_m=.04;
    p.initial_momentum_valid=true; p.schedule_epoch=3;
    for(auto &s:p.initial_surfaces) s=Surface();
    for(int k=0;k<=n;++k) {
        p.grid.push_back(T(1+.02*k)); StateBox box;
        box.lower<<-1,-1,.1,-20,-20,-20,-10,-10,-10;
        box.upper<<1,1,1,20,20,20,10,10,10;
        p.bounds.push_back(box);
    }
    p.required_start=p.grid.front(); p.required_end=p.grid.back();
    i.budget.prediction_start=p.grid.front(); i.budget.prediction_end=p.grid.back();
    p.schedule.push_back({p.grid.front(),p.grid.back(),{{true,false,false,true}},{{-1,-1,-1,-1}}});
    return p;
}
void PinRest(CentroidalProblem &p) {
    for(std::size_t k=1;k<p.bounds.size();++k) {
        p.bounds[k].lower.tail<6>().setZero(); p.bounds[k].upper.tail<6>().setZero();
    }
}
void Event(CentroidalProblem &p,int leg,double td,double end,double x,double y) {
    TouchdownEvent e; e.id={3,static_cast<go2::Leg>(leg),static_cast<std::uint32_t>(p.request.events.events.size()+1)};
    e.touchdown_time=T(td); e.contact_interval_end=T(end); e.target_world=Point(x,y,0);
    StageCCandidate c; c.candidate_id=1; c.target_world=e.target_world;
    c.geometry_hard_feasible=true; c.coverage=MapCoverageState::kKnown;
    p.request.events.events.push_back(e); p.request.candidate_sets.push_back({e.id,{c},true});
    p.combination.push_back(0); p.event_surfaces.push_back(Surface());
}
// Independent analytic oracle: in a symmetric flat vertical two-contact
// rest fixture the unique symmetric minimum-norm solution is mg/2 per leg.
void RestOracle(const CentroidalProblem &p,const CentroidalResult &r) {
    Check(r.certificate.feasible,"rest certificate");
    for(const auto &f:r.forces) {
        Check(std::abs(f.force_world[0].z-p.model.mass_kg*9.81/2)<2e-5,"mg/2 FR oracle");
        Check(std::abs(f.force_world[3].z-p.model.mass_kg*9.81/2)<2e-5,"mg/2 RL oracle");
    }
}
// Independent witness recomputation by exhaustive cone vertices. This does
// not call optimizer support-function algebra and checks separation strictly.
void SeparationOracle(const CentroidalProblem &p,const CentroidalResult &r) {
    const auto &w=r.certificate.separation;
    Check(r.failure==JointPlannerFailure::kDynamicsInfeasible && w.valid,"infeasible must have witness");
    Check(w.required_lower>w.attainable_upper+1e-6,"positive separation gap");
    if(w.interval!=0) return;
    const auto &s=p.schedule.front(); double dt=(p.grid[1].value-p.grid[0].value)*1e-9;
    Eigen::Vector3d c(0,0,.4),v=Eigen::Vector3d::Zero(),g(0,0,-9.81);
    double upper=0;
    for(int l=0;l<4;++l) if(s.contact[l]) {
        int e=s.event_index[l]; auto surface=e<0?p.initial_surfaces[l]:p.event_surfaces[e];
        auto point=e<0?p.request.input.feet[l].measured_support_anchor_world.value:
            p.request.candidate_sets[e].candidates[p.combination[e]].target_world.value;
        Eigen::Vector3d foot(point.x,point.y,point.z);
        double best=-1e100;
        for(double normal:{surface.min_normal_n,surface.max_normal_n})
            for(double a:{-1.0,1.0}) for(double b:{-1.0,1.0}) {
                Eigen::Vector3d force=surface.basis_world*Eigen::Vector3d(a*surface.friction_mu/std::sqrt(2.)*normal,b*surface.friction_mu/std::sqrt(2.)*normal,normal);
                Eigen::Vector3d impulse=dt/6*((foot-c).cross(force)+4*(foot-(c+v*dt/2+g*dt*dt/8)).cross(force)+(foot-(c+v*dt+g*dt*dt/2)).cross(force));
                best=std::max(best,w.linear_direction.dot(force)+w.angular_direction.dot(impulse/dt));
            }
        upper+=best;
    }
    Check(std::abs(upper-w.attainable_upper)<1e-8,"independent vertex separation oracle");
}
CentroidalProblem Choices() {
    auto p=Fixture(); PinRest(p);
    p.request.input.measured_contact.mask[0]=false;
    p.request.input.feet[3].measured_support_anchor_world=Point(-.2,0,0);
    p.initial_surfaces[3].friction_mu=0;
    Event(p,0,1,1.02,.2,0); p.event_surfaces[0].friction_mu=0;
    auto bad=p.request.candidate_sets[0].candidates[0]; bad.candidate_id=2; bad.target_world=Point(.2,.04,0);
    p.request.candidate_sets[0].candidates.push_back(bad);
    p.schedule[0].event_index[0]=0; return p;
}
}
int main(int argc,char **argv) {
 try {
    auto rest=Fixture(3); PinRest(rest); auto result=SolveCentroidalSubproblem(rest);
    RestOracle(rest,result);
    auto again=SolveCentroidalSubproblem(rest);
    Check(result.states==again.states,"deterministic state rollout");
    Check(result.cost==again.cost && result.qp_iterations==again.qp_iterations,"deterministic solve");
    for(std::size_t k=0;k<result.forces.size();++k)
        Check(result.forces[k].force_world[0].z==again.forces[k].force_world[0].z,"deterministic forces");

    auto friction=Fixture(); PinRest(friction);
    friction.bounds[1].lower[3]=friction.bounds[1].upper[3]=.2;
    friction.bounds[1].lower.tail<3>().setConstant(-10); friction.bounds[1].upper.tail<3>().setConstant(10);
    auto fr=SolveCentroidalSubproblem(friction); SeparationOracle(friction,fr);
    auto high_friction=friction;
    high_friction.initial_surfaces[0].friction_mu=high_friction.initial_surfaces[3].friction_mu=2;
    Check(SolveCentroidalSubproblem(high_friction).certificate.feasible,"friction alone separates the acceleration fixture");
    auto force=Fixture(); PinRest(force);
    force.initial_surfaces[0].max_normal_n=force.initial_surfaces[3].max_normal_n=20;
    SeparationOracle(force,SolveCentroidalSubproblem(force));

    auto capped=Fixture();
    capped.bounds[1].lower.segment<3>(3).setZero(); capped.bounds[1].upper.segment<3>(3).setZero();
    capped.initial_surfaces[0].max_normal_n=20;
    auto cap_result=SolveCentroidalSubproblem(capped);
    Check(cap_result.certificate.feasible && cap_result.qp_iterations>0,"active force cap QP feasible");
    Check(cap_result.forces[0].force_world[0].z<=20+kForceTolerance &&
        std::abs(cap_result.forces[0].force_world[0].z+cap_result.forces[0].force_world[3].z-98.1)<kForceTolerance,"unequal load analytic oracle");

    auto qp_unresolved=capped; qp_unresolved.max_qp_iterations=1;
    auto nr=SolveCentroidalSubproblem(qp_unresolved);
    Check(nr.failure==JointPlannerFailure::kNumericalFailure && !nr.certificate.separation.valid,"actual unconverged ADMM is not infeasibility");

    auto aerial=Fixture(3); aerial.schedule[0].contact.fill(false);
    auto ar=SolveCentroidalSubproblem(aerial); Check(ar.certificate.feasible,"aerial feasible");
    Check(ar.certificate.frozen_conflict==FrozenContractConflict::kTransferRequiresTwoContactsButAerialInterval,"T13 preserved");
    for(std::size_t k=0;k<ar.states.size();++k) {
        double t=.02*k;
        Check(std::abs(ar.states[k][2]-(.4-4.905*t*t))<1e-12,"ballistic position oracle");
        Check(std::abs(ar.states[k][5]+9.81*t)<1e-12,"ballistic velocity oracle");
        Check(ar.states[k].tail<3>().norm()==0,"aerial momentum conservation");
    }
    Check(std::abs(ar.states[1][2]-.4)>.001,"exact integration distinct from legacy Euler");

    auto td=Fixture(4); td.schedule.clear();
    Event(td,0,1.02,1.04,.2,-.1); Event(td,3,1.02,1.04,-.2,.1);
    Event(td,0,1.06,1.08,.25,-.1); Event(td,3,1.06,1.08,-.15,.1);
    td.schedule={ {T(1),T(1.02),{},{{-1,-1,-1,-1}}},
        {T(1.02),T(1.04),{{true,false,false,true}},{{0,-1,-1,1}}},
        {T(1.04),T(1.06),{},{{-1,-1,-1,-1}}},
        {T(1.06),T(1.08),{{true,false,false,true}},{{2,-1,-1,3}}} };
    auto tr=SolveCentroidalSubproblem(td); Check(tr.certificate.feasible,"multiple touchdown solve");
    Check(tr.forces[0].force_world[0].z==0 && tr.forces[1].force_world[0].z>1,"touchdown force transition");
    Check(tr.forces[2].force_world[0].z==0 && tr.forces[3].force_world[0].z>1,"second touchdown force transition");
    auto td_left=SampleCentroidalTrajectory(td,tr,TimeNs{T(1.02).value-1});
    auto td_right=SampleCentroidalTrajectory(td,tr,T(1.02));
    auto td_end=SampleCentroidalTrajectory(td,tr,T(1.08));
    Check(td_left.force_valid && td_left.force.force_world[0].z==0 && td_right.force.force_world[0].z>1,"half open touchdown force query");
    Check(td_end.state_valid && !td_end.force_valid &&
        !SampleCentroidalTrajectory(td,tr,TimeNs{T(1.08).value+1}).state_valid,"terminal state has no invented force interval");
    auto wrong_td=td; wrong_td.schedule[3].event_index[0]=0;
    Check(SolveCentroidalSubproblem(wrong_td).failure==JointPlannerFailure::kInvalidInput,"no first TD reuse");

    auto committed=rest; committed.committed_state_times={rest.grid[0],rest.grid[1]};
    committed.committed_states={result.states[0],result.states[1]}; committed.committed_forces={result.forces[0]};
    auto cr=SolveCentroidalSubproblem(committed); Check(cr.certificate.feasible,"continuous commitment");
    auto changed=cr; changed.forces[0].force_world[0].z+=.1;
    Check(!VerifyCentroidalTrajectory(committed,changed).feasible,"tampered committed force");
    committed.committed_state_times[1].value++;
    Check(SolveCentroidalSubproblem(committed).failure==JointPlannerFailure::kCommitmentConflict,"commitment time conflict");

    auto choices=Choices(); auto good=SolveCentroidalSubproblem(choices); Check(good.certificate.feasible,"good foothold dynamic");
    for(const auto &candidate:choices.request.candidate_sets[0].candidates) {
        std::array<go2::Vec3,4> feet{};
        feet[0]=candidate.target_world.value; feet[3]=choices.request.input.feet[3].measured_support_anchor_world.value;
        Check(go2_terrain::SupportMargin2D(feet,{{true,false,false,true}},
            choices.request.input.body.model_com_world.value,.015,.040)>=.015,"both choices pass real 15mm support diagnostic");
    }
    choices.combination[0]=1; auto bad=SolveCentroidalSubproblem(choices); SeparationOracle(choices,bad);
    auto oracle=ExhaustiveOracle(choices.request,[&](const std::vector<std::size_t> &selection) {
        auto p=choices; p.combination=selection; return AsJointEvaluation(p,SolveCentroidalSubproblem(p));
    });
    Check(oracle.feasible && oracle.plan.candidate_indices[0]==0 && oracle.diagnostics.combinations_considered==2,"two foothold exhaustive oracle");
    Check(!oracle.plan.certificate.geometry_checked && !oracle.plan.rollout.knots[0].body_pose_valid,"reduced certificate no fabricated pose");
    choices.combination[0]=0; choices.request.events.events[0].committed=true;
    choices.request.accepted_commitments=choices.request.events;
    choices.request.events.events[0].contact_interval_end.value++;
    Check(SolveCentroidalSubproblem(choices).failure==JointPlannerFailure::kCommitmentConflict,"stance end commitment");
    choices=Choices(); choices.request.events.events[0].committed=true;
    choices.request.accepted_commitments=choices.request.events; choices.combination[0]=1;
    Check(SolveCentroidalSubproblem(choices).failure==JointPlannerFailure::kCommitmentConflict,"candidate cannot alter committed target");

    auto endpoint=rest; endpoint.required_end.value++;
    Check(SolveCentroidalSubproblem(endpoint).failure==JointPlannerFailure::kCoverageIncomplete,"horizon one ns overrun");
    endpoint=rest; endpoint.grid[1]=endpoint.grid[0];
    Check(SolveCentroidalSubproblem(endpoint).failure==JointPlannerFailure::kInvalidInput,"duplicate grid rejected");
    endpoint=td; endpoint.grid.erase(endpoint.grid.begin()+1); endpoint.bounds.erase(endpoint.bounds.begin()+1);
    Check(SolveCentroidalSubproblem(endpoint).failure==JointPlannerFailure::kCoverageIncomplete,"contact boundary grid coverage");
    auto unknown=rest; unknown.initial_surfaces[0].coverage=MapCoverageState::kUnknownInside;
    Check(SolveCentroidalSubproblem(unknown).failure==JointPlannerFailure::kCoverageIncomplete,"unknown contact fail closed");
    unknown=rest; unknown.initial_surfaces[0].frame=Frame::kHeadingMap;
    Check(SolveCentroidalSubproblem(unknown).failure==JointPlannerFailure::kCoverageIncomplete,"surface frame fail closed");
    unknown=rest; unknown.initial_surfaces[0].valid_until=T(1.01);
    Check(SolveCentroidalSubproblem(unknown).failure==JointPlannerFailure::kCoverageIncomplete,"surface age fail closed");
    auto invalid=rest; invalid.request.input.body.model_com_world.value.x=std::numeric_limits<double>::quiet_NaN();
    Check(SolveCentroidalSubproblem(invalid).failure==JointPlannerFailure::kObservationUnavailable,"nan state fail closed");
    invalid=rest; invalid.bounds[0].upper[2]=.3;
    Check(SolveCentroidalSubproblem(invalid).failure==JointPlannerFailure::kInitialConditionConflict,"fixed dynamic initial conflict");
    invalid=Fixture(); invalid.request.input.body.model_com_world.value.y=.035;
    std::array<go2::Vec3,4> initial_feet{};
    initial_feet[0]=invalid.request.input.feet[0].measured_support_anchor_world.value;
    initial_feet[3]=invalid.request.input.feet[3].measured_support_anchor_world.value;
    invalid.request.input.initial_support_margin_m=go2_terrain::SupportMargin2D(
        initial_feet,{{true,false,false,true}},invalid.request.input.body.model_com_world.value,.015,.040);
    Check(invalid.request.input.initial_support_margin_m<.015,"actual initial geometry conflict");
    auto geo=SolveCentroidalSubproblem(invalid);
    Check(geo.certificate.feasible && !geo.certificate.geometric_15mm_pass &&
          ValidateInitialCondition(invalid.request.input)==JointPlannerFailure::kInitialConditionConflict,"geometric 15mm separate from dynamic feasibility");
    invalid=rest; invalid.max_qp_iterations=0;
    Check(SolveCentroidalSubproblem(invalid).failure==JointPlannerFailure::kNumericalFailure,"numerical not infeasible");

    // Independent checker must reject corrupted states, contact metadata,
    // swing force, force cones, and even an interior-only ballistic violation.
    auto corrupt=result; corrupt.states[1][2]+=.01;
    Check(!VerifyCentroidalTrajectory(rest,corrupt).feasible,"position corruption");
    corrupt=result; corrupt.states[1][8]+=.01;
    Check(!VerifyCentroidalTrajectory(rest,corrupt).feasible,"momentum corruption");
    corrupt=result; corrupt.forces[0].force_world[1].z=.01;
    Check(!VerifyCentroidalTrajectory(rest,corrupt).feasible,"swing zero force");
    corrupt=result; corrupt.forces[0].force_world[0].x=100;
    Check(!VerifyCentroidalTrajectory(rest,corrupt).feasible,"friction residual");
    corrupt=result; corrupt.forces[0].contact[0]=false;
    Check(!VerifyCentroidalTrajectory(rest,corrupt).feasible,"contact metadata corruption");
    corrupt=result; corrupt.states[0][0]=std::numeric_limits<double>::infinity();
    Check(!VerifyCentroidalTrajectory(rest,corrupt).feasible,"finite certificate");
    auto arc=Fixture(); arc.schedule[0].contact.fill(false); arc.request.input.body.com_velocity_world.z=.0981;
    auto arc_result=SolveCentroidalSubproblem(arc); Check(arc_result.certificate.feasible,"ballistic arc");
    arc.bounds[0].upper[2]=arc.bounds[1].upper[2]=.4001;
    Check(arc_result.states[1][2]<.4001 && !VerifyCentroidalTrajectory(arc,arc_result).feasible,"interior extrema rejection");

    auto observed=Choices(); observed.request.input.measured_contact.mask[0]=true;
    Check(SolveCentroidalSubproblem(observed).failure==JointPlannerFailure::kInitialConditionConflict,"already measured touchdown is fixed");
    Check(T(std::numeric_limits<double>::quiet_NaN()).value<0 && T(1e100).value<0,"invalid time never aliases zero");
    auto map=rest.request.input.map; map.outside_cells=map.total_cells;
    Check(ClassifyMapCoverage(map)==MapCoverageState::kMetadataUnavailable,"map counts inconsistent");
    auto malformed=Choices(); malformed.request.events.events[0].id.leg=static_cast<go2::Leg>(8);
    Check(!malformed.request.events.valid(),"event leg range");
    malformed=Choices(); malformed.request.events.events[0].target_world.value.z=std::numeric_limits<double>::quiet_NaN();
    Check(!malformed.request.events.valid(),"event finite target");

    auto slope=Fixture(); PinRest(slope);
    Eigen::Matrix3d rotation=Eigen::AngleAxisd(.1,Eigen::Vector3d::UnitY()).toRotationMatrix();
    for(auto &surface:slope.initial_surfaces) surface.basis_world=rotation;
    auto sr=SolveCentroidalSubproblem(slope); Check(sr.certificate.feasible,"sloped world normal solve");
    RestOracle(slope,sr);

    auto transformed=Fixture(); PinRest(transformed);
    Eigen::Matrix3d yaw=Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Vector3d shift(2,-3,.2);
    auto transform=[&](TimedPoint &point) {
        Eigen::Vector3d v(point.value.x,point.value.y,point.value.z); v=yaw*v+shift;
        point.value={v.x(),v.y(),v.z()};
    };
    transform(transformed.request.input.body.base_position_world);
    transform(transformed.request.input.body.model_com_world);
    for(auto &foot:transformed.request.input.feet) transform(foot.measured_support_anchor_world);
    for(auto &surface:transformed.initial_surfaces) surface.basis_world=yaw;
    for(auto &box:transformed.bounds) { box.lower.head<3>()+=shift; box.upper.head<3>()+=shift; }
    auto transformed_result=SolveCentroidalSubproblem(transformed);
    RestOracle(transformed,transformed_result);
    Check(std::abs(transformed_result.states.back()[0]-2)<1e-12 &&
          std::abs(transformed_result.states.back()[1]+3)<1e-12,"world translation and heading rotation invariance");
    auto no_swing=td; no_swing.schedule[2].contact={{true,false,false,true}};
    no_swing.schedule[2].event_index={{-1,-1,-1,-1}};
    Check(SolveCentroidalSubproblem(no_swing).failure==JointPlannerFailure::kInvalidInput,"no anchor revival without liftoff");

    // Foundation audit regression: mixed unknown/physical failures cannot
    // prove exhaustion, and the unbounded oracle must ignore request budgets.
    auto req=Choices().request; req.input.budget.max_candidate_combinations=1;
    auto mix=ExhaustiveOracle(req,[](const std::vector<std::size_t> &v) {
        JointEvaluation e; e.failure=v[0]?JointPlannerFailure::kNumericalFailure:JointPlannerFailure::kDynamicsInfeasible; return e;
    });
    Check(mix.failure==JointPlannerFailure::kNumericalFailure && mix.diagnostics.combinations_considered==2,"foundation mixed failures and unbounded oracle");
    auto coverage=ExhaustiveOracle(req,[](const std::vector<std::size_t> &) {
        JointEvaluation e; e.failure=JointPlannerFailure::kCoverageIncomplete; return e;
    });
    Check(coverage.failure==JointPlannerFailure::kCoverageIncomplete,"foundation coverage preserved");

    // A complete 0.24 s Phase-1 running-trot cycle, duty 0.4, including
    // event-aligned nonuniform subdivisions of the 20 ms base grid.
    auto trot=Fixture(12); trot.grid.push_back(T(1.096)); trot.grid.push_back(T(1.216));
    std::sort(trot.grid.begin(),trot.grid.end()); trot.bounds.assign(trot.grid.size(),trot.bounds[0]);
    Event(trot,1,1.12,1.216,.2,.1); Event(trot,2,1.12,1.216,-.2,-.1);
    trot.schedule={{T(1),T(1.096),{{true,false,false,true}},{{-1,-1,-1,-1}}},
        {T(1.096),T(1.12),{},{{-1,-1,-1,-1}}},
        {T(1.12),T(1.216),{{false,true,true,false}},{{-1,0,1,-1}}},
        {T(1.216),T(1.24),{},{{-1,-1,-1,-1}}}};
    auto trot_result=SolveCentroidalSubproblem(trot);
    Check(trot_result.certificate.feasible,"complete Phase1 running trot cycle");
    Check(trot_result.states.back()[2]!=trot_result.states.front()[2],"COM is optimized not copied");

    std::cout<<"PASS "<<checks<<" checks\n";
    if(argc>1) {
        std::ofstream file(argv[1]); file<<std::setprecision(17);
        std::vector<std::pair<std::string,CentroidalProblem>> cases={{"rest",rest},{"friction",friction},{"force",force},{"aerial",aerial},{"multi_td",td},{"choice",Choices()},{"running_trot",trot},{"active_force_cap",capped}};
        auto bad_choice=Choices(); bad_choice.combination[0]=1;
        cases.push_back({"choice_infeasible",bad_choice});
        file<<"{\"checks\":"<<checks<<",\"cases\":[";
        bool first=true;
        for(const auto &item:cases) {
            std::vector<double> times; CentroidalResult r;
            for(int n=0;n<105;++n) {
                auto start=std::chrono::steady_clock::now(); r=SolveCentroidalSubproblem(item.second);
                auto finish=std::chrono::steady_clock::now();
                if(n>=5) times.push_back(std::chrono::duration<double,std::micro>(finish-start).count());
            }
            std::sort(times.begin(),times.end());
            if(!first) file<<","; first=false;
            file<<"{\"name\":\""<<item.first<<"\",\"failure\":\""<<JointPlannerFailureName(r.failure)<<"\",\"p50_us\":"<<times[49]<<",\"p95_us\":"<<times[94]<<",\"max_us\":"<<times.back()
                <<",\"position_residual\":"<<r.certificate.residual.position_m<<",\"velocity_residual\":"<<r.certificate.residual.velocity_mps<<",\"momentum_residual\":"<<r.certificate.residual.momentum_nms<<",\"force_violation\":"<<r.certificate.residual.force_n<<",\"bound_violation\":"<<r.certificate.residual.state_bound<<",\"states\":[";
            for(std::size_t k=0;k<r.states.size();++k) {
                if(k) file<<","; file<<"["<<item.second.grid[k].seconds();
                for(int a=0;a<9;++a) file<<","<<r.states[k][a]; file<<"]";
            }
            file<<"],\"forces\":[";
            for(std::size_t k=0;k<r.forces.size();++k) {
                if(k) file<<","; file<<"[";
                for(int l=0;l<4;++l) { if(l) file<<","; const auto &f=r.forces[k].force_world[l]; file<<"["<<f.x<<","<<f.y<<","<<f.z<<"]"; }
                file<<"]";
            }
            file<<"],\"input\":{\"mass\":"<<item.second.model.mass_kg<<",\"gravity\":"<<item.second.model.gravity_mps2<<",\"grid\":[";
            for(std::size_t k=0;k<item.second.grid.size();++k) { if(k) file<<","; file<<item.second.grid[k].seconds(); }
            const auto &body=item.second.request.input.body;
            file<<"],\"initial\":["<<body.model_com_world.value.x<<","<<body.model_com_world.value.y<<","<<body.model_com_world.value.z<<","<<body.com_velocity_world.x<<","<<body.com_velocity_world.y<<","<<body.com_velocity_world.z;
            for(int a=0;a<3;++a) file<<","<<item.second.initial_momentum_world[a];
            file<<"],\"lower\":[";
            for(std::size_t k=0;k<item.second.bounds.size();++k) {
                if(k) file<<","; file<<"[";
                for(int a=0;a<9;++a) { if(a) file<<","; file<<item.second.bounds[k].lower[a]; } file<<"]";
            }
            file<<"],\"upper\":[";
            for(std::size_t k=0;k<item.second.bounds.size();++k) {
                if(k) file<<","; file<<"[";
                for(int a=0;a<9;++a) { if(a) file<<","; file<<item.second.bounds[k].upper[a]; } file<<"]";
            }
            file<<"],\"intervals\":[";
            for(std::size_t k=0;k+1<item.second.grid.size();++k) {
                if(k) file<<","; file<<"[";
                const FixedScheduleInterval *schedule=nullptr;
                for(const auto &s:item.second.schedule) if(s.start<=item.second.grid[k] && item.second.grid[k]<s.end) schedule=&s;
                for(int l=0;l<4;++l) {
                    if(l) file<<","; int e=schedule->event_index[l];
                    auto surf=e<0?item.second.initial_surfaces[l]:item.second.event_surfaces[e];
                    auto point=e<0?item.second.request.input.feet[l].measured_support_anchor_world.value:
                        item.second.request.candidate_sets[e].candidates[item.second.combination[e]].target_world.value;
                    file<<"{\"contact\":"<<(schedule->contact[l]?"true":"false")<<",\"foot\":["<<point.x<<","<<point.y<<","<<point.z<<"],\"mu\":"<<surf.friction_mu<<",\"min\":"<<surf.min_normal_n<<",\"max\":"<<surf.max_normal_n<<",\"basis\":[";
                    for(int a=0;a<3;++a) { if(a) file<<","; file<<"["; for(int j=0;j<3;++j) { if(j) file<<","; file<<surf.basis_world(a,j); } file<<"]"; }
                    file<<"]}";
                }
                file<<"]";
            }
            const auto &w=r.certificate.separation;
            file<<"]},\"separation\":{\"valid\":"<<(w.valid?"true":"false")<<",\"interval\":"<<w.interval<<",\"direction\":[";
            for(int a=0;a<3;++a) { if(a) file<<","; file<<w.linear_direction[a]; }
            for(int a=0;a<3;++a) file<<","<<w.angular_direction[a];
            file<<"],\"required\":"<<w.required_lower<<",\"available\":"<<w.attainable_upper<<"}}";
        }
        file<<"]}\n";
    }
    return 0;
 } catch(const std::exception &e) { std::cerr<<"FAIL: "<<e.what()<<" after "<<checks<<" checks\n"; return 1; }
}
