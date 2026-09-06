#include "centroidal_internal.h"
#include <utility>

namespace go2_terrain { namespace stage_c {
namespace {
using namespace detail;
struct Variables {
    std::vector<std::array<int,4>> index;
    int count=0;
    explicit Variables(const Prepared &q) {
        for(const auto &contact:q.contacts) {
            std::array<int,4> row{{-1,-1,-1,-1}};
            for(int l=0;l<4;++l) if(contact[l]) { row[l]=count; count+=3; }
            index.push_back(row);
        }
    }
};
ForceArray Forces(const Variables &v,const Eigen::VectorXd &u,std::size_t k) {
    ForceArray f;
    for(int l=0;l<4;++l) {
        f[l].setZero();
        if(v.index[k][l]>=0) f[l]=u.segment<3>(v.index[k][l]);
    }
    return f;
}
// Exact zero-order-hold integration. The self-cross term from sum(f)/m
// vanishes in the angular momentum integral, leaving this quadratic map.
CentroidalState Advance(const CentroidalState &x,const ForceArray &feet,
                       const ForceArray &f,double dt,double mass,
                       const Eigen::Vector3d &g) {
    Eigen::Vector3d total=Eigen::Vector3d::Zero(), moment=total;
    for(int l=0;l<4;++l) { total+=f[l]; moment+=feet[l].cross(f[l]); }
    Eigen::Vector3d acc=g+total/mass;
    CentroidalState y;
    y.head<3>()=x.head<3>()+dt*x.segment<3>(3)+0.5*dt*dt*acc;
    y.segment<3>(3)=x.segment<3>(3)+dt*acc;
    y.tail<3>()=x.tail<3>()+dt*moment-
        (dt*x.head<3>()+0.5*dt*dt*x.segment<3>(3)+dt*dt*dt*g/6.0).cross(total);
    return y;
}
Eigen::VectorXd Roll(const CentroidalProblem &p,const Prepared &q,
                     const Variables &v,const Eigen::VectorXd &u) {
    Eigen::VectorXd x(9*q.dt.size()); CentroidalState s=q.initial;
    for(std::size_t k=0;k<q.dt.size();++k) {
        s=Advance(s,q.feet[k],Forces(v,u,k),q.dt[k],p.model.mass_kg,q.gravity);
        x.segment<9>(9*k)=s;
    }
    return x;
}
void Fill(const CentroidalProblem &p,const Prepared &q,const Variables &v,
          const Eigen::VectorXd &u,CentroidalResult &out) {
    auto x=Roll(p,q,v,u); out.states={q.initial}; out.forces.clear();
    for(std::size_t k=0;k<q.dt.size();++k) {
        out.states.push_back(x.segment<9>(9*k));
        ContactForceInterval interval;
        interval.start=p.grid[k]; interval.end=p.grid[k+1]; interval.contact=q.contacts[k];
        auto f=Forces(v,u,k);
        for(int l=0;l<4;++l) interval.force_world[l]=V(f[l]);
        out.forces.push_back(interval);
    }
}
// A separating support-function certificate, not an inference from ADMM.
// At interval zero both linear and angular impulse are affine in forces even
// for the exact continuous centroidal equations. Later force-only separators
// use conservative velocity boxes and cannot prove angular infeasibility.
SeparationWitness Separate(const CentroidalProblem &p,const Prepared &q) {
    SeparationWitness out;
    for(std::size_t k=0;k<q.dt.size();++k) {
        const double dt=q.dt[k];
        CentroidalState lo=p.bounds[k].lower, hi=p.bounds[k].upper;
        if(k==0) lo=hi=q.initial;
        Eigen::Matrix<double,6,1> low,high;
        low.head<3>()=p.model.mass_kg*((p.bounds[k+1].lower.segment<3>(3)-hi.segment<3>(3))/dt-q.gravity);
        high.head<3>()=p.model.mass_kg*((p.bounds[k+1].upper.segment<3>(3)-lo.segment<3>(3))/dt-q.gravity);
        low.tail<3>()=(p.bounds[k+1].lower.tail<3>()-hi.tail<3>())/dt;
        high.tail<3>()=(p.bounds[k+1].upper.tail<3>()-lo.tail<3>())/dt;
        std::vector<Eigen::Matrix<double,6,1>> directions;
        for(int a=0;a<(k==0?6:3);++a) for(double sign:{-1.0,1.0}) {
            Eigen::Matrix<double,6,1> d=Eigen::Matrix<double,6,1>::Zero();
            d[a]=sign; directions.push_back(d);
        }
        for(int l=0;l<4;++l) if(q.contacts[k][l]) {
            const auto &s=q.surfaces[k][l];
            for(int a=0;a<2;++a) for(double sign:{-1.0,1.0}) {
                Eigen::Matrix<double,6,1> d=Eigen::Matrix<double,6,1>::Zero();
                d.head<3>()=sign*s.basis_world.col(a)-s.friction_mu/std::sqrt(2.0)*s.basis_world.col(2);
                directions.push_back(d);
            }
        }
        if(k==0) for(int l=0;l<4;++l) if(q.contacts[k][l]) {
            Eigen::Vector3d r=q.feet[k][l]-q.initial.head<3>()-
                0.5*dt*q.initial.segment<3>(3)-dt*dt*q.gravity/6.0;
            for(int axis=0;axis<3;++axis) for(double sign:{-1.0,1.0}) {
                Eigen::Matrix<double,6,1> d=Eigen::Matrix<double,6,1>::Zero();
                d[axis+3]=sign; d.head<3>()=-d.tail<3>().cross(r); directions.push_back(d);
            }
        }
        if(k==0) for(int a=0;a<4;++a) for(int b=a+1;b<4;++b)
            if(q.contacts[k][a] && q.contacts[k][b]) {
                Eigen::Vector3d chord=q.feet[k][a]-q.feet[k][b];
                if(chord.norm()<1e-12) continue;
                Eigen::Vector3d r=q.feet[k][a]-q.initial.head<3>()-
                    0.5*dt*q.initial.segment<3>(3)-dt*dt*q.gravity/6.0;
                for(double sign:{-1.0,1.0}) {
                    Eigen::Matrix<double,6,1> d=Eigen::Matrix<double,6,1>::Zero();
                    d.tail<3>()=sign*chord.normalized();
                    d.head<3>()=-d.tail<3>().cross(r); directions.push_back(d);
                }
            }
        for(const auto &d:directions) {
            double required=0,available=0;
            for(int a=0;a<6;++a) required+=d[a]*(d[a]>=0?low[a]:high[a]);
            for(int l=0;l<4;++l) if(q.contacts[k][l]) {
                Eigen::Vector3d r=q.feet[k][l]-q.initial.head<3>()-
                    0.5*dt*q.initial.segment<3>(3)-dt*dt*q.gravity/6.0;
                Eigen::Vector3d direction=d.head<3>()+d.tail<3>().cross(r);
                const auto &s=q.surfaces[k][l];
                Eigen::Vector3d local=s.basis_world.transpose()*direction;
                const double coefficient=local.z()+s.friction_mu/std::sqrt(2.0)*(std::abs(local.x())+std::abs(local.y()));
                available+=coefficient*(coefficient>=0?s.max_normal_n:s.min_normal_n);
            }
            if(required>available+1e-6 && std::isfinite(required) && std::isfinite(available)) {
                out.valid=true; out.interval=static_cast<int>(k);
                out.linear_direction=d.head<3>(); out.angular_direction=d.tail<3>();
                out.required_lower=required; out.attainable_upper=available; return out;
            }
        }
    }
    return out;
}
struct Rows {
    int n;
    std::vector<Eigen::RowVectorXd> a,e;
    std::vector<double> b,d;
    void add(Eigen::RowVectorXd row,double rhs,bool eq=false) {
        double norm=row.norm();
        if(norm>1e-12) { row/=norm; rhs/=norm; }
        (eq?e:a).push_back(std::move(row)); (eq?d:b).push_back(rhs);
    }
    Eigen::MatrixXd matrix(bool eq) const {
        const auto &rows=eq?e:a; Eigen::MatrixXd m(rows.size(),n);
        for(std::size_t i=0;i<rows.size();++i) m.row(i)=rows[i]; return m;
    }
    Eigen::VectorXd vector(bool eq) const {
        const auto &r=eq?d:b; Eigen::VectorXd x(r.size());
        for(std::size_t i=0;i<r.size();++i) x[i]=r[i]; return x;
    }
};
}

CentroidalResult SolveCentroidalSubproblem(const CentroidalProblem &p) {
    CentroidalResult out; Prepared q;
    out.failure=detail::Prepare(p,q,out.detail);
    if(out.failure!=JointPlannerFailure::kNone) return out;
    out.certificate.input_checked=true; out.certificate.coverage_checked=true;
    out.certificate.commitment_checked=true;
    out.certificate.separation=Separate(p,q);
    if(out.certificate.separation.valid) {
        out.failure=JointPlannerFailure::kDynamicsInfeasible;
        out.detail="verified_wrench_support_separation"; return out;
    }
    if(p.max_scp_iterations==0 || p.max_qp_iterations==0) {
        out.failure=JointPlannerFailure::kNumericalFailure;
        out.detail="continuous_solver_iteration_budget_zero"; return out;
    }
    Variables vars(q); const int n=vars.count, nx=9*q.dt.size();
    Eigen::VectorXd u=Eigen::VectorXd::Zero(n), diagonal=Eigen::VectorXd::Zero(n);
    for(std::size_t k=0;k<q.dt.size();++k) {
        int nc=std::count(q.contacts[k].begin(),q.contacts[k].end(),true);
        for(int l=0;l<4;++l) if(vars.index[k][l]>=0) {
            int j=vars.index[k][l];
            u.segment<3>(j)=-p.model.mass_kg*q.gravity/static_cast<double>(nc);
            diagonal.segment<3>(j)<<p.model.w_force_trot_xy,p.model.w_force_trot_xy,p.model.w_force;
            if(k<p.committed_forces.size()) u.segment<3>(j)=V(p.committed_forces[k].force_world[l]);
        }
    }
    Eigen::VectorXd reference(nx), weights(nx);
    for(std::size_t k=0;k<q.dt.size();++k) {
        CentroidalState ref=q.initial;
        double t=static_cast<double>(p.grid[k+1].value-p.grid.front().value)*1e-9;
        ref[0]+=t*p.request.input.command.applied_vx_mps;
        ref[1]+=t*p.request.input.command.applied_vy_mps;
        ref[3]=p.request.input.command.applied_vx_mps;
        ref[4]=p.request.input.command.applied_vy_mps;
        ref[5]=0; ref.tail<3>().setZero();
        reference.segment<9>(9*k)=ref;
        // Momentum regularization has units (kg*m^2/s)^-2; no posture claim.
        weights.segment<9>(9*k)<<p.model.w_pos_xy,p.model.w_pos_xy,p.model.w_pos_z,
            p.model.w_vel_xy,p.model.w_vel_xy,p.model.w_vel_z,
            p.w_momentum,p.w_momentum,p.w_momentum;
    }
    if(n==0) {
        Fill(p,q,vars,u,out); out.certificate=VerifyCentroidalTrajectory(p,out);
        out.failure=out.certificate.feasible?JointPlannerFailure::kNone:JointPlannerFailure::kNumericalFailure;
        out.detail="ballistic_zero_variable_problem";
        out.cost=0.5*((Roll(p,q,vars,u)-reference).array().square()*weights.array()).sum();
        return out;
    }
    for(int iteration=0;iteration<p.max_scp_iterations;++iteration) {
        out.scp_iterations=iteration+1;
        Eigen::VectorXd x=Roll(p,q,vars,u);
        Eigen::MatrixXd jac(nx,n);
        // Rollout is quadratic in forces, so central differences have zero
        // truncation error in exact arithmetic. Fixed perturbation/order;
        // original-equation verifier does not consume these derivatives.
        for(int j=0;j<n;++j) {
            auto plus=u,minus=u; plus[j]+=1e-3; minus[j]-=1e-3;
            jac.col(j)=(Roll(p,q,vars,plus)-Roll(p,q,vars,minus))/2e-3;
        }
        Eigen::VectorXd offset=x-jac*u;
        Eigen::MatrixXd H=jac.transpose()*weights.asDiagonal()*jac;
        H.diagonal()+=diagonal;
        Eigen::VectorXd g=jac.transpose()*weights.asDiagonal()*(offset-reference);
        Rows rows{n,{},{},{},{}};
        for(std::size_t k=0;k<q.dt.size();++k) {
            for(int l=0;l<4;++l) if(vars.index[k][l]>=0) {
                int j=vars.index[k][l]; const auto &s=q.surfaces[k][l];
                auto force_row=[&](Eigen::Vector3d a,double b) {
                    Eigen::RowVectorXd row=Eigen::RowVectorXd::Zero(n);
                    row.segment<3>(j)=a.transpose(); rows.add(row,b);
                };
                force_row(-s.basis_world.col(2),-s.min_normal_n);
                force_row(s.basis_world.col(2),s.max_normal_n);
                for(int a=0;a<2;++a) for(double sign:{-1.0,1.0})
                    force_row(sign*s.basis_world.col(a)-s.friction_mu/std::sqrt(2.0)*s.basis_world.col(2),0);
                if(k<p.committed_forces.size()) for(int a=0;a<3;++a) {
                    Eigen::RowVectorXd row=Eigen::RowVectorXd::Zero(n); row[j+a]=1;
                    rows.add(row,V(p.committed_forces[k].force_world[l])[a],true);
                }
            }
            for(int a=0;a<9;++a) {
                int j=9*k+a; double low=p.bounds[k+1].lower[a],high=p.bounds[k+1].upper[a];
                if(k+1<p.committed_states.size()) low=high=p.committed_states[k+1][a];
                if(low==high) rows.add(jac.row(j),low-offset[j],true);
                else { rows.add(jac.row(j),high-offset[j]); rows.add(-jac.row(j),offset[j]-low); }
            }
        }
        for(int j=0;j<n;++j) {
            Eigen::RowVectorXd row=Eigen::RowVectorXd::Zero(n); row[j]=1;
            rows.add(row,u[j]+p.force_trust_n); rows.add(-row,p.force_trust_n-u[j]);
        }
        Eigen::VectorXd next; int iters=0;
        go2_control::DenseQpSettings settings;
        settings.max_iterations=p.max_qp_iterations;
        settings.rho=1.0; settings.abs_tol=1e-9; settings.rel_tol=1e-9; settings.feasibility_tol=1e-8;
        bool solved=go2_control::SolveDenseQpEqNullspace(H,g,rows.matrix(false),rows.vector(false),
            rows.matrix(true),rows.vector(true),next,iters,settings);
        out.qp_iterations+=iters;
        if(!solved || next.size()!=n || !next.allFinite()) break;
        u=next;
        Fill(p,q,vars,u,out); out.certificate=VerifyCentroidalTrajectory(p,out);
        if(out.certificate.feasible) {
            out.failure=JointPlannerFailure::kNone; out.detail="original_continuous_centroidal_certificate";
            out.cost=0.5*((Roll(p,q,vars,u)-reference).array().square()*weights.array()).sum()+
                0.5*(u.array().square()*diagonal.array()).sum();
            return out;
        }
    }
    out.failure=JointPlannerFailure::kNumericalFailure;
    out.detail="no_verified_iterate_no_infeasibility_proof";
    return out;
}

CentroidalSample SampleCentroidalTrajectory(const CentroidalProblem &p,
    const CentroidalResult &r,TimeNs time) {
    CentroidalSample sample;
    if(!VerifyCentroidalTrajectory(p,r).feasible || time<p.grid.front() || time>p.grid.back()) return sample;
    sample.state_valid=true;
    if(time==p.grid.back()) { sample.state=r.states.back(); return sample; }
    auto upper=std::upper_bound(p.grid.begin(),p.grid.end(),time);
    std::size_t k=static_cast<std::size_t>(upper-p.grid.begin()-1);
    Prepared q; std::string why; detail::Prepare(p,q,why);
    ForceArray forces; for(int l=0;l<4;++l) forces[l]=V(r.forces[k].force_world[l]);
    sample.state=Advance(r.states[k],q.feet[k],forces,
        static_cast<double>(time.value-p.grid[k].value)*1e-9,p.model.mass_kg,q.gravity);
    sample.force_valid=true; sample.force=r.forces[k]; return sample;
}

JointEvaluation AsJointEvaluation(const CentroidalProblem &p,const CentroidalResult &r) {
    JointEvaluation e; auto certificate=VerifyCentroidalTrajectory(p,r);
    e.feasible=r.failure==JointPlannerFailure::kNone && certificate.feasible;
    e.failure=e.feasible?JointPlannerFailure::kNone:r.failure;
    if(!e.feasible && e.failure==JointPlannerFailure::kNone) e.failure=JointPlannerFailure::kNumericalFailure;
    if(!e.feasible) return e;
    e.cost=r.cost;
    for(std::size_t i=0;i<p.combination.size();++i)
        e.cost+=p.request.candidate_sets[i].candidates[p.combination[i]].foothold_cost;
    e.plan.cost=e.cost; e.plan.candidate_indices=p.combination;
    e.plan.certificate={true,false,true,true,true,true};
    e.plan.rollout.complete=true; e.plan.rollout.force_intervals=r.forces;
    for(std::size_t k=0;k<r.states.size();++k) {
        RolloutKnot knot; knot.time=p.grid[k]; knot.com_world=V(r.states[k].head<3>());
        knot.com_velocity_world=V(r.states[k].segment<3>(3));
        knot.angular_momentum_world=V(r.states[k].tail<3>());
        // Legacy pose placeholders deliberately invalid, never COM==base.
        knot.body_pose_valid=false;
        e.plan.rollout.knots.push_back(knot);
    }
    return e;
}
}} // namespace
