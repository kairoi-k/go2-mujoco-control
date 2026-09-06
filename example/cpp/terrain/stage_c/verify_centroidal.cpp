#include "centroidal_internal.h"

namespace go2_terrain { namespace stage_c {
namespace {
// Exact extrema of a cubic on [0,1]. Bounds are linear between state nodes.
// This is deliberately independent of the optimizer's Jacobian and rollout.
double MaxCubic(double a,double b,double c,double d) {
    auto value=[&](double s) {
        double v=((d*s+c)*s+b)*s+a;
        return std::isfinite(v)?v:std::numeric_limits<double>::infinity();
    };
    double high=std::max(value(0),value(1));
    double derivative_scale=std::max({std::abs(b),std::abs(c),std::abs(d)});
    if(derivative_scale==0) return high;
    b/=derivative_scale; c/=derivative_scale; d/=derivative_scale;
    // value() must keep original polynomial coefficients after normalization.
    auto test_scaled=[&](double s) { if(s>0 && s<1) {
        double v=((d*s+c)*s+b)*s*derivative_scale+a;
        high=std::max(high,std::isfinite(v)?v:std::numeric_limits<double>::infinity());
    }};
    if(std::abs(d)<1e-18) { if(std::abs(c)>1e-18) test_scaled(-b/(2*c)); }
    else {
        double disc=4*c*c-12*d*b;
        if(disc>=0) { double r=std::sqrt(disc); test_scaled((-2*c+r)/(6*d)); test_scaled((-2*c-r)/(6*d)); }
    }
    return high;
}
}
DynamicsCertificate VerifyCentroidalTrajectory(const CentroidalProblem &p,
                                               const CentroidalResult &out) {
    using namespace detail;
    DynamicsCertificate cert; Prepared q; std::string why;
    if(Prepare(p,q,why)!=JointPlannerFailure::kNone) return cert;
    cert.input_checked=true; cert.coverage_checked=true; cert.commitment_checked=true;
    cert.geometric_15mm_checked=p.request.input.initial_support_margin_valid;
    cert.geometric_15mm_pass=cert.geometric_15mm_checked && p.request.input.initial_support_margin_m>=0.015;
    bool aerial=false; int min_contacts=4;
    for(const auto &mask:q.contacts) {
        int n=std::count(mask.begin(),mask.end(),true);
        min_contacts=std::min(min_contacts,n); aerial|=n==0;
    }
    // Hypothetical transfer classification is retained; this core never sets
    // a live transfer flag or edits the frozen analyzer.
    cert.frozen_conflict=ClassifyTransferContract({true,min_contacts,aerial});
    if(out.states.size()!=p.grid.size() || out.forces.size()!=q.dt.size()) return cert;
    for(const auto &s:out.states) if(!s.allFinite()) return cert;
    for(const auto &f:out.forces) for(const auto &v:f.force_world) if(!V(v).allFinite()) return cert;
    auto &r=cert.residual;
    r.initial=(out.states.front()-q.initial).lpNorm<Eigen::Infinity>();
    for(std::size_t k=0;k<q.dt.size();++k) {
        const auto &f=out.forces[k]; const auto &x=out.states[k]; const auto &y=out.states[k+1];
        if(f.start!=p.grid[k] || f.end!=p.grid[k+1] || f.contact!=q.contacts[k]) {
            cert.coverage_checked=false; return cert;
        }
        Eigen::Vector3d total=Eigen::Vector3d::Zero();
        for(int l=0;l<4;++l) {
            Eigen::Vector3d force=V(f.force_world[l]); total+=force;
            if(!q.contacts[k][l]) { r.force_n=std::max(r.force_n,force.lpNorm<Eigen::Infinity>()); continue; }
            const auto &s=q.surfaces[k][l]; Eigen::Vector3d local=s.basis_world.transpose()*force;
            if(!local.allFinite()) return cert;
            r.force_n=std::max({r.force_n,s.min_normal_n-local.z(),local.z()-s.max_normal_n,
                std::abs(local.x())-s.friction_mu/std::sqrt(2.0)*local.z(),
                std::abs(local.y())-s.friction_mu/std::sqrt(2.0)*local.z()});
        }
        double dt=q.dt[k]; Eigen::Vector3d acc=q.gravity+total/p.model.mass_kg;
        if(!total.allFinite() || !acc.allFinite()) return cert;
        auto pos=[&](double t)->Eigen::Vector3d {
            return x.head<3>()+t*x.segment<3>(3)+0.5*t*t*acc;
        };
        auto torque=[&](double t)->Eigen::Vector3d {
            Eigen::Vector3d tau=Eigen::Vector3d::Zero();
            for(int l=0;l<4;++l) tau+=(q.feet[k][l]-pos(t)).cross(V(f.force_world[l]));
            return tau;
        };
        Eigen::Vector3d angular_impulse=dt/6.0*(torque(0)+4*torque(dt/2)+torque(dt));
        if(!angular_impulse.allFinite() || !pos(dt).allFinite()) return cert;
        r.position_m=std::max(r.position_m,(y.head<3>()-pos(dt)).lpNorm<Eigen::Infinity>());
        r.velocity_mps=std::max(r.velocity_mps,(y.segment<3>(3)-x.segment<3>(3)-dt*acc).lpNorm<Eigen::Infinity>());
        r.momentum_nms=std::max(r.momentum_nms,(y.tail<3>()-x.tail<3>()-angular_impulse).lpNorm<Eigen::Infinity>());
        CentroidalState b=CentroidalState::Zero(),c=b,d=b;
        b.head<3>()=dt*x.segment<3>(3); c.head<3>()=0.5*dt*dt*acc;
        b.segment<3>(3)=dt*acc;
        b.tail<3>()=dt*torque(0);
        c.tail<3>()=-0.5*dt*dt*x.segment<3>(3).cross(total);
        d.tail<3>()=-dt*dt*dt/6.0*acc.cross(total);
        if(!b.allFinite() || !c.allFinite() || !d.allFinite()) return cert;
        for(int a=0;a<9;++a) {
            r.state_bound=std::max({r.state_bound,
                MaxCubic(x[a]-p.bounds[k].upper[a],b[a]-(p.bounds[k+1].upper[a]-p.bounds[k].upper[a]),c[a],d[a]),
                MaxCubic(p.bounds[k].lower[a]-x[a],p.bounds[k+1].lower[a]-p.bounds[k].lower[a]-b[a],-c[a],-d[a]),
                y[a]-p.bounds[k+1].upper[a],p.bounds[k+1].lower[a]-y[a]});
        }
    }
    for(std::size_t k=0;k<p.committed_states.size();++k)
        if((p.committed_states[k]-out.states[k]).lpNorm<Eigen::Infinity>()>kDynamicsTolerance)
            cert.commitment_checked=false;
    for(std::size_t k=0;k<p.committed_forces.size();++k)
        for(int l=0;l<4;++l)
            if((V(p.committed_forces[k].force_world[l])-V(out.forces[k].force_world[l])).lpNorm<Eigen::Infinity>()>kForceTolerance)
                cert.commitment_checked=false;
    cert.original_dynamics_checked=true;
    cert.feasible=cert.commitment_checked && r.initial<=kDynamicsTolerance &&
        r.position_m<=kDynamicsTolerance && r.velocity_mps<=kDynamicsTolerance &&
        r.momentum_nms<=kDynamicsTolerance && r.force_n<=kForceTolerance && r.state_bound<=kStateTolerance;
    return cert;
}
}} // namespace
