#pragma once
// Offline force-constrained nearest-torque research kernel; no runtime authority.
#include <mujoco/mujoco.h>
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
namespace stage_c_research {
using Torque = Eigen::Matrix<double,12,1>;
using FootForces = Eigen::Matrix<double,8,1>;
using ForceJacobian = Eigen::Matrix<double,8,12>;
struct ForceTrackingResult {
  bool feasible = false;
  bool local_stationary = false; // Not a global optimality certificate.
  std::string status;
  Torque torque = Torque::Zero();
  FootForces forces = FootForces::Zero();
  int evaluations = 0, derivative_evaluations = 0, sqp_iterations = 0;
  double elapsed_ms = 0, correction_norm = 0;
};
class WholeBodyForceTracker {
 public:
  explicit WholeBodyForceTracker(const mjModel* model,int derivative_threads=1) : model_(model),threads_(derivative_threads),
      pre_(mj_makeData(model),mj_deleteData),post_(mj_makeData(model),mj_deleteData),
      state_(mj_stateSize(model,mjSTATE_INTEGRATION)) {
    if(threads_<1 || threads_>12)throw std::invalid_argument("derivative threads must be 1..12");
    if (!pre_ || !post_ || model->nu!=12 || model->nplugin || model->na)
      throw std::invalid_argument("requires 12 direct motors, no activation or plugins");
    for (int i=0;i<12;++i) {
      if (model->actuator_gaintype[i]!=mjGAIN_FIXED || model->actuator_biastype[i]!=mjBIAS_NONE ||
          model->actuator_gainprm[i*mjNGAIN]!=1 || model->actuator_gear[6*i]!=1 ||
          (model->actuator_ctrllimited[i] && (model->actuator_ctrlrange[2*i]>-35 || model->actuator_ctrlrange[2*i+1]<35)))
        throw std::invalid_argument("requires unit-gain direct torque controls covering +/-35Nm");
    }
    for(int i=0;i<12;++i){gradient_pre_.emplace_back(mj_makeData(model),mj_deleteData);gradient_post_.emplace_back(mj_makeData(model),mj_deleteData);}
    const char* names[]={"FR","FL","RR","RL"};
    for(int i=0;i<4;++i) {gids_[i]=mj_name2id(model,mjOBJ_GEOM,names[i]);if(gids_[i]<0)throw std::invalid_argument("missing foot");}
  }
  void SetState(const mjData* data) {
    mj_getState(model_,data,state_.data(),mjSTATE_INTEGRATION);
    for(double value:state_)if(!std::isfinite(value))throw std::invalid_argument("nonfinite integration state");
    has_state_=true;
  }
  FootForces Evaluate(const Torque& torque) {
    if(!has_state_ || !torque.allFinite())throw std::invalid_argument("invalid state or torque");
    FootForces forces=EvaluateOn(torque,pre_.get(),post_.get());
    if(!forces.allFinite())throw std::runtime_error("nonfinite contact force");
    return forces;
  }
  ForceTrackingResult Solve(const mjData* data,const Torque& desired,int max_iterations=12,double budget_ms=30000) {
    ForceTrackingResult out;auto start=std::chrono::steady_clock::now();
    auto elapsed=[&](){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();};
    bool have_best=false;Torque best;double best_cost=std::numeric_limits<double>::infinity();
    if(!desired.allFinite() || max_iterations<1 || !(budget_ms>0))throw std::invalid_argument("invalid solve input");
    SetState(data);
    auto record=[&](const Torque& tau,const FootForces& f){
      if(elapsed()>budget_ms)throw std::runtime_error("wall_budget_exhausted");
      if(!f.allFinite())throw std::runtime_error("nonfinite contact force");
      ++out.evaluations;
      double cost=.5*(tau-desired).squaredNorm();
      if(Feasible(tau,f) && cost<best_cost){have_best=true;best=tau;best_cost=cost;}
      return f;
    };
    auto evaluate=[&](const Torque& tau){return record(tau,Evaluate(tau));};
    try {
      Torque tau=desired.cwiseMax(-35).cwiseMin(35);FootForces f=evaluate(tau);
      if(Feasible(tau,f)){out.status="box_optimum_feasible";out.local_stationary=true;}
      else for(int iter=0;iter<max_iterations;++iter) {
        out.sqp_iterations=iter+1;
        ForceJacobian jac,previous;bool stable=false;
        ++out.derivative_evaluations;
        for(double eps: {1e-3,1e-4,1e-5,1e-6}) {
          std::array<FootForces,12> plus,minus;
          // Independent MuJoCo data per column; deterministic collection order.
          #pragma omp parallel for num_threads(threads_) schedule(static)
          for(int j=0;j<12;++j){Torque shift=Torque::Zero();shift[j]=eps;plus[j]=EvaluateOn(tau+shift,gradient_pre_[j].get(),gradient_post_[j].get());minus[j]=EvaluateOn(tau-shift,gradient_pre_[j].get(),gradient_post_[j].get());}
          for(int j=0;j<12;++j){Torque shift=Torque::Zero();shift[j]=eps;record(tau+shift,plus[j]);record(tau-shift,minus[j]);jac.col(j)=(plus[j]-minus[j])/(2*eps);}
          if(eps!=1e-3) {
            stable=true;
            for(int j=0;j<12;++j)if((jac.col(j)-previous.col(j)).norm()>180e-6+.01*jac.col(j).norm())stable=false;
            if(stable)break;
          }
          previous=jac;
        }
        if(!stable){out.status="force_derivative_unresolved";break;}
        Eigen::Matrix<double,32,12> A;Eigen::Matrix<double,32,1> b;
        A.topRows<8>()=jac/180.;b.head<8>()=(FootForces::Constant(180)-f+jac*tau)/180.;
        A.middleRows<12>(8)=Eigen::Matrix<double,12,12>::Identity();A.bottomRows<12>()=-Eigen::Matrix<double,12,12>::Identity();b.tail<24>().setConstant(35);
        // Hildreth dual coordinate descent for Euclidean projection onto linear constraints.
        Torque next=desired;Eigen::Matrix<double,32,1> lambda=Eigen::Matrix<double,32,1>::Zero();bool qp_converged=false;
        for(int sweep=0;sweep<20000;++sweep) {
          double max_change=0;
          for(int row=0;row<32;++row) {
            double norm=A.row(row).squaredNorm();
            if(norm<1e-24)continue;
            double updated=std::max(0.,lambda[row]+(A.row(row).dot(next)-b[row])/norm);
            double change=updated-lambda[row];next-=change*A.row(row).transpose();lambda[row]=updated;max_change=std::max(max_change,std::abs(change)*std::sqrt(norm));
          }
          if(max_change<1e-11 && (A*next-b).maxCoeff()<1e-10){qp_converged=true;break;}
        }
        if(!qp_converged){out.status="linear_projection_unresolved";break;}
        FootForces next_force=evaluate(next);
        const double change=(next-tau).norm();tau=next;f=next_force;
        if(change<1e-8 && Feasible(tau,f)){out.status="local_sqp_stationary_feasible";out.local_stationary=true;break;}
        if(change<1e-10){out.status="nonlinear_boundary_roundoff";break;}
        out.status="sqp_iteration_limit";
      }
      if(have_best) {
        FootForces verified=evaluate(best); // Fresh exact nonlinear physical witness.
        if(Feasible(best,verified)) {out.feasible=true;out.torque=best;out.forces=verified;out.correction_norm=(best-desired).norm();if(!out.local_stationary)out.status="strictly_feasible_evaluated_witness:"+out.status;}
      }
      if(!out.feasible && out.status.empty())out.status="no_feasible_witness";
    } catch(const std::runtime_error& error){out.status=error.what();out.feasible=false;}
    out.elapsed_ms=elapsed();return out;
  }
 private:
  FootForces EvaluateOn(const Torque& torque,mjData* pre,mjData* post) {
    mj_setState(model_,pre,state_.data(),mjSTATE_INTEGRATION);mj_setState(model_,post,state_.data(),mjSTATE_INTEGRATION);
    for(int i=0;i<12;++i)pre->ctrl[i]=post->ctrl[i]=torque[i];
    mj_forward(model_,pre);mj_step(model_,post);mj_forward(model_,post);
    FootForces forces;forces.head<4>()=Sum(pre);forces.tail<4>()=Sum(post);return forces;
  }
  static bool Feasible(const Torque& tau,const FootForces& f){return tau.cwiseAbs().maxCoeff()<=35. && f.maxCoeff()<=180.;}
  Eigen::Vector4d Sum(const mjData* d) const {
    Eigen::Vector4d f=Eigen::Vector4d::Zero();
    for(int i=0;i<d->ncon;++i){mjtNum cf[6];mj_contactForce(model_,d,i,cf);for(int leg=0;leg<4;++leg)if(d->contact[i].geom1==gids_[leg] || d->contact[i].geom2==gids_[leg])f[leg]+=cf[0];}
    return f;
  }
  const mjModel* model_;int threads_;std::unique_ptr<mjData,decltype(&mj_deleteData)> pre_,post_;
  std::vector<std::unique_ptr<mjData,decltype(&mj_deleteData)>> gradient_pre_,gradient_post_;
  std::vector<mjtNum> state_;std::array<int,4> gids_{};bool has_state_=false;
};
}  // namespace stage_c_research
