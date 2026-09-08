// Offline fixed12ms scheduler: frozen12ms prefix + optimized20ms tail.
// Simulation advances after synchronous computation; measured lateness only gates
// adoption. This is not a realtime worker and grants no production authority.
#include "../../terrain/stage_c/whole_body_force_tracking.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
using namespace stage_c_research;
using Q=Eigen::Matrix<double,19,1>;using V=Eigen::Matrix<double,18,1>;
using Gain=Eigen::Matrix<double,12,36,Eigen::RowMajor>;
struct Ref{Q q;V v;Torque u;Gain K;};
struct Stage{int tick;Ref law;FootForces forces;double solve_ms;std::string status;};
struct Plan{int version=0,source_version=-1,observed_tick=0,start_tick=0,end_tick=6;double generation_ms=0;bool valid=false,adopted=false,late=false;std::string failure;std::vector<Stage> stages;std::vector<double> initial_state;};
struct Row{int tick,version;Torque tau;FootForces forces;std::vector<double> next_state;};
template<class T> void Read(std::istream& in,T& x){for(int i=0;i<x.size();++i)in>>x.data()[i];if(!in || !x.allFinite())throw std::runtime_error("invalid input array");}
template<class T> void Print(const T& x){std::cout<<'[';for(int i=0;i<x.size();++i){if(i)std::cout<<',';std::cout<<x.data()[i];}std::cout<<']';}
std::vector<double> State(const mjModel* m,const mjData* d){std::vector<double> x(mj_stateSize(m,mjSTATE_INTEGRATION));mj_getState(m,d,x.data(),mjSTATE_INTEGRATION);return x;}
Torque Control(const mjModel* m,const mjData* d,const Ref& ref){Eigen::Matrix<double,36,1> e;mj_differentiatePos(m,e.data(),1,ref.q.data(),d->qpos);for(int j=0;j<18;++j)e[18+j]=d->qvel[j]-ref.v[j];return ref.u-ref.K*e;}
Ref AtState(const mjData* d,const Torque& u,const Gain& K){Ref r;r.q=Eigen::Map<const Q>(d->qpos);r.v=Eigen::Map<const V>(d->qvel);r.u=u;r.K=K;return r;}
void Advance(const mjModel* m,mjData* d,const Torque& u){for(int j=0;j<12;++j)d->ctrl[j]=u[j];mj_step(m,d);}
int main(int argc,char** argv){
 if(argc!=2)return 2;int lock=open("/tmp/go2_mujoco_experiment.lock",O_CREAT|O_WRONLY,0600);if(lock<0 || flock(lock,LOCK_EX|LOCK_NB))return 3;
 try{
  std::ifstream in(argv[1]);std::string magic,scene;std::getline(in,magic);std::getline(in,scene);if(magic!="whole-body-rolling-horizon-input-v1")throw std::runtime_error("invalid magic");
  char error[2048];std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model(mj_loadXML(scene.c_str(),nullptr,error,sizeof(error)),mj_deleteModel);if(!model)throw std::runtime_error(error);auto m=model.get();
  int count=0,N=0;double vx=0,period=0;in>>count>>N>>vx>>period;if(count!=mj_stateSize(m,mjSTATE_INTEGRATION) || N!=70 || m->nq!=19 || m->nv!=18 || m->nu!=12 || std::abs(m->opt.timestep-.002)>1e-15 || std::abs(period-.14)>1e-12)throw std::runtime_error("requires Go2 2ms 70step reference");
  std::vector<double> initial(count);for(double& v:initial)in>>v;for(double v:initial)if(!std::isfinite(v))throw std::runtime_error("nonfinite initial state");
  std::vector<Ref> refs(N);for(auto& r:refs){Read(in,r.q);Read(in,r.v);Read(in,r.u);Read(in,r.K);}std::string end;in>>end;if(end!="end" || in>>end)throw std::runtime_error("invalid input end");
  std::unique_ptr<mjData,decltype(&mj_deleteData)> actual(mj_makeData(m),mj_deleteData),predicted(mj_makeData(m),mj_deleteData);auto d=actual.get();auto p=predicted.get();
  mj_setState(m,d,initial.data(),mjSTATE_INTEGRATION);d->qvel[1]+=.05;initial=State(m,d);double start_time=d->time;WholeBodyForceTracker tracker(m,4);
  auto nominal=[&](int tick){Ref r=refs[tick%N];r.q[0]+=(tick/N)*vx*period;return r;};
  Plan bootstrap;bootstrap.valid=true;bootstrap.adopted=true;bootstrap.start_tick=0;bootstrap.end_tick=6;
  for(int k=0;k<6;++k)bootstrap.stages.push_back({k,nominal(k),FootForces::Zero(),0,"bootstrap_nominal_feedback"});
  std::vector<Plan> plans;plans.reserve(240);std::vector<Row> rows;rows.reserve(1400);Plan active=bootstrap;int pending=-1;std::string failure;int failure_tick=-1;
  auto law_at=[&](const Plan& plan,int tick)->const Ref&{if(tick<plan.start_tick || tick>=plan.end_tick)throw std::runtime_error("law outside coverage");for(const Stage& s:plan.stages)if(s.tick==tick)return s.law;throw std::runtime_error("law sample missing");};
  for(int tick=0;tick<1400;++tick){
   if(tick%6==0){
    // Adoption is ordered before the next observation. No publication can occur
    // while that observation's six-step commitment is being executed.
    if(pending>=0){Plan& candidate=plans[pending];if(candidate.valid && !candidate.late && candidate.source_version==active.version){candidate.adopted=true;active=candidate;}pending=-1;}
    if(tick+6<1400){
     if(active.end_tick<tick+6){failure="missing_committed_prefix";failure_tick=tick;break;}
     Plan candidate;candidate.version=static_cast<int>(plans.size())+1;candidate.source_version=active.version;candidate.observed_tick=tick;candidate.start_tick=tick+6;candidate.end_tick=tick+16;candidate.initial_state=State(m,d);
     auto begin=std::chrono::steady_clock::now();mj_setState(m,p,candidate.initial_state.data(),mjSTATE_INTEGRATION);
     for(int j=0;j<16;++j){
      int future=tick+j;Ref ref=j<6?law_at(active,future):nominal(future);Torque requested=Control(m,p,ref);ForceTrackingResult solved;
      if(j<6){solved.torque=requested.cwiseMax(-35).cwiseMin(35);tracker.SetState(p);solved.forces=tracker.Evaluate(solved.torque);solved.feasible=solved.forces.maxCoeff()<=180;solved.status="frozen_published_prefix";}
      else solved=tracker.Solve(p,requested);
      candidate.stages.push_back({future,AtState(p,solved.torque,ref.K),solved.forces,solved.elapsed_ms,solved.status});
      if(!solved.feasible){candidate.failure=(j<6?"committed_prefix_force_violation:":"optimized_tail_failure:")+solved.status;break;}
      Advance(m,p,solved.torque);
     }
     candidate.valid=candidate.stages.size()==16 && candidate.failure.empty();candidate.generation_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();candidate.late=candidate.generation_ms>12;
     plans.push_back(std::move(candidate));pending=static_cast<int>(plans.size())-1;
     if(!plans.back().valid){failure=plans.back().failure;failure_tick=tick;break;}
    }
   }
   if(tick>=active.end_tick){failure="active_law_expired";failure_tick=tick;break;}
   Torque tau=Control(m,d,law_at(active,tick)).cwiseMax(-35).cwiseMin(35);tracker.SetState(d);FootForces f=tracker.Evaluate(tau);Advance(m,d,tau);rows.push_back({tick,active.version,tau,f,State(m,d)});
   if(f.maxCoeff()>180 || tau.cwiseAbs().maxCoeff()>35){failure="actual_force_or_torque_violation";failure_tick=tick;break;}
  }
  std::cout<<std::setprecision(17)<<"{\"schema\":\"whole-body-continuous-rolling-horizon-v1\",\"scope\":\"offline initialized flat; explicitly authorized periodic nominal reuse; synchronous simulator with measured late discard, not liveRT\",\"start_time\":"<<start_time<<",\"target_steps\":1400,\"completed_steps\":"<<rows.size()<<",\"failure_tick\":"<<failure_tick<<",\"failure\":\""<<failure<<"\",\"update_period_steps\":6,\"committed_prefix_steps\":6,\"optimized_tail_steps\":10,\"initial_integration_state\":";Print(initial);
  std::cout<<",\"candidates\":[";
  for(size_t index=0;index<plans.size();++index){const Plan& c=plans[index];if(index)std::cout<<',';std::cout<<"{\"version\":"<<c.version<<",\"source_version\":"<<c.source_version<<",\"observation_tick\":"<<c.observed_tick<<",\"start_tick\":"<<c.start_tick<<",\"end_tick\":"<<c.end_tick<<",\"generation_ms\":"<<c.generation_ms<<",\"valid\":"<<(c.valid?"true":"false")<<",\"late\":"<<(c.late?"true":"false")<<",\"adopted\":"<<(c.adopted?"true":"false")<<",\"failure\":\""<<c.failure<<"\",\"initial_integration_state\":";Print(c.initial_state);std::cout<<",\"stages\":[";
   for(size_t k=0;k<c.stages.size();++k){const Stage& s=c.stages[k];if(k)std::cout<<',';std::cout<<"{\"tick\":"<<s.tick<<",\"qpos\":";Print(s.law.q);std::cout<<",\"qvel\":";Print(s.law.v);std::cout<<",\"tau\":";Print(s.law.u);std::cout<<",\"K\":";Print(s.law.K);std::cout<<",\"forces\":";Print(s.forces);std::cout<<",\"solve_ms\":"<<s.solve_ms<<",\"status\":\""<<s.status<<"\"}";}
   std::cout<<"]}";
  }
  std::cout<<"],\"rows\":[";for(size_t k=0;k<rows.size();++k){const Row& r=rows[k];if(k)std::cout<<',';std::cout<<"{\"tick\":"<<r.tick<<",\"active_version\":"<<r.version<<",\"tau\":";Print(r.tau);std::cout<<",\"forces\":";Print(r.forces);std::cout<<",\"next_integration_state\":";Print(r.next_state);std::cout<<'}';}std::cout<<"]}\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
 close(lock);return 0;
}
