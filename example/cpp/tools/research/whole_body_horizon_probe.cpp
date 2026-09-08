// Offline 20ms trajectory candidate producer; no runtime admission/actuation authority.
#include "../../terrain/stage_c/whole_body_force_tracking.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
using namespace stage_c_research;
using Gain=Eigen::Matrix<double,12,36,Eigen::RowMajor>;
struct Reference {double time;Eigen::Matrix<double,19,1> q;Eigen::Matrix<double,18,1> v;Torque u;Gain K;};
template<class T> void Read(std::istream& in,T& x){for(int i=0;i<x.size();++i)in>>x.data()[i];if(!in || !x.allFinite())throw std::runtime_error("invalid reference array");}
template<class T> void Print(const T& x){std::cout<<'[';for(int i=0;i<x.size();++i){if(i)std::cout<<',';std::cout<<x.data()[i];}std::cout<<']';}
void Array(const mjtNum* x,int n){std::cout<<'[';for(int i=0;i<n;++i){if(i)std::cout<<',';std::cout<<x[i];}std::cout<<']';}
Torque Requested(const mjModel* m,const mjData* d,const Reference& ref){Eigen::Matrix<double,36,1> e;mj_differentiatePos(m,e.data(),1,ref.q.data(),d->qpos);for(int j=0;j<18;++j)e[18+j]=d->qvel[j]-ref.v[j];return ref.u-ref.K*e;}
int main(int argc,char** argv){
 if(argc!=2)return 2;
 int lock=open("/tmp/go2_mujoco_experiment.lock",O_CREAT|O_WRONLY,0600);if(lock<0 || flock(lock,LOCK_EX|LOCK_NB))return 3;
 try {
  std::ifstream in(argv[1]);std::string magic,scene;std::getline(in,magic);std::getline(in,scene);if(magic!="whole-body-horizon-probe-input-v1")throw std::runtime_error("invalid magic");
  char error[2048];std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model(mj_loadXML(scene.c_str(),nullptr,error,sizeof(error)),mj_deleteModel);if(!model)throw std::runtime_error(error);auto m=model.get();
  if(m->nq!=19 || m->nv!=18 || m->nu!=12 || std::abs(m->opt.timestep-.002)>1e-15)throw std::runtime_error("Go2 2ms model required");
  int count=0,N=0;in>>count>>N;if(count!=mj_stateSize(m,mjSTATE_INTEGRATION) || N!=10)throw std::runtime_error("invalid 20ms coverage");
  std::vector<double> state(count);for(double& value:state)in>>value;for(double value:state)if(!std::isfinite(value))throw std::runtime_error("nonfinite integration state");
  std::vector<Reference> refs(N);for(auto& r:refs){in>>r.time;Read(in,r.q);Read(in,r.v);Read(in,r.u);Read(in,r.K);}std::string end;in>>end;if(end!="end" || in>>end)throw std::runtime_error("invalid input end");
  std::unique_ptr<mjData,decltype(&mj_deleteData)> data(mj_makeData(m),mj_deleteData);auto d=data.get();WholeBodyForceTracker tracker(m,4);
  std::cout<<std::setprecision(17)<<"{\"schema\":\"whole-body-horizon-probe-v1\",\"horizon_steps\":10,\"derivative_threads\":4,\"plans\":[";
  for(int scenario=0;scenario<3;++scenario){
   mj_setState(m,d,state.data(),mjSTATE_INTEGRATION);double vy=scenario? .05:0;d->qvel[1]+=vy;int prefix=scenario==2?3:0;
   if(scenario)std::cout<<',';std::cout<<"{\"name\":\""<<(scenario==0?"nominal":scenario==1?"actual_vy":"actual_vy_prefix6ms")<<"\",\"initial_vy_delta\":"<<vy<<",\"committed_prefix_steps\":"<<prefix<<",\"initial_integration_state\":";
   std::vector<double> initial(count);mj_getState(m,d,initial.data(),mjSTATE_INTEGRATION);Array(initial.data(),count);
   std::cout<<",\"stages\":[";bool complete=true;std::string failure;int completed=0;auto start=std::chrono::steady_clock::now();
   for(int k=0;k<N;++k){
    if(std::abs(d->time-refs[k].time)>1e-9)throw std::runtime_error("absolute reference coverage mismatch");
    Torque desired=Requested(m,d,refs[k]);ForceTrackingResult result;
    if(k<prefix){result.torque=desired.cwiseMax(-35).cwiseMin(35);tracker.SetState(d);result.forces=tracker.Evaluate(result.torque);result.feasible=result.forces.maxCoeff()<=180;result.status="committed_existing_feedback";}
    else result=tracker.Solve(d,desired);
    if(!result.feasible){complete=false;failure=result.status;break;}
    if(k)std::cout<<',';std::cout<<"{\"step\":"<<k<<",\"time\":"<<d->time<<",\"qpos\":";Array(d->qpos,19);std::cout<<",\"qvel\":";Array(d->qvel,18);std::cout<<",\"tau\":";Print(result.torque);std::cout<<",\"K\":";Print(refs[k].K);
    std::cout<<",\"desired\":";Print(desired);std::cout<<",\"forces\":";Print(result.forces);std::cout<<",\"solve_ms\":"<<result.elapsed_ms<<",\"evaluations\":"<<result.evaluations<<",\"status\":\""<<result.status<<"\"";
    for(int j=0;j<12;++j)d->ctrl[j]=result.torque[j];mj_step(m,d);std::vector<double> next(count);mj_getState(m,d,next.data(),mjSTATE_INTEGRATION);std::cout<<",\"next_integration_state\":";Array(next.data(),count);std::cout<<'}';++completed;
   }
   double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
   std::cout<<"],\"complete\":"<<(complete?"true":"false")<<",\"failure\":\""<<failure<<"\",\"completed_steps\":"<<completed<<",\"producer_wall_ms_including_output\":"<<elapsed<<",\"terminal_time\":"<<d->time<<",\"terminal_qpos\":";Array(d->qpos,19);std::cout<<",\"terminal_qvel\":";Array(d->qvel,18);std::cout<<'}';
  }
  std::cout<<"]}\n";
 }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
 close(lock);return 0;
}
