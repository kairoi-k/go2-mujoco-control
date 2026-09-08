// Focused offline contract and numerical-failure regressions. Run with saved input fixture.
#include "../terrain/stage_c/whole_body_force_tracking.h"
#include <fstream>
#include <iostream>
#include <functional>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
using namespace stage_c_research;
void NoControl(const mjModel*,mjData*) {}
int main(int argc,char** argv) {
  if(argc!=2 && argc!=3)return 2;
  int lock=open("/tmp/go2_mujoco_experiment.lock",O_CREAT|O_WRONLY,0600);
  if(lock<0 || flock(lock,LOCK_EX|LOCK_NB)){std::cerr<<"experiment lock unavailable\n";return 3;}
  int checks=0;
  auto require=[&](bool ok,const char* name){if(!ok)throw std::runtime_error(name);++checks;};
  auto rejects=[&](std::function<void()> action,const char* name){bool caught=false;try{action();}catch(const std::exception&){caught=true;}require(caught,name);};
  try {
    std::ifstream input(argv[1]);std::string scene;std::getline(input,scene);if(argc==3)scene=argv[2];char error[2048];
    std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model(mj_loadXML(scene.c_str(),nullptr,error,sizeof(error)),mj_deleteModel);
    if(!model)throw std::runtime_error(error);auto m=model.get();
    std::unique_ptr<mjData,decltype(&mj_deleteData)> data(mj_makeData(m),mj_deleteData);auto d=data.get();
    int count=0;input>>count;require(count==mj_stateSize(m,mjSTATE_INTEGRATION),"fixture size");
    std::vector<double> state(count);for(double& x:state)input>>x;Torque desired;for(double& x:desired)input>>x;
    require(bool(input),"fixture parsing");mj_setState(m,d,state.data(),mjSTATE_INTEGRATION);
    rejects([&]{WholeBodyForceTracker tracker(nullptr);},"null model");
    rejects([&]{WholeBodyForceTracker tracker(m,0);},"thread range");
    auto trn=m->actuator_trntype[0];m->actuator_trntype[0]=mjTRN_TENDON;
    rejects([&]{WholeBodyForceTracker tracker(m);},"nonjoint transmission");m->actuator_trntype[0]=trn;
    int joint=m->actuator_trnid[0];m->actuator_trnid[0]=m->actuator_trnid[2];
    rejects([&]{WholeBodyForceTracker tracker(m);},"duplicate joint transmission");m->actuator_trnid[0]=joint;
    auto limited=m->actuator_forcelimited[0];double bound=m->actuator_forcerange[1];m->actuator_forcelimited[0]=1;m->actuator_forcerange[1]=34;
    rejects([&]{WholeBodyForceTracker tracker(m);},"actuator force clipping");m->actuator_forcelimited[0]=limited;m->actuator_forcerange[1]=bound;
    auto joint_limited=m->jnt_actfrclimited[joint];double joint_bound=m->jnt_actfrcrange[2*joint+1];m->jnt_actfrclimited[joint]=1;m->jnt_actfrcrange[2*joint+1]=34;
    rejects([&]{WholeBodyForceTracker tracker(m);},"joint force clipping");m->jnt_actfrclimited[joint]=joint_limited;m->jnt_actfrcrange[2*joint+1]=joint_bound;
    int flags=m->opt.disableflags;m->opt.disableflags|=mjDSBL_ACTUATION;
    rejects([&]{WholeBodyForceTracker tracker(m);},"disabled actuation");m->opt.disableflags=flags;
    mjcb_control=NoControl;rejects([&]{WholeBodyForceTracker tracker(m);},"control callback");mjcb_control=nullptr;
    WholeBodyForceTracker tracker(m);rejects([&]{tracker.Evaluate(desired);},"missing state");
    rejects([&]{tracker.SetState(nullptr);},"null state");tracker.SetState(d);
    mjcb_control=NoControl;rejects([&]{tracker.Evaluate(desired);},"late callback");mjcb_control=nullptr;
    d->qpos[0]=std::numeric_limits<double>::quiet_NaN();rejects([&]{tracker.SetState(d);},"nonfinite state");
    rejects([&]{tracker.Evaluate(desired);},"invalid state cannot reuse old state");mj_setState(m,d,state.data(),mjSTATE_INTEGRATION);
    rejects([&]{tracker.Solve(d,desired,12,std::numeric_limits<double>::infinity());},"infinite budget");
    auto timed=tracker.Solve(d,desired,12,1e-12);require(!timed.feasible && !timed.local_stationary && timed.status=="wall_budget_exhausted","expired budget fails closed");
    auto result=tracker.Solve(d,desired);require(result.feasible && result.forces.maxCoeff()<=180 && result.torque.cwiseAbs().maxCoeff()<=35,"fixed witness remains feasible");
    std::vector<double> after(count);mj_getState(m,d,after.data(),mjSTATE_INTEGRATION);require(after==state,"input state preserved");
    auto fast=tracker.Solve(d,result.torque);require(fast.feasible && fast.local_stationary && (fast.torque.array()==result.torque.array()).all(),"stationarity belongs to returned witness");
    d->qvel[0]=1e20;tracker.SetState(d);rejects([&]{tracker.Evaluate(desired);},"automatic numerical reset rejected");
    std::cout<<"{\"checks_passed\":"<<checks<<",\"fixed_force_max\":"<<result.forces.maxCoeff()<<",\"scope\":\"offline numerical and model contract only\"}\n";
  }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
  close(lock);return 0;
}
