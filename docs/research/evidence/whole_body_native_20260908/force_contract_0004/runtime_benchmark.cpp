// Standalone offline benchmark; input is scene path, integration state, desired torque.
#include "../../terrain/stage_c/whole_body_force_tracking.h"
#include <fstream>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
int main(int argc,char**argv){
 if(argc<2 || argc>4)return 2;
 int lock=open("/tmp/go2_mujoco_experiment.lock",O_CREAT|O_WRONLY,0600);if(lock<0 || flock(lock,LOCK_EX|LOCK_NB)){std::cerr<<"experiment lock unavailable\n";return 3;}
 try {
  std::ifstream input(argv[1]);std::string scene;std::getline(input,scene);char error[2048];
  std::unique_ptr<mjModel,decltype(&mj_deleteModel)> m(mj_loadXML(scene.c_str(),nullptr,error,sizeof(error)),mj_deleteModel);
  if(!m)throw std::runtime_error(error);
  std::unique_ptr<mjData,decltype(&mj_deleteData)> d(mj_makeData(m.get()),mj_deleteData);
  int count=0;input>>count;
  if(!input || count!=mj_stateSize(m.get(),mjSTATE_INTEGRATION))throw std::runtime_error("state size mismatch");
  std::vector<double> state(count);for(double& value:state)input>>value;
  if(!input)throw std::runtime_error("malformed integration state");
  mj_setState(m.get(),d.get(),state.data(),mjSTATE_INTEGRATION);stage_c_research::Torque desired;for(int i=0;i<12;++i)input>>desired[i];if(!input)throw std::runtime_error("malformed input");
  int threads=argc>2?std::stoi(argv[2]):1;
  mjOption original_options=m->opt;stage_c_research::WholeBodyForceTracker tracker(m.get(),threads);
  if(argc>3 && std::string(argv[3])=="fast"){auto feasible=tracker.Solve(d.get(),desired);if(!feasible.feasible)throw std::runtime_error("no fastpath fixture");desired=feasible.torque;}
  std::cout<<std::setprecision(17)<<"{\"runs\":[";
  for(int rep=0;rep<5;++rep){auto r=tracker.Solve(d.get(),desired);if(rep)std::cout<<",";
   std::cout<<"{\"feasible\":"<<(r.feasible?"true":"false")<<",\"status\":\""<<r.status<<"\",\"elapsed_ms\":"<<r.elapsed_ms<<",\"evaluations\":"<<r.evaluations<<",\"iterations\":"<<r.sqp_iterations<<",\"control\":[";
   for(int i=0;i<12;++i){if(i)std::cout<<",";std::cout<<r.torque[i];}std::cout<<"],\"forces\":[";for(int i=0;i<8;++i){if(i)std::cout<<",";std::cout<<r.forces[i];}std::cout<<"]}";
   std::vector<double> after(count);mj_getState(m.get(),d.get(),after.data(),mjSTATE_INTEGRATION);if(std::memcmp(&original_options,&m->opt,sizeof(mjOption)))throw std::runtime_error("model options mutated");if(after!=state)throw std::runtime_error("input mutated");
  }std::cout<<"],\"input_state_unchanged\":true}\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}
 close(lock);return 0;
}
