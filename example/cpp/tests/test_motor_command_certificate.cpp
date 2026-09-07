#include "trot/motor_command_certificate.h"
#include <iostream>
#include <stdexcept>
using namespace go2_trot;
void Check(bool ok,const char *why){if(!ok)throw std::runtime_error(why);}
int main(){try{
 go2_control::Go2RigidBody robot;Check(robot.Load(GO2_MODEL_PATH),"model load");
 char error[1024]{};mjModel *model=mj_loadXML(GO2_MODEL_PATH,nullptr,error,sizeof(error));
 Check(model!=nullptr,"independent model load");mjData *data=mj_makeData(model);Check(data!=nullptr,"data allocation");
 go2_control::RigidBodyState state;state.q.setConstant(.1);state.dq.setConstant(-.2);
 std::array<MotorCommandSample,12> commands;
 for(int i=0;i<12;++i)commands[i]={.4,.3,30,2,3}; // 3+9+1 = 13 Nm
 auto good=VerifyMotorCommandComposition(robot,commands,state);
 Check(good.input_valid && good.within_model_envelope,"interior composition");
 for(double t:good.requested_torque_nm)Check(std::abs(t-13)<1e-12,"analytic PD composition");
 for(int i=0;i<12;++i)commands[i].tau=(i%2?100:-100);
 const auto clipped=VerifyMotorCommandComposition(robot,commands,state);
 Check(clipped.input_valid && !clipped.within_model_envelope && clipped.maximum_saturation_nm>0,"saturation detected");
 for(int i=0;i<12;++i)data->ctrl[i]=clipped.requested_torque_nm[i];
 mj_forward(model,data);
 double residual=0;
 for(int i=0;i<12;++i){
  const int joint=mj_name2id(model,mjOBJ_JOINT,go2_control::Go2MotorJointName(i));
  residual=std::max(residual,std::abs(data->qfrc_actuator[model->jnt_dofadr[joint]]-clipped.predicted_applied_torque_nm[i]));
 }
 Check(residual<1e-12,"MuJoCo actuator oracle");
 commands[0].kp=NAN;Check(!VerifyMotorCommandComposition(robot,commands,state).input_valid,"NaN rejected");
 commands[0].kp=-1;Check(!VerifyMotorCommandComposition(robot,commands,state).input_valid,"negative gain rejected");
 go2_control::Go2RigidBody unloaded;double low,high;Check(!unloaded.MotorTorqueEnvelope(0,low,high),"missing model rejected");
 Check(!robot.MotorTorqueEnvelope(12,low,high),"motor index rejected");
 mj_deleteData(data);mj_deleteModel(model);
 std::cout<<"Motor command composition passed MuJoCo_residual_Nm="<<residual<<"\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
