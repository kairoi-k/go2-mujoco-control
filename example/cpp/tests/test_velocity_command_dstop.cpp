#include "velocity_command.h"
#include <cassert>
int main(){go2_trot::VelocityCommandShaperParams p;p.max_accel_mps2=.8;p.max_decel_mps2=1.2;p.max_jerk_mps3=4;p.max_speed_mps=.3;go2_trot::VelocityCommandShaper s(p);s.Reset(.3);double d=0;bool stop=false;for(int i=0;i<10000;i++){auto x=s.Step(0,1e-4);d+=x.shaped_mps*1e-4;assert(x.shaped_mps>=0&&x.shaped_mps<=.3);assert(x.accel_mps2<=.8+1e-9&&x.accel_mps2>=-1.2-1e-9);if(x.shaped_mps==0){stop=true;break;}}assert(stop&&d<=.183334+1e-9);}
