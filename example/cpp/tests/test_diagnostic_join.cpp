#include "join_view.h"
#include <cassert>
int main(){using namespace go2_diagnostic;MapRecord m;m.capture_id=1;m.width=m.height=1;m.complete_value=true;m.map_stamp=2;m.map_stamp_valid=true;m.frame_id="base";m.frame_valid=true;m.cells={{0,0,0,true,0,true}};StateRecord s;s.capture_id=2;s.state_stamp=2.000001;s.state_stamp_valid=true;s.frame_id="base";s.frame_valid=true;s.pose_valid=true;assert(JoinNearest(m,s,2000).valid);s.frame_id="x";assert(!JoinNearest(m,s,2000).valid);}
