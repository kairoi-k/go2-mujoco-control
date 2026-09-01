#include "diagnostic_schema.h"
#include <cassert>
int main(){using namespace go2_diagnostic;MapRecord m;m.capture_id=7;m.width=2;m.height=1;m.complete_value=true;m.map_stamp_valid=true;m.frame_valid=true;m.frame_id="base";m.cells={{0,0,1,true,0,true},{1,0,2,true,0,true}};assert(m.valid());StateRecord s;s.capture_id=8;s.state_stamp_valid=true;s.frame_valid=true;s.pose_valid=true;assert(s.valid());assert(!ConsumerAck::Absent().valid);}
