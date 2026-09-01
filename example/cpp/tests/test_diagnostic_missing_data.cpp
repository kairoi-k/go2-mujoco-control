#include "diagnostic_schema.h"
#include "offline_roi_swept.h"
#include <cassert>
int main(){using namespace go2_diagnostic;MapRecord m;m.width=m.height=2;m.complete_value=true;m.cells={{0,0,1,true,0,true}};assert(!m.valid());StateRecord s;s.pose_valid=true;s.state_stamp_valid=true;assert(!s.valid());assert(!MakeOfflineRoi("","f","g",1,1).valid);assert(!ConsumerAck::Absent().valid);}
