#include "immutable_records.h"
#include <cassert>
int main(){using namespace go2_diagnostic;MapRecord m;m.capture_id=1;m.width=1;m.height=1;m.complete_value=true;m.cells={{0,0,4,true,0,true}};ImmutableRecordStore s;s.CommitMap(m);m.cells[0].value_m=9;assert(s.maps()[0]->cells[0].value_m==4);}
