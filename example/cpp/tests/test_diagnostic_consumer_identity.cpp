#include "diagnostic_schema.h"
#include <cassert>
int main(){using namespace go2_diagnostic;assert(!ConsumerAck::Absent().valid);ConsumerIdentity x{ConsumerIdentityKind::kAbsent,"writer-guid"};assert(!x.valid());}
