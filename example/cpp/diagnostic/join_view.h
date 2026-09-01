#pragma once
#include "diagnostic_schema.h"
#include <memory>
namespace go2_diagnostic {
JoinView JoinNearest(const MapRecord &map, const StateRecord &state, std::uint64_t tolerance_ns);
}  // namespace go2_diagnostic
