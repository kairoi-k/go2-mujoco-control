#include "diagnostic_schema.h"
namespace go2_diagnostic {
const char *TopicName(Topic topic) noexcept {
  switch (topic) {
    case Topic::kLowState: return "rt/lowstate";
    case Topic::kLowCmd: return "rt/lowcmd";
    case Topic::kSportModeState: return "rt/sportmodestate";
    case Topic::kLidarHeightMap: return "rt/go2/lidar_heightmap";
    case Topic::kEnvironmentHeightMap: return "rt/go2/environment_heightmap";
  }
  return "";
}
}  // namespace go2_diagnostic
