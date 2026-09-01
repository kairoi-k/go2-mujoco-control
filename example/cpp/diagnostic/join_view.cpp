#include "join_view.h"
#include <cmath>
#include <limits>
namespace go2_diagnostic {
JoinView JoinNearest(const MapRecord &map, const StateRecord &state, std::uint64_t tolerance_ns) {
  JoinView out;
  out.map_capture_id = map.capture_id; out.state_capture_id = state.capture_id;
  out.map_frame = map.frame_id; out.state_frame = state.frame_id;
  out.pair_algorithm = "nearest_source_stamp_unique"; out.pair_tolerance_ns = tolerance_ns;
  if (!map.valid() || !state.valid() || map.capture_id == 0 || state.capture_id == 0 ||
      map.capture_id == state.capture_id || map.frame_id != state.frame_id) return out;
  const double delta = std::abs(map.map_stamp - state.state_stamp) * 1.0e9;
  if (!std::isfinite(delta) || delta > static_cast<double>(tolerance_ns)) return out;
  out.pair_error_ns = static_cast<std::uint64_t>(delta);
  out.transform = state.pose_transform; out.transform_valid = true; out.valid = true;
  return out;
}
}  // namespace go2_diagnostic
