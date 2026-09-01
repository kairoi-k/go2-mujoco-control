#include "diagnostic_writer.h"

#include <cstdio>
#include <cstdlib>
#include <limits>

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  const std::string path = argv[1];
  std::remove(path.c_str());
  go2_diagnostic::DiagnosticWriter writer(path);

  go2_diagnostic::CaptureRecord capture;
  capture.capture_seq = 11;
  capture.topic = go2_diagnostic::Topic::kLowState;
  capture.source_message_id = "must-be-absent";
  capture.payload_repr = go2_diagnostic::PayloadRepresentation::kAbsent;
  capture.serialized_payload = "must-be-absent";
  capture.complete_decoded_value = "must-be-absent";
  capture.source_stamp = std::numeric_limits<double>::quiet_NaN();
  capture.source_stamp_valid = false;
  capture.source_id_valid = false;
  capture.publication_identity_valid = false;
  if (!writer.WriteCapture(capture)) return 3;

  go2_diagnostic::MapRecord map;
  map.capture_id = map.capture_seq = 12;
  map.map_stamp = 4.5;
  map.map_stamp_valid = true;
  map.frame_id = "frame\"\\line\n\x01";
  map.frame_valid = true;
  map.resolution = 0.05f;
  map.origin = {1.0f, -2.0f};
  map.width = 3;
  map.height = 2;
  for (std::uint32_t iy = 0; iy < map.height; ++iy) {
    for (std::uint32_t ix = 0; ix < map.width; ++ix) {
      const bool valid = !(ix == 2 && iy == 1);
      map.cells.push_back({ix, iy, valid ? static_cast<float>(ix + iy * 3) : 0.0f,
                           valid, 10.0 + ix + iy, valid});
    }
  }
  map.complete_value = true;
  map.capture.capture_seq = map.capture_seq;
  map.capture.topic = go2_diagnostic::Topic::kLidarHeightMap;
  map.capture.payload_repr = go2_diagnostic::PayloadRepresentation::kAbsent;
  if (!writer.WriteMap(map)) return 4;

  go2_diagnostic::StateRecord state;
  state.capture_id = state.capture_seq = 13;
  state.state_stamp = 8.0;
  state.state_stamp_valid = true;
  state.position = {1.0f, 2.0f, 3.0f};
  state.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
  state.pose_valid = false;
  state.capture.capture_seq = state.capture_seq;
  state.capture.topic = go2_diagnostic::Topic::kSportModeState;
  state.capture.payload_repr = go2_diagnostic::PayloadRepresentation::kAbsent;
  if (!writer.WriteState(state)) return 5;
  return 0;
}
