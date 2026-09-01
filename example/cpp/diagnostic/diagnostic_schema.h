#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace go2_diagnostic {

enum class PayloadRepresentation : std::uint8_t { kAbsent, kRawSerialized, kCompleteMessageValue };
enum Validity : std::uint32_t {
  kNone = 0, kSourceStamp = 1u << 0, kSourceMessageId = 1u << 1,
  kPublicationIdentity = 1u << 2, kFrame = 1u << 3, kPose = 1u << 4,
  kCompleteMap = 1u << 5, kRawRays = 1u << 6, kJoin = 1u << 7,
};
inline constexpr bool HasValidity(std::uint32_t bits, Validity bit) {
  return (bits & static_cast<std::uint32_t>(bit)) != 0;
}

enum class Topic : std::uint8_t { kLowState, kSportModeState, kLidarHeightMap, kEnvironmentHeightMap };
const char *TopicName(Topic topic) noexcept;

enum class ConsumerIdentityKind : std::uint8_t { kAbsent, kAuthoritative };
struct ConsumerIdentity {
  ConsumerIdentityKind kind = ConsumerIdentityKind::kAbsent;
  std::string value;
  bool valid() const noexcept { return kind == ConsumerIdentityKind::kAuthoritative && !value.empty(); }
};

struct CaptureRecord {
  std::uint64_t capture_seq = 0;
  Topic topic = Topic::kLowState;
  std::string source_message_id;
  PayloadRepresentation payload_repr = PayloadRepresentation::kAbsent;
  std::string serialized_payload;
  std::string complete_decoded_value;
  std::uint64_t receipt_mono_ns = 0;
  double source_stamp = 0.0;
  bool source_stamp_valid = false;
  std::string publication_handle_or_guid;
  bool source_id_valid = false;
  bool publication_identity_valid = false;
};

struct Cell {
  std::uint32_t ix = 0, iy = 0;
  float value_m = 0.0f;
  bool height_valid = false;
  double cell_stamp = 0.0;
  bool cell_valid = false;
};

struct MapRecord {
  std::uint64_t capture_id = 0, capture_seq = 0;
  double map_stamp = 0.0;
  bool map_stamp_valid = false;
  std::string frame_id;
  bool frame_valid = false;
  float resolution = 0.0f;
  std::array<float, 2> origin{};
  std::uint32_t width = 0, height = 0;
  std::vector<Cell> cells;
  bool complete_value = false;
  bool raw_ray_available = false;
  std::vector<float> raw_rays;
  CaptureRecord capture;
  bool valid() const noexcept { return complete_value && map_stamp_valid && frame_valid && cells.size() == static_cast<std::size_t>(width) * height; }
};

struct StateRecord {
  std::uint64_t capture_id = 0, capture_seq = 0;
  double state_stamp = 0.0;
  bool state_stamp_valid = false;
  std::string frame_id;
  bool frame_valid = false;
  std::array<double, 16> pose_transform{};
  bool pose_valid = false;
  std::array<float, 3> position{};
  std::array<float, 4> quaternion{};
  CaptureRecord capture;
  bool valid() const noexcept { return state_stamp_valid && frame_valid && pose_valid; }
};

struct JoinView {
  std::uint64_t map_capture_id = 0, state_capture_id = 0;
  std::string map_frame, state_frame;
  std::array<double, 16> transform{};
  bool transform_valid = false;
  std::string pair_algorithm;
  std::uint64_t pair_tolerance_ns = 0, pair_error_ns = 0;
  bool valid = false;
};

struct RoiInput {
  std::string input_id, source, frame_id, geometry;
  double stamp = 0.0;
  std::uint64_t seq = 0;
  bool valid = false;
  PayloadRepresentation raw_or_complete_value = PayloadRepresentation::kAbsent;
};
struct SweptInput : RoiInput {};

struct ConsumerAck {
  ConsumerIdentity consumer;
  std::uint64_t plan_id = 0, map_capture_id = 0, state_capture_id = 0;
  std::string frame;
  std::uint64_t valid_until = 0, ack_deadline = 0, observed_receipt = 0;
  bool valid = false;
  static ConsumerAck Absent() { return {}; }
};

}  // namespace go2_diagnostic
