#include "dds_capture.h"
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <unitree/idl/go2/HeightMap_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
namespace go2_diagnostic {
static std::uint64_t ReceiptNow(){return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());}
struct DdsCapture::Impl {
  std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>> low;
  std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>> lowcmd;
  std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> sport;
  std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::HeightMap_>> lidar;
  std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::HeightMap_>> environment;
};
static bool Finite(double value) { return std::isfinite(value); }
static double Stamp(const unitree_go::msg::dds_::TimeSpec_ &stamp) {
  return static_cast<double>(stamp.sec()) + static_cast<double>(stamp.nanosec()) * 1e-9;
}
DdsCapture::DdsCapture(std::uint32_t domain_id) : domain_id_(domain_id), impl_(new Impl) {}
DdsCapture::~DdsCapture() { Stop(); }
bool DdsCapture::Start() {
  unitree::robot::ChannelFactory::Instance()->Init(domain_id_);
  impl_->low.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TopicName(Topic::kLowState)));
  impl_->lowcmd.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TopicName(Topic::kLowCmd)));
  impl_->sport.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TopicName(Topic::kSportModeState)));
  impl_->lidar.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::HeightMap_>(TopicName(Topic::kLidarHeightMap)));
  impl_->environment.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::HeightMap_>(TopicName(Topic::kEnvironmentHeightMap)));
  impl_->low->InitChannel([this](const void *m) { OnLowState(m); }, 32);
  impl_->lowcmd->InitChannel([this](const void *m) { OnLowCmd(m); }, 32);
  impl_->sport->InitChannel([this](const void *m) { OnSportState(m); }, 32);
  impl_->lidar->InitChannel([this](const void *m) { OnLidarMap(m); }, 8);
  impl_->environment->InitChannel([this](const void *m) { OnEnvironmentMap(m); }, 8);
  return true;
}
void DdsCapture::Stop() noexcept {
  if (!impl_) return;
  impl_->low.reset(); impl_->lowcmd.reset(); impl_->sport.reset(); impl_->lidar.reset(); impl_->environment.reset();
  unitree::robot::ChannelFactory::Instance()->Release();
}
void DdsCapture::OnLowState(const void *message) {
  if (!message) return;
  const auto seq = NextCapture();
  // ChannelSubscriber exposes only the typed callback value.  LowState is not
  // retained because this observer has no complete-copy schema for it.
  const CaptureRecord capture{seq, Topic::kLowState, {}, PayloadRepresentation::kAbsent,
                              {}, {}, ReceiptNow(), 0.0, false, {}, false, false};
  records_.CommitCapture(capture);
  ++lowstate_count_;
}
void DdsCapture::OnLowCmd(const void *) noexcept {
  // The typed callback deliberately does not inspect or retain LowCmd.
  ++lowcmd_count_;
}
void DdsCapture::OnSportState(const void *message) {
  if (!message) return;
  const auto &value = *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(message);
  const auto seq = NextCapture();
  StateRecord state;
  state.capture_id = seq; state.capture_seq = seq; state.state_stamp = Stamp(value.stamp()); state.state_stamp_valid = Finite(state.state_stamp);
  // SportModeState has no frame_id field. Do not infer one from topic or convention.
  state.frame_valid = false; state.position = value.position(); state.quaternion = value.imu_state().quaternion();
  // pose_transform is not a field in SportModeState; never synthesize one.
  state.pose_transform = {};
  state.pose_valid = false;
  // The message stamp is retained as state data, not DDS source provenance.
  state.capture = {seq, Topic::kSportModeState, {}, PayloadRepresentation::kAbsent,
                   {}, {}, ReceiptNow(), 0.0, false, {}, false, false};
  records_.CommitState(state); ++sport_count_;
}
void DdsCapture::OnLidarMap(const void *message) {
  if (!message) return;
  const auto &value = *static_cast<const unitree_go::msg::dds_::HeightMap_ *>(message);
  MapRecord map; map.capture_id = NextCapture(); map.capture_seq = map.capture_id;
  map.map_stamp = value.stamp(); map.map_stamp_valid = Finite(map.map_stamp); map.frame_id = value.frame_id(); map.frame_valid = !map.frame_id.empty();
  map.resolution = value.resolution(); map.origin = value.origin(); map.width = value.width(); map.height = value.height();
  const std::uint64_t expected = static_cast<std::uint64_t>(map.width) * map.height;
  map.complete_value = expected == value.data().size() && expected <= static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max());
  if (map.complete_value) { map.cells.reserve(value.data().size()); for (std::uint32_t i = 0; i < map.height; ++i) for (std::uint32_t j = 0; j < map.width; ++j) { const float cell = value.data()[static_cast<std::size_t>(i) * map.width + j]; map.cells.push_back({j, i, cell, std::isfinite(cell), 0.0, std::isfinite(cell)}); } }
  // Typed map fields are retained above; DDS serialization and SampleInfo are absent.
  map.capture = {map.capture_id, Topic::kLidarHeightMap, {}, PayloadRepresentation::kAbsent,
                 {}, {}, ReceiptNow(), 0.0, false, {}, false, false};
  records_.CommitMap(map); ++lidar_count_;
}
void DdsCapture::OnEnvironmentMap(const void *message) {
  if (!message) return;
  const auto &value = *static_cast<const unitree_go::msg::dds_::HeightMap_ *>(message);
  MapRecord map; map.capture_id = NextCapture(); map.capture_seq = map.capture_id; map.map_stamp = value.stamp(); map.map_stamp_valid = Finite(map.map_stamp); map.frame_id = value.frame_id(); map.frame_valid = !map.frame_id.empty(); map.resolution = value.resolution(); map.origin = value.origin(); map.width = value.width(); map.height = value.height();
  const std::uint64_t expected = static_cast<std::uint64_t>(map.width) * map.height; map.complete_value = expected == value.data().size();
  if (map.complete_value) { map.cells.reserve(value.data().size()); for (std::uint32_t i=0;i<map.height;++i) for (std::uint32_t j=0;j<map.width;++j) { float v=value.data()[static_cast<std::size_t>(i)*map.width+j]; map.cells.push_back({j,i,v,std::isfinite(v),0.0,std::isfinite(v)}); } }
  // Typed map fields are retained above; DDS serialization and SampleInfo are absent.
  map.capture = {map.capture_id, Topic::kEnvironmentHeightMap, {}, PayloadRepresentation::kAbsent,
                 {}, {}, ReceiptNow(), 0.0, false, {}, false, false};
  records_.CommitMap(map); ++environment_count_;
}
}  // namespace go2_diagnostic
