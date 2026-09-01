#pragma once
#include "diagnostic_schema.h"
#include "immutable_records.h"
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
namespace go2_diagnostic {
class DdsCapture {
 public:
  explicit DdsCapture(std::uint32_t domain_id = 0);
  ~DdsCapture();
  bool Start();
  void Stop() noexcept;
  std::uint64_t capture_count() const noexcept { return next_capture_seq_.load() - 1; }
  const ImmutableRecordStore &records() const noexcept { return records_; }
  std::uint64_t lowstate_count() const noexcept { return lowstate_count_.load(); }
  std::uint64_t sport_count() const noexcept { return sport_count_.load(); }
  std::uint64_t lidar_count() const noexcept { return lidar_count_.load(); }
  std::uint64_t environment_count() const noexcept { return environment_count_.load(); }
 private:
  void OnLowState(const void *message);
  void OnSportState(const void *message);
  void OnLidarMap(const void *message);
  void OnEnvironmentMap(const void *message);
  std::uint64_t NextCapture() noexcept { return next_capture_seq_.fetch_add(1); }
  std::uint32_t domain_id_;
  std::atomic<std::uint64_t> next_capture_seq_{1};
  std::atomic<std::uint64_t> lowstate_count_{0}, sport_count_{0}, lidar_count_{0}, environment_count_{0};
  ImmutableRecordStore records_;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace go2_diagnostic
