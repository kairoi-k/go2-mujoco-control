#pragma once
#include "diagnostic_schema.h"
#include <memory>
#include <vector>
namespace go2_diagnostic {
MapRecord CopyMapRecord(const MapRecord &source);
StateRecord CopyStateRecord(const StateRecord &source);
class ImmutableRecordStore {
 public:
  std::uint64_t CommitCapture(const CaptureRecord &record);
  const std::vector<std::shared_ptr<const CaptureRecord>> &captures() const noexcept { return captures_; }
  std::uint64_t CommitMap(const MapRecord &record);
  std::uint64_t CommitState(const StateRecord &record);
  const std::vector<std::shared_ptr<const MapRecord>> &maps() const noexcept { return maps_; }
  const std::vector<std::shared_ptr<const StateRecord>> &states() const noexcept { return states_; }
 private:
  std::vector<std::shared_ptr<const CaptureRecord>> captures_;
  std::vector<std::shared_ptr<const MapRecord>> maps_;
  std::vector<std::shared_ptr<const StateRecord>> states_;
};
}  // namespace go2_diagnostic
