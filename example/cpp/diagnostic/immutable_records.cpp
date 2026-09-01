#include "immutable_records.h"
namespace go2_diagnostic {
MapRecord CopyMapRecord(const MapRecord &source) { return source; }
StateRecord CopyStateRecord(const StateRecord &source) { return source; }
std::uint64_t ImmutableRecordStore::CommitCapture(const CaptureRecord &record) { captures_.push_back(std::make_shared<const CaptureRecord>(record)); return record.capture_seq; }
std::uint64_t ImmutableRecordStore::CommitMap(const MapRecord &record) {
  auto copy = std::make_shared<MapRecord>(CopyMapRecord(record));
  maps_.push_back(std::shared_ptr<const MapRecord>(std::move(copy)));
  return maps_.back()->capture_id;
}
std::uint64_t ImmutableRecordStore::CommitState(const StateRecord &record) {
  auto copy = std::make_shared<StateRecord>(CopyStateRecord(record));
  states_.push_back(std::shared_ptr<const StateRecord>(std::move(copy)));
  return states_.back()->capture_id;
}
}  // namespace go2_diagnostic
