#include "diagnostic_writer.h"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>

namespace go2_diagnostic {
namespace {

bool AppendLine(const std::string &path, const std::string &line) {
  std::ofstream output(path, std::ios::app);
  if (!output) return false;
  output << line << '\n';
  return static_cast<bool>(output);
}

std::string Escape(const std::string &value) {
  std::ostringstream escaped;
  escaped << '"';
  escaped << std::hex << std::setfill('0');
  for (unsigned char byte : value) {
    switch (byte) {
      case '"': escaped << "\\\""; break;
      case '\\': escaped << "\\\\"; break;
      case '\b': escaped << "\\b"; break;
      case '\f': escaped << "\\f"; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default:
        if (byte < 0x20) {
          escaped << "\\u" << std::setw(4) << static_cast<unsigned>(byte);
        } else {
          escaped << static_cast<char>(byte);
        }
    }
  }
  escaped << '"';
  return escaped.str();
}

std::string Number(double value) {
  if (!std::isfinite(value)) return "null";
  std::ostringstream number;
  number.imbue(std::locale::classic());
  number << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return number.str();
}

std::string Number(float value) { return Number(static_cast<double>(value)); }
std::string Bool(bool value) { return value ? "true" : "false"; }

std::string OptionalString(const std::string &value, bool valid) {
  return valid ? Escape(value) : "null";
}

std::string PayloadName(PayloadRepresentation representation) {
  switch (representation) {
    case PayloadRepresentation::kAbsent: return "absent";
    case PayloadRepresentation::kRawSerialized: return "raw_serialized";
    case PayloadRepresentation::kCompleteMessageValue: return "complete_message_value";
  }
  return "absent";
}

std::string FloatArray(const float *values, std::size_t count) {
  std::ostringstream result;
  result << '[';
  for (std::size_t i = 0; i < count; ++i) {
    if (i != 0) result << ',';
    result << Number(values[i]);
  }
  result << ']';
  return result.str();
}

std::string DoubleArray(const double *values, std::size_t count, bool valid) {
  std::ostringstream result;
  result << '[';
  for (std::size_t i = 0; i < count; ++i) {
    if (i != 0) result << ',';
    result << (valid ? Number(values[i]) : "null");
  }
  result << ']';
  return result.str();
}

std::string CaptureJson(const CaptureRecord &record) {
  const bool raw_present = record.payload_repr == PayloadRepresentation::kRawSerialized &&
                           !record.serialized_payload.empty();
  const bool complete_present = record.payload_repr == PayloadRepresentation::kCompleteMessageValue &&
                                !record.complete_decoded_value.empty();
  std::ostringstream json;
  json << "{\"capture_seq\":" << record.capture_seq
       << ",\"topic\":" << Escape(TopicName(record.topic))
       << ",\"source_message_id\":"
       << OptionalString(record.source_message_id, record.source_id_valid)
       << ",\"payload_repr\":" << Escape(PayloadName(record.payload_repr))
       << ",\"serialized_payload\":"
       << (raw_present ? Escape(record.serialized_payload) : "null")
       << ",\"complete_decoded_value\":"
       << (complete_present ? Escape(record.complete_decoded_value) : "null")
       << ",\"receipt_mono_ns\":" << record.receipt_mono_ns
       << ",\"source_stamp\":"
       << (record.source_stamp_valid ? Number(record.source_stamp) : "null")
       << ",\"source_stamp_valid\":" << Bool(record.source_stamp_valid)
       << ",\"publication_handle_or_guid\":"
       << OptionalString(record.publication_handle_or_guid, record.publication_identity_valid)
       << ",\"source_id_valid\":" << Bool(record.source_id_valid)
       << ",\"publication_identity_valid\":" << Bool(record.publication_identity_valid)
       << '}';
  return json.str();
}

std::string MapValidity(const MapRecord &record) {
  std::ostringstream json;
  json << "{\"map_stamp\":" << Bool(record.map_stamp_valid)
       << ",\"frame\":" << Bool(record.frame_valid)
       << ",\"complete_map\":" << Bool(record.complete_value)
       << ",\"raw_rays\":" << Bool(record.raw_ray_available) << '}';
  return json.str();
}

std::string StateValidity(const StateRecord &record) {
  std::ostringstream json;
  json << "{\"state_stamp\":" << Bool(record.state_stamp_valid)
       << ",\"frame\":" << Bool(record.frame_valid)
       << ",\"pose\":" << Bool(record.pose_valid)
       << ",\"join\":false}";
  return json.str();
}

std::string CellJson(const Cell &cell) {
  std::ostringstream json;
  json << "{\"ix\":" << cell.ix << ",\"iy\":" << cell.iy
       << ",\"value_m\":" << (cell.height_valid ? Number(cell.value_m) : "null")
       << ",\"height_valid\":" << Bool(cell.height_valid)
       << ",\"cell_stamp\":" << (cell.cell_valid ? Number(cell.cell_stamp) : "null")
       << ",\"cell_valid\":" << Bool(cell.cell_valid) << '}';
  return json.str();
}

}  // namespace

bool DiagnosticWriter::WriteCapture(const CaptureRecord &record) const {
  return AppendLine(path_, std::string("{\"record_type\":\"capture\",\"capture\":") +
                               CaptureJson(record) + '}');
}

bool DiagnosticWriter::WriteMap(const MapRecord &record) const {
  std::ostringstream json;
  json << "{\"record_type\":\"map\",\"capture_id\":" << record.capture_id
       << ",\"capture_seq\":" << record.capture_seq
       << ",\"map_stamp\":" << (record.map_stamp_valid ? Number(record.map_stamp) : "null")
       << ",\"map_stamp_valid\":" << Bool(record.map_stamp_valid)
       << ",\"frame_id\":" << OptionalString(record.frame_id, record.frame_valid)
       << ",\"frame_valid\":" << Bool(record.frame_valid)
       << ",\"resolution\":" << Number(record.resolution)
       << ",\"origin\":" << FloatArray(record.origin.data(), record.origin.size())
       << ",\"width\":" << record.width << ",\"height\":" << record.height
       << ",\"cells\":[";
  for (std::size_t i = 0; i < record.cells.size(); ++i) {
    if (i != 0) json << ',';
    json << CellJson(record.cells[i]);
  }
  json << "],\"complete_value\":" << Bool(record.complete_value)
       << ",\"raw_ray_available\":" << Bool(record.raw_ray_available)
       << ",\"raw_rays\":";
  if (!record.raw_ray_available) {
    json << "null";
  } else {
    json << FloatArray(record.raw_rays.data(), record.raw_rays.size());
  }
  json << ",\"valid\":" << Bool(record.valid())
       << ",\"validity\":" << MapValidity(record)
       << ",\"capture\":" << CaptureJson(record.capture) << '}';
  return AppendLine(path_, json.str());
}

bool DiagnosticWriter::WriteState(const StateRecord &record) const {
  std::ostringstream json;
  json << "{\"record_type\":\"state\",\"capture_id\":" << record.capture_id
       << ",\"capture_seq\":" << record.capture_seq
       << ",\"state_stamp\":" << (record.state_stamp_valid ? Number(record.state_stamp) : "null")
       << ",\"state_stamp_valid\":" << Bool(record.state_stamp_valid)
       << ",\"frame_id\":" << OptionalString(record.frame_id, record.frame_valid)
       << ",\"frame_valid\":" << Bool(record.frame_valid)
       << ",\"position\":" << FloatArray(record.position.data(), record.position.size())
       << ",\"quaternion\":" << FloatArray(record.quaternion.data(), record.quaternion.size())
       << ",\"pose_transform\":"
       << DoubleArray(record.pose_transform.data(), record.pose_transform.size(), record.pose_valid)
       << ",\"pose_valid\":" << Bool(record.pose_valid)
       << ",\"valid\":" << Bool(record.valid())
       << ",\"validity\":" << StateValidity(record)
       << ",\"capture\":" << CaptureJson(record.capture) << '}';
  return AppendLine(path_, json.str());
}

}  // namespace go2_diagnostic
