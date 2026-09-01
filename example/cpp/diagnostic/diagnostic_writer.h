#pragma once
#include "diagnostic_schema.h"
#include <string>
namespace go2_diagnostic { class DiagnosticWriter { std::string path_; public: explicit DiagnosticWriter(std::string p):path_(std::move(p)){} bool WriteCapture(const CaptureRecord&) const; bool WriteMap(const MapRecord&) const; bool WriteState(const StateRecord&) const; }; }
