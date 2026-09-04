#pragma once

// Order-109 verification-only motion clock. The normal controller clock
// remains owned by TrotExperiment::MotionClockStep; this small DDS-free seam
// supplies elapsed time after the lockstep writer handoff.

#include <cstdint>

namespace lockstep_motion
{

class StateSynchronousClock
{
public:
  static constexpr std::uint32_t kStateTickDeltaMs = 2;

  StateSynchronousClock() = default;

  // Rebase at the tick consumed by the handoff update. The first gated
  // update therefore advances time once, by exactly its state-tick delta.
  void Engage(std::uint32_t consumed_tick)
  {
    engaged_ = true;
    have_previous_tick_ = true;
    previous_tick_ = consumed_tick;
  }

  bool Engaged() const { return engaged_; }

  // Returns false for an inactive clock or a non-consecutive tick. The
  // existing WriterGate is authoritative for fail-closed reporting; this
  // seam deliberately does not invent a second protocol or recovery path.
  bool Step(std::uint32_t consumed_tick, double &motion_dt_s)
  {
    motion_dt_s = 0.0;
    if (!engaged_ || !have_previous_tick_)
      return false;
    const std::uint32_t delta_ms = consumed_tick - previous_tick_;
    if (delta_ms != kStateTickDeltaMs)
      return false;
    previous_tick_ = consumed_tick;
    motion_dt_s = static_cast<double>(delta_ms) * 0.001;
    return true;
  }

private:
  bool engaged_ = false;
  bool have_previous_tick_ = false;
  std::uint32_t previous_tick_ = 0;
};

} // namespace lockstep_motion
