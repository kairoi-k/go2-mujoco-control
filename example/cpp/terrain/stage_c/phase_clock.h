#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include "types.h"
namespace go2_terrain
{
namespace stage_c
{
// The running trot kernel advances its wrapped phase accumulator by
// dt/current_period.  This owner intentionally freezes that period and duty
// for one epoch; it does not claim to integrate arbitrary variable-speed
// history.  A timing change therefore starts a new epoch explicitly.
struct PhaseClockObservation
{
    TimeNs observation_time{};
    double phase = 0.0;
    double period_s = 0.0;
    double duty_factor = 0.0;
    std::array<double, go2::kLegCount> leg_offsets{};
};
struct PhaseClockConfig
{
    // Residual is the shortest signed phase error expressed in absolute time.
    // The tolerance is explicit so a caller can relate it to its state clock.
    TimeNs phase_time_tolerance{1'000'000};
};
enum class PhaseClockFailure : std::uint8_t
{
    kNone = 0,
    kInvalidObservation,
    kTimeRewind,
    kPhaseDrift,
    kCommitmentConflict,
    kEventUnavailable,
};
struct PhaseClockSnapshot
{
    bool valid = false;
    std::uint64_t epoch = 0;
    TimeNs origin_time{};
    TimeNs last_observation_time{};
    TimeNs last_phase_residual{};
    double phase = 0.0;
    double period_s = 0.0;
    double duty_factor = 0.0;
    std::array<double, go2::kLegCount> leg_offsets{};
};
struct PhaseClockCaptureResult
{
    bool accepted = false;
    bool epoch_changed = false;
    bool timing_changed = false;
    PhaseClockFailure failure = PhaseClockFailure::kNone;
    std::uint64_t epoch = 0;
    TimeNs phase_residual{};
    bool phase_residual_valid = false;
};
struct PhaseClockLegEvent
{
    bool valid = false;
    std::int64_t cycle_index = -1;
    TimeNs liftoff_time{};
    TimeNs touchdown_time{};
};
class PhaseClock final
{
public:
    explicit PhaseClock(PhaseClockConfig config = {})
        : config_(config)
    {
    }
    // A timing change with active commitments is rejected before mutating the
    // old epoch.  With no active commitments it is an explicit reinitialise:
    // origin and epoch move together, so old events cannot be reused silently.
    PhaseClockCaptureResult Capture(
        const PhaseClockObservation &observation,
        bool active_commitments = false)
    {
        PhaseClockCaptureResult result;
        result.epoch = state_.epoch;
        if (!ValidConfig() || !ValidObservation(observation))
        {
            result.failure = PhaseClockFailure::kInvalidObservation;
            return result;
        }
        if (!state_.valid)
        {
            TimeNs origin{};
            if (!ComputeOrigin(observation, origin))
            {
                result.failure = PhaseClockFailure::kInvalidObservation;
                return result;
            }
            state_.valid = true;
            state_.epoch = 1;
            state_.origin_time = origin;
            state_.last_observation_time = observation.observation_time;
            state_.last_phase_residual = TimeNs{};
            state_.phase = observation.phase;
            state_.period_s = observation.period_s;
            state_.duty_factor = observation.duty_factor;
            state_.leg_offsets = observation.leg_offsets;
            result.accepted = true;
            result.epoch = state_.epoch;
            result.phase_residual_valid = true;
            return result;
        }
        if (observation.observation_time < state_.last_observation_time)
        {
            result.failure = PhaseClockFailure::kTimeRewind;
            return result;
        }
        result.timing_changed = TimingChanged(observation);
        if (result.timing_changed)
        {
            if (active_commitments)
            {
                result.failure = PhaseClockFailure::kCommitmentConflict;
                return result;
            }
            if (state_.epoch == std::numeric_limits<std::uint64_t>::max())
            {
                result.failure = PhaseClockFailure::kInvalidObservation;
                return result;
            }
            TimeNs origin{};
            if (!ComputeOrigin(observation, origin))
            {
                result.failure = PhaseClockFailure::kInvalidObservation;
                return result;
            }
            state_.epoch += 1;
            state_.origin_time = origin;
            state_.last_observation_time = observation.observation_time;
            state_.last_phase_residual = TimeNs{};
            state_.phase = observation.phase;
            state_.period_s = observation.period_s;
            state_.duty_factor = observation.duty_factor;
            state_.leg_offsets = observation.leg_offsets;
            result.accepted = true;
            result.epoch_changed = true;
            result.epoch = state_.epoch;
            result.phase_residual_valid = true;
            return result;
        }
        const TimeNs residual = PhaseResidual(observation);
        result.phase_residual = residual;
        result.phase_residual_valid = true;
        if (AbsNs(residual.value) > config_.phase_time_tolerance.value)
        {
            result.failure = PhaseClockFailure::kPhaseDrift;
            return result;
        }
        state_.last_observation_time = observation.observation_time;
        state_.last_phase_residual = residual;
        state_.phase = observation.phase;
        result.accepted = true;
        result.epoch = state_.epoch;
        return result;
    }
    PhaseClockSnapshot snapshot() const
    {
        return state_;
    }
    // Returns the first touchdown at or after at_or_after.  Its liftoff may
    // precede the observation: that is the physical in-flight anchor.  The
    // returned touchdown itself is never fabricated in the past.
    PhaseClockFailure NextEvent(
        std::size_t leg, TimeNs at_or_after, PhaseClockLegEvent &out) const
    {
        out = PhaseClockLegEvent{};
        if (!state_.valid || leg >= go2::kLegCount || at_or_after.value < 0)
            return PhaseClockFailure::kEventUnavailable;
        if (at_or_after < state_.last_observation_time)
            return PhaseClockFailure::kTimeRewind;
        long double period_ns = 0.0L;
        if (!PeriodNs(state_.period_s, period_ns))
            return PhaseClockFailure::kEventUnavailable;
        const long double target =
            (static_cast<long double>(at_or_after.value) -
             static_cast<long double>(state_.origin_time.value)) /
                period_ns + static_cast<long double>(state_.leg_offsets[leg]);
        long double cycle = std::ceil(target);
        if (!std::isfinite(cycle) ||
            cycle < static_cast<long double>(std::numeric_limits<std::int64_t>::min()) + 2.0L ||
            cycle > static_cast<long double>(std::numeric_limits<std::int64_t>::max()) - 2.0L)
            return PhaseClockFailure::kEventUnavailable;
        const long double offset =
            static_cast<long double>(state_.leg_offsets[leg]);
        TimeNs touchdown{};
        TimeNs liftoff{};
        if (!Stamp(state_.origin_time, cycle - offset, period_ns, touchdown) ||
            !Stamp(state_.origin_time,
                  cycle - 1.0L + state_.duty_factor - offset,
                  period_ns, liftoff))
            return PhaseClockFailure::kEventUnavailable;
        // Integer rounding can put an exact boundary one nanosecond before the
        // query. Move only forward; never retime an event backward.
        while (touchdown < at_or_after)
        {
            cycle += 1.0L;
            if (!Stamp(state_.origin_time, cycle - offset, period_ns, touchdown) ||
                !Stamp(state_.origin_time,
                      cycle - 1.0L + state_.duty_factor - offset,
                      period_ns, liftoff))
                return PhaseClockFailure::kEventUnavailable;
        }
        if (touchdown.value <= liftoff.value || liftoff.value < 0)
            return PhaseClockFailure::kEventUnavailable;
        out.valid = true;
        out.cycle_index = static_cast<std::int64_t>(cycle);
        out.liftoff_time = liftoff;
        out.touchdown_time = touchdown;
        return PhaseClockFailure::kNone;
    }
private:
    static std::int64_t AbsNs(std::int64_t value)
    {
        if (value == std::numeric_limits<std::int64_t>::min())
            return std::numeric_limits<std::int64_t>::max();
        return value < 0 ? -value : value;
    }
    // Match TimeNs::FromSeconds exactly: schedule and clock boundaries must
    // use the same integer nanosecond period, including non-binary decimals.
    static bool PeriodNs(double period_s, long double &period_ns)
    {
        const TimeNs quantized = TimeNs::FromSeconds(period_s);
        if (quantized.value <= 0 ||
            quantized.value == std::numeric_limits<std::int64_t>::min())
            return false;
        period_ns = static_cast<long double>(quantized.value);
        return true;
    }
    bool ValidConfig() const
    {
        return config_.phase_time_tolerance.value >= 0;
    }
    static bool ValidObservation(const PhaseClockObservation &observation)
    {
        long double period_ns = 0.0L;
        if (observation.observation_time.value < 0 ||
            !std::isfinite(observation.phase) || observation.phase < 0.0 ||
            observation.phase >= 1.0 || !std::isfinite(observation.period_s) ||
            observation.period_s <= 0.0 || !PeriodNs(observation.period_s, period_ns) ||
            period_ns >= static_cast<long double>(
                std::numeric_limits<std::int64_t>::max()) ||
            !std::isfinite(observation.duty_factor) ||
            observation.duty_factor <= 0.0 || observation.duty_factor >= 1.0)
            return false;
        for (const double offset : observation.leg_offsets)
            if (!std::isfinite(offset) || offset < 0.0 || offset >= 1.0)
                return false;
        return true;
    }
    bool TimingChanged(const PhaseClockObservation &observation) const
    {
        if (observation.period_s != state_.period_s ||
            observation.duty_factor != state_.duty_factor)
            return true;
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            if (observation.leg_offsets[leg] != state_.leg_offsets[leg])
                return true;
        return false;
    }
    static bool ToTimeNs(long double value, TimeNs &out)
    {
        if (!std::isfinite(value) ||
            value <= static_cast<long double>(
                std::numeric_limits<std::int64_t>::min()) ||
            value >= static_cast<long double>(
                std::numeric_limits<std::int64_t>::max()) ||
            value < static_cast<long double>(
                std::numeric_limits<std::int64_t>::min()) + 0.5L ||
            value > static_cast<long double>(
                std::numeric_limits<std::int64_t>::max()) - 0.5L)
            return false;
        out = TimeNs{static_cast<std::int64_t>(std::llround(value))};
        return out.value != std::numeric_limits<std::int64_t>::min();
    }
    static bool ComputeOrigin(
        const PhaseClockObservation &observation, TimeNs &origin)
    {
        long double period_ns = 0.0L;
        if (!PeriodNs(observation.period_s, period_ns))
            return false;
        return ToTimeNs(
            static_cast<long double>(observation.observation_time.value) -
                static_cast<long double>(observation.phase) * period_ns,
            origin);
    }
    TimeNs PhaseResidual(const PhaseClockObservation &observation) const
    {
        long double period_ns = 0.0L;
        if (!PeriodNs(state_.period_s, period_ns))
            return TimeNs{std::numeric_limits<std::int64_t>::max()};
        const long double elapsed =
            static_cast<long double>(observation.observation_time.value) -
            static_cast<long double>(state_.origin_time.value);
        long double predicted = elapsed / period_ns;
        predicted -= std::floor(predicted);
        long double delta = static_cast<long double>(observation.phase) - predicted;
        while (delta > 0.5L)
            delta -= 1.0L;
        while (delta < -0.5L)
            delta += 1.0L;
        TimeNs residual{};
        if (!ToTimeNs(delta * period_ns, residual))
            return TimeNs{std::numeric_limits<std::int64_t>::max()};
        return residual;
    }
    static bool Stamp(TimeNs origin, long double cycle_phase,
                      long double period_ns, TimeNs &out)
    {
        return ToTimeNs(
            static_cast<long double>(origin.value) + cycle_phase * period_ns,
            out);
    }
    PhaseClockConfig config_{};
    PhaseClockSnapshot state_{};
};
} // namespace stage_c
} // namespace go2_terrain
