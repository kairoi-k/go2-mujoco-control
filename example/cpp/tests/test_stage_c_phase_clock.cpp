#include "stage_c/event_schedule.h"
#include "stage_c/phase_clock.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace go2_terrain::stage_c;
namespace
{
void Check(bool ok, const char *message)
{
    if (!ok)
        throw std::runtime_error(message);
}
TimeNs T(double seconds)
{
    return TimeNs::FromSeconds(seconds);
}
PhaseClockObservation Observation(
    double time, double phase, double period = 0.24, double duty = 0.44)
{
    PhaseClockObservation observation;
    observation.observation_time = T(time);
    observation.phase = phase;
    observation.period_s = period;
    observation.duty_factor = duty;
    observation.leg_offsets = {0.0, 0.46, 0.46, 0.0};
    return observation;
}
} // namespace
int main()
{
    try
    {
        PhaseClockConfig config;
        config.phase_time_tolerance = TimeNs{100};
        PhaseClock clock(config);
        const auto first = clock.Capture(Observation(1.0, 0.25));
        Check(first.accepted && first.epoch == 1 && !first.epoch_changed,
              "initial capture failed");
        const auto origin = clock.snapshot().origin_time;
        const auto next = clock.Capture(Observation(1.12, 0.75));
        Check(next.accepted && next.epoch == 1 && !next.timing_changed &&
                  next.phase_residual_valid && std::abs(next.phase_residual.value) <= 100 &&
                  clock.snapshot().origin_time == origin,
              "same fixed clock did not preserve origin");
        const auto wrapped = clock.Capture(Observation(1.24, 0.25));
        Check(wrapped.accepted && clock.snapshot().epoch == 1 &&
                  clock.snapshot().origin_time == origin,
              "wrapped phase reset the clock");
        auto drift = Observation(1.30, 0.25);
        const auto drift_result = clock.Capture(drift);
        Check(!drift_result.accepted &&
                  drift_result.failure == PhaseClockFailure::kPhaseDrift &&
                  drift_result.phase_residual_valid && drift_result.phase_residual.value != 0 &&
                  clock.snapshot().epoch == 1 &&
                  clock.snapshot().last_observation_time == T(1.24),
              "phase drift was silently accepted or retimed");
        auto changed = Observation(1.36, 0.60, 0.25);
        const auto new_epoch = clock.Capture(changed);
        Check(new_epoch.accepted && new_epoch.epoch_changed &&
                  new_epoch.timing_changed && new_epoch.epoch == 2 &&
                  clock.snapshot().origin_time != origin,
              "period change did not explicitly start an epoch");
        const auto committed_before = clock.snapshot();
        auto duty_change = Observation(1.40, 0.41, 0.25, 0.50);
        const auto conflict = clock.Capture(duty_change, true);
        Check(!conflict.accepted &&
                  conflict.failure == PhaseClockFailure::kCommitmentConflict &&
                  conflict.timing_changed && conflict.epoch == committed_before.epoch &&
                  clock.snapshot().epoch == committed_before.epoch &&
                  clock.snapshot().duty_factor == committed_before.duty_factor,
              "timing change mutated an active committed clock");
        PhaseClockLegEvent event;
        Check(clock.NextEvent(0, T(1.36), event) == PhaseClockFailure::kNone &&
                  event.valid && event.touchdown_time >= T(1.36) &&
                  event.liftoff_time < T(1.36) && event.liftoff_time >= T(0.0) &&
                  event.touchdown_time > event.liftoff_time,
              "next event did not expose a future touchdown and past anchor");
        PhaseClockLegEvent later;
        Check(clock.NextEvent(0, T(1.60), later) == PhaseClockFailure::kNone &&
                  later.valid && later.touchdown_time >= T(1.60) &&
                  later.touchdown_time > event.touchdown_time,
              "next event query fabricated or retimed a past touchdown");
        PhaseClockLegEvent rewound;
        Check(clock.NextEvent(0, T(1.20), rewound) == PhaseClockFailure::kTimeRewind,
              "past event query was not rejected");

        // Non-binary decimal periods must use the same integer-nanosecond
        // boundaries as BuildFixedSchedulePreview. Compare both TD and LO
        // directly so a floating-point boundary cannot skip an event.
        PhaseClock decimal_clock;
        const auto decimal_observation = Observation(1.017, 0.123, 0.14, 0.44);
        const auto decimal_capture = decimal_clock.Capture(decimal_observation);
        Check(decimal_capture.accepted, "decimal period capture failed");
        FixedSchedulePreviewRequest preview_request;
        preview_request.start = decimal_observation.observation_time;
        preview_request.end = T(1.65);
        preview_request.phase_zero_time = decimal_clock.snapshot().origin_time;
        preview_request.period = TimeNs::FromSeconds(0.14);
        preview_request.max_interval = T(0.02);
        preview_request.schedule_epoch = 7;
        preview_request.duty = decimal_observation.duty_factor;
        preview_request.leg_offsets = decimal_observation.leg_offsets;
        const auto preview = BuildFixedSchedulePreview(preview_request);
        Check(preview.complete && preview.events.valid(false),
              "decimal schedule preview failed");
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            PhaseClockLegEvent clock_event;
            Check(decimal_clock.NextEvent(leg, preview_request.start, clock_event) ==
                      PhaseClockFailure::kNone,
                  "decimal clock next event failed");
            const TouchdownEvent *preview_event = nullptr;
            for (const auto &candidate : preview.events.events)
                if (static_cast<std::size_t>(candidate.id.leg) == leg &&
                    candidate.touchdown_time >= preview_request.start)
                {
                    preview_event = &candidate;
                    break;
                }
            Check(preview_event != nullptr, "decimal preview omitted leg event");
            Check(clock_event.touchdown_time == preview_event->touchdown_time &&
                      clock_event.liftoff_time == preview_event->liftoff_time,
                  "decimal clock and preview boundaries differ");
        }

        auto invalid = Observation(1.50, 1.0);
        Check(!clock.Capture(invalid).accepted,
              "phase one was accepted");
        invalid = Observation(1.50, 0.20);
        invalid.period_s = 0.0;
        Check(clock.Capture(invalid).failure == PhaseClockFailure::kInvalidObservation,
              "zero period was accepted");
        invalid = Observation(1.50, 0.20);
        invalid.leg_offsets[2] = std::numeric_limits<double>::quiet_NaN();
        Check(clock.Capture(invalid).failure == PhaseClockFailure::kInvalidObservation,
              "unknown offset was accepted");
        std::cout << "Stage C phase clock checks passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "FAIL: " << error.what() << "\n";
        return 1;
    }
}
