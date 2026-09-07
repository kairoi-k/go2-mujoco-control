#pragma once
#include "centroidal_subproblem.h"
#include "locomotion_kernel.h"
namespace go2_terrain { namespace stage_c {
// The phase authority owns this origin and epoch. Replanning must not reset
// either to its capture time. Offsets are captured once from the gait kernel.
struct FixedSchedulePreviewRequest {
    TimeNs start{}, end{}, phase_zero_time{}, period{}, max_interval{};
    std::uint64_t schedule_epoch = 0;
    double duty = 0.0;
    std::array<double,4> leg_offsets{};
};
struct FixedSchedulePreview {
    JointPlannerFailure failure = JointPlannerFailure::kInvalidInput;
    std::vector<TimeNs> grid;
    std::vector<FixedScheduleInterval> intervals;
    TouchdownEventTable events;
    // Schedule completion is not a foothold or execution certificate: new
    // events intentionally have unknown targets until candidate generation.
    bool complete = false;
};
inline std::array<double,4> CaptureGaitOffsets(go2_control::GaitPattern pattern) {
    std::array<double,4> offsets{};
    for(std::size_t l=0;l<4;++l)
        offsets[l]=go2_control::GaitLegPhase(l,0.0,pattern);
    return offsets;
}
inline FixedSchedulePreview BuildFixedSchedulePreview(
    const FixedSchedulePreviewRequest &r) {
    FixedSchedulePreview out;
    if(r.start.value<0 || r.end<=r.start || r.period.value<=0 ||
       r.max_interval.value<=0 || !r.schedule_epoch ||
       !std::isfinite(r.duty) || r.duty<=0 || r.duty>=1 ||
       static_cast<long double>(r.end.value)-r.start.value>128.L*r.period.value)
        return out;
    for(double offset:r.leg_offsets)
        if(!std::isfinite(offset) || offset<0 || offset>=1) return out;
    struct Stance { int leg; TimeNs td,lo; int event=-1; };
    std::vector<Stance> stances;
    std::vector<TimeNs> boundaries{r.start,r.end};
    const auto stamp=[&](long double cycle, double offset) {
        long double t=static_cast<long double>(r.phase_zero_time.value)+
            (cycle-offset)*r.period.value;
        if(t<=static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
           t>=static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
            return TimeNs{std::numeric_limits<std::int64_t>::min()};
        return TimeNs{static_cast<std::int64_t>(std::llround(t))};
    };
    for(int l=0;l<4;++l) {
        const long double first=std::floor((static_cast<long double>(r.start.value)-
            r.phase_zero_time.value)/r.period.value+r.leg_offsets[l])-1;
        const long double last=std::ceil((static_cast<long double>(r.end.value)-
            r.phase_zero_time.value)/r.period.value+r.leg_offsets[l]);
        if(first < -2 || last>=std::numeric_limits<std::uint32_t>::max()) return out;
        for(long double cycle=first;cycle<=last;++cycle) {
            const TimeNs td=stamp(cycle,r.leg_offsets[l]);
            const TimeNs lo=stamp(cycle+r.duty,r.leg_offsets[l]);
            if(lo<=r.start || td>=r.end) continue;
            if(td.value==std::numeric_limits<std::int64_t>::min() || lo<=td)
                return out;
            stances.push_back({l,td,lo,-1});
            if(td>r.start) boundaries.push_back(td);
            if(lo<r.end) boundaries.push_back(lo);
            if(td>=r.start) {
                if(cycle<0 || out.events.events.size()>=kStageCMaxEvents) {
                    out.failure=JointPlannerFailure::kCoverageIncomplete; return out;
                }
                TouchdownEvent event;
                event.id={r.schedule_epoch,static_cast<go2::Leg>(l),
                          static_cast<std::uint32_t>(cycle)+1};
                event.touchdown_time=td;
                event.contact_interval_end=lo;
                event.liftoff_time=stamp(cycle-1+r.duty,r.leg_offsets[l]);
                if(event.liftoff_time.value<0) {
                    out.failure=JointPlannerFailure::kCoverageIncomplete;return out;
                }
                event.liftoff_valid=true;
                out.events.events.push_back(event);
            }
        }
    }
    std::sort(out.events.events.begin(),out.events.events.end(),
        [](const TouchdownEvent &a,const TouchdownEvent &b) {
            return a.touchdown_time==b.touchdown_time ? a.id<b.id :
                a.touchdown_time<b.touchdown_time;
        });
    for(auto &s:stances)
        for(std::size_t e=0;e<out.events.events.size();++e)
            if(static_cast<int>(out.events.events[e].id.leg)==s.leg &&
               out.events.events[e].touchdown_time==s.td) s.event=static_cast<int>(e);
    std::sort(boundaries.begin(),boundaries.end());
    boundaries.erase(std::unique(boundaries.begin(),boundaries.end()),boundaries.end());
    out.grid.push_back(r.start);
    for(std::size_t b=1;b<boundaries.size();++b) {
        const auto duration=boundaries[b].value-boundaries[b-1].value;
        const auto pieces=1+(duration-1)/r.max_interval.value;
        if(pieces>128 || out.grid.size()+static_cast<std::size_t>(pieces)>129) {
            out.failure=JointPlannerFailure::kCoverageIncomplete; return out;
        }
        for(std::int64_t k=1;k<=pieces;++k) {
            const TimeNs next{boundaries[b-1].value+
                static_cast<std::int64_t>((static_cast<long double>(duration)*k)/pieces)};
            FixedScheduleInterval interval;
            interval.start=out.grid.back(); interval.end=next;
            for(const auto &s:stances)
                if(interval.start>=s.td && interval.start<s.lo) {
                    interval.contact[s.leg]=true; interval.event_index[s.leg]=s.event;
                }
            out.intervals.push_back(interval); out.grid.push_back(next);
        }
    }
    out.failure=JointPlannerFailure::kNone; out.complete=true;
    return out;
}
}} // namespace
