#include "stage_c/event_schedule.h"
#include <iostream>
#include <stdexcept>
using namespace go2_terrain::stage_c;
namespace {
void Check(bool ok,const char *name) { if(!ok) throw std::runtime_error(name); }
TimeNs T(double t) { return TimeNs::FromSeconds(t); }
void KernelOracle(const FixedSchedulePreviewRequest &r,const FixedSchedulePreview &p) {
    Check(p.complete,"preview complete");
    Check(p.grid.front()==r.start && p.grid.back()==r.end,"absolute endpoints");
    for(const auto &interval:p.intervals) {
        Check(interval.end>interval.start && interval.end.value-interval.start.value<=r.max_interval.value,"positive bounded dt");
        const double midpoint=(interval.start.seconds()+interval.end.seconds())/2;
        std::array<std::array<bool,4>,1> contact;
        go2_control::FillTrotContactSchedulePhase(
            (midpoint-r.phase_zero_time.seconds())/r.period.seconds(),
            r.period.seconds(),r.duty,1,0,contact,go2_control::GaitPattern::kRunningTrot);
        Check(interval.contact==contact[0],"same production phase authority");
        for(int l=0;l<4;++l) if(interval.contact[l] && interval.event_index[l]>=0) {
            const auto &e=p.events.events.at(interval.event_index[l]);
            Check(e.touchdown_time<=interval.start && e.contact_interval_end>=interval.end,"event interval identity");
            Check(static_cast<int>(e.id.leg)==l,"event leg identity");
        }
    }
}
}
int main() {
 try {
    FixedSchedulePreviewRequest r;
    r.start=T(1.017);r.end=T(1.7);r.phase_zero_time=T(0);
    r.period=T(.24);r.max_interval=T(.04);r.schedule_epoch=7;r.duty=.44;
    r.leg_offsets=CaptureGaitOffsets(go2_control::GaitPattern::kRunningTrot);
    const auto p=BuildFixedSchedulePreview(r);KernelOracle(r,p);
    Check(p.events.events.size()>4,"multiple touchdowns");
    bool aerial=false,in_flight=false;
    for(const auto &i:p.intervals) aerial|=std::count(i.contact.begin(),i.contact.end(),true)==0;
    for(const auto &e:p.events.events) {
        Check(e.liftoff_valid && e.liftoff_time<e.touchdown_time,"physical liftoff precedes touchdown");
        Check(!e.target_world.valid,"schedule cannot invent a foothold");
        in_flight|=e.liftoff_time<r.start;
    }
    Check(aerial && in_flight,"aerial plus already-in-flight coverage");
    auto shifted=r;shifted.start=T(1.101);shifted.end=T(1.8);
    auto q=BuildFixedSchedulePreview(shifted);KernelOracle(shifted,q);
    for(const auto &a:p.events.events) if(a.touchdown_time>=shifted.start) {
        bool found=false;
        for(const auto &b:q.events.events) if(a.id==b.id) {
            found=true;Check(a.touchdown_time==b.touchdown_time && a.liftoff_time==b.liftoff_time,"replanning preserves absolute event identity");
        }
        Check(found,"overlapping future event preserved");
    }
    auto edge=r;edge.end=p.events.events[2].touchdown_time;
    auto endpoint=BuildFixedSchedulePreview(edge);
    Check(endpoint.complete,"endpoint preview");
    for(const auto &e:endpoint.events.events)Check(e.touchdown_time<edge.end,"terminal state has no force interval touchdown");
    auto invalid=r;invalid.period={0};Check(!BuildFixedSchedulePreview(invalid).complete,"zero period rejected");
    invalid=r;invalid.leg_offsets[0]=std::numeric_limits<double>::quiet_NaN();Check(!BuildFixedSchedulePreview(invalid).complete,"unknown offset rejected");
    invalid=r;invalid.end=invalid.start;Check(!BuildFixedSchedulePreview(invalid).complete,"empty horizon rejected");
    std::cout<<"event preview: kernel oracle, multiple TD, flight, absolute prefix and endpoint checks passed; events="<<p.events.events.size()<<" intervals="<<p.intervals.size()<<"\n";
    return 0;
 } catch(const std::exception &e) {std::cerr<<e.what()<<"\n";return 1;}
}
