#include "state_elapsed_clock.h"
#include "stage_c/phase_clock.h"
#include <cmath>
#include <cstdio>
#include <limits>
int main() {
 using namespace go2_terrain::stage_c;
 go2_trot::StateElapsedClock clock;
 PhaseClock events;
 PhaseClockObservation observation;
 observation.period_s=.14;observation.duty_factor=.46;
 observation.leg_offsets={0.,.5,.5,0.};
 double phase=.409084804, elapsed=0;
 const double times[]={21.054,21.054,21.064,21.066,21.060,21.066,21.068};
 for(int i=0;i<7;++i) {
  auto step=clock.Step(times[i],.008);
  if(i==4) {if(step.observation_valid||step.elapsed_s!=0)return 1;continue;}
  if(!step.observation_valid)return 2;
  if(i==2 && (step.integration_valid||step.integration_dt_s!=0||std::abs(step.elapsed_s-.010)>1e-12))return 3;
  elapsed+=step.elapsed_s;
  phase=std::fmod(phase+step.elapsed_s/.14,1.);
  observation.phase=phase;observation.observation_time=TimeNs::FromSeconds(times[i]);
  auto result=events.Capture(observation,i!=0);
  if(!result.accepted||result.epoch!=1||std::abs(result.phase_residual.value)>1)return 4;
 }
 if(std::abs(elapsed-.014)>1e-12)return 5;
 if(clock.Step(std::numeric_limits<double>::quiet_NaN(),.008).observation_valid)return 6;
 auto next=clock.Step(21.070,.008);
 if(!next.integration_valid||std::abs(next.elapsed_s-.002)>1e-12)return 7;
 std::puts("absolute elapsed, bounded integration, duplicate/rewind and committed epoch checks passed");
}
