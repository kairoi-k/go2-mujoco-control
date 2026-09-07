#pragma once
#include <cmath>
namespace go2_trot {
struct StateElapsedStep {
 double elapsed_s=0.0;
 double integration_dt_s=0.0;
 bool observation_valid=false;
 bool integration_valid=false;
};
// Absolute schedule time is not an integration budget. A fresh gapped sample
// advances elapsed time but does not authorize a large estimator/control step.
// Rewinds never move the high-water mark; reset requires a new owner instance.
class StateElapsedClock {
 public:
 StateElapsedStep Step(double state_s,double max_integration_s) {
  StateElapsedStep out;
  if(!std::isfinite(state_s)||state_s<0 || !std::isfinite(max_integration_s)||max_integration_s<=0) return out;
  if(!initialized_) {initialized_=true;previous_=state_s;out.observation_valid=true;return out;}
  if(state_s<previous_) return out;
  out.observation_valid=true;
  out.elapsed_s=state_s-previous_;
  previous_=state_s;
  out.integration_valid=out.elapsed_s>1e-6 && out.elapsed_s<=max_integration_s;
  if(out.integration_valid)out.integration_dt_s=out.elapsed_s;
  return out;
 }
 private:
 bool initialized_=false;
 double previous_=0.0;
};
}
