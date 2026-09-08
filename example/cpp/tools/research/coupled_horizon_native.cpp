// Research-only native evaluator for the coupled fixed-prefix horizon.
// The caller owns privilege/terrain admission; this file only evaluates the
// existing MuJoCo model and the Python ActualModelHorizon contract.
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
namespace {
constexpr int kSteps = 16;
constexpr int kPrefixSteps = 6;
constexpr int kNu = 12;
constexpr int kNq = 19;
constexpr int kNv = 18;
constexpr int kNa = 0;
constexpr int kObserveSize = 48;
constexpr int kGSize = kSteps * 2 * kObserveSize + 4;
constexpr int kControlSize = kSteps * kNu;
constexpr double kPi = 3.141592653589793238462643383279502884;
static_assert(kGSize == 1540, "ActualModelHorizon constraint size");
static_assert(kControlSize == 192, "ActualModelHorizon control size");
struct InputError : std::runtime_error {
  using std::runtime_error::runtime_error;
};
struct NumericalError : std::runtime_error {
  using std::runtime_error::runtime_error;
};
using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;
void WriteError(char* error, int cap, const char* message) noexcept {
  if (!error || cap <= 0) return;
  std::snprintf(error, static_cast<size_t>(cap), "%s", message ? message : "native evaluator failure");
  error[cap - 1] = '\0';
}
void WriteError(char* error, int cap, const std::string& message) noexcept {
  WriteError(error, cap, message.c_str());
}
template <typename T>
bool AllFinite(const T* values, int count) {
  if (!values || count < 0) return false;
  for (int i = 0; i < count; ++i) {
    if (!std::isfinite(static_cast<double>(values[i]))) return false;
  }
  return true;
}
void RequireNoCallbacks() {
  // Python installs its profiling timer after model creation. Retain that timer;
  // reject callbacks that supply forces, controls, sensor values or actuation.
  if (mjcb_passive || mjcb_control || mjcb_contactfilter || mjcb_sensor ||
      mjcb_act_dyn || mjcb_act_gain || mjcb_act_bias) {
    throw InputError("MuJoCo callback active");
  }
}
class Handle {
 public:
  Handle(const char* xml, const double* state, int state_n,
         const double* baseline, double terminal_vy)
      : model_(nullptr, mj_deleteModel),
        initial_(nullptr, mj_deleteData),
        work_(nullptr, mj_deleteData),
        pre_(nullptr, mj_deleteData),
        post_(nullptr, mj_deleteData),
        baseline_{} ,
        terminal_vy_(terminal_vy),
        state_size_(0),
        va_{},
        qa_{},
        gids_{} {
    if (!xml || !*xml) throw InputError("xml path required");
    if (!state || state_n < 0) throw InputError("invalid integration state");
    if (!baseline) throw InputError("baseline controls required");
    if (!std::isfinite(terminal_vy) || terminal_vy <= 0) {
      throw InputError("finite positive terminal bound required");
    }
    for (int i = 0; i < kControlSize; ++i) {
      if (!std::isfinite(baseline[i])) throw InputError("nonfinite baseline control");
      baseline_[i] = baseline[i];
    }
    char load_error[2048]{};
    model_.reset(mj_loadXML(xml, nullptr, load_error, sizeof(load_error)));
    if (!model_) {
      throw InputError(load_error[0] ? load_error : "mj_loadXML failed");
    }
    RequireNoCallbacks();
    if (model_->nq != kNq || model_->nv != kNv || model_->nu != kNu ||
        model_->na != kNa || model_->nplugin != 0) {
      throw InputError("requires direct-torque Go2 nq19 nv18 nu12 na0 nplugin0");
    }
    if (std::abs(static_cast<double>(model_->opt.timestep) - .002) > 1e-15) {
      throw InputError("unchanged2ms model required");
    }
    state_size_ = mj_stateSize(model_.get(), mjSTATE_INTEGRATION);
    if (state_n != state_size_) throw InputError("integration state size mismatch");
    for (int i = 0; i < kNu; ++i) {
      const int joint = model_->actuator_trnid[2 * i];
      if (joint < 0 || joint >= model_->njnt) throw InputError("invalid actuator joint");
      qa_[i] = model_->jnt_qposadr[joint];
      va_[i] = model_->jnt_dofadr[joint];
      if (qa_[i] < 0 || qa_[i] >= model_->nq || va_[i] < 0 || va_[i] >= model_->nv) {
        throw InputError("invalid actuator address");
      }
    }
    const char* foot_names[4] = {"FR", "FL", "RR", "RL"};
    for (int i = 0; i < 4; ++i) {
      gids_[i] = mj_name2id(model_.get(), mjOBJ_GEOM, foot_names[i]);
      if (gids_[i] < 0) throw InputError("foot geometry missing");
    }
    if (!AllFinite(state, state_n)) throw InputError("nonfinite integration state");
    initial_.reset(mj_makeData(model_.get()));
    work_.reset(mj_makeData(model_.get()));
    pre_.reset(mj_makeData(model_.get()));
    post_.reset(mj_makeData(model_.get()));
    if (!initial_ || !work_ || !pre_ || !post_) throw InputError("mjData allocation failed");
    std::vector<mjtNum> state_buffer(static_cast<size_t>(state_n));
    for (int i = 0; i < state_n; ++i) state_buffer[static_cast<size_t>(i)] = static_cast<mjtNum>(state[i]);
    mj_setState(model_.get(), initial_.get(), state_buffer.data(), mjSTATE_INTEGRATION);
    if (!std::isfinite(static_cast<double>(initial_->time)) ||
        !AllFinite(initial_->qpos, model_->nq) || !AllFinite(initial_->qvel, model_->nv) ||
        !AllFinite(initial_->qacc, model_->nv)) {
      throw InputError("invalid initial dynamics state");
    }
  }
  int gsize() const noexcept { return kGSize; }
  void Evaluate(const double* controls, double* cost, double* output) {
    RequireNoCallbacks();
    if (!controls || !cost || !output) throw InputError("null evaluation buffer");
    for (int i = 0; i < kControlSize; ++i) {
      if (!std::isfinite(controls[i])) throw InputError("nonfinite control");
    }
    for (int i = 0; i < kPrefixSteps * kNu; ++i) {
      if (controls[i] != baseline_[i]) throw InputError("fixed prefix altered");
    }
    if (!mj_copyData(work_.get(), model_.get(), initial_.get())) {
      throw NumericalError("initial data copy failed");
    }
    std::array<double, kGSize> values{};
    int offset = 0;
    for (int step = 0; step < kSteps; ++step) {
      const double* tau = controls + step * kNu;
      if (!mj_copyData(pre_.get(), model_.get(), work_.get())) {
        throw NumericalError("pre-step data copy failed");
      }
      for (int j = 0; j < kNu; ++j) pre_->ctrl[j] = static_cast<mjtNum>(tau[j]);
      mj_forward(model_.get(), pre_.get());
      CheckData(pre_.get());
      Observe(pre_.get(), values.data() + offset);
      offset += kObserveSize;
      for (int j = 0; j < kNu; ++j) work_->ctrl[j] = static_cast<mjtNum>(tau[j]);
      mj_step(model_.get(), work_.get());
      if (!mj_copyData(post_.get(), model_.get(), work_.get())) {
        throw NumericalError("post-step data copy failed");
      }
      mj_forward(model_.get(), post_.get());
      CheckData(post_.get());
      Observe(post_.get(), values.data() + offset);
      offset += kObserveSize;
      const double expected_time = static_cast<double>(initial_->time) +
                                   (step + 1) * static_cast<double>(model_->opt.timestep);
      if (std::abs(static_cast<double>(work_->time) - expected_time) > 1e-10) {
        throw NumericalError("absolute clock mismatch");
      }
    }
    values[static_cast<size_t>(offset++)] = terminal_vy_ - std::abs(static_cast<double>(work_->qvel[1]));
    for (int j = 0; j < 3; ++j) {
      values[static_cast<size_t>(offset++)] = .3 - std::abs(static_cast<double>(work_->qvel[3 + j]));
    }
    if (offset != kGSize) throw NumericalError("constraint size mismatch");
    double control_cost = 0;
    for (int i = 0; i < kControlSize; ++i) {
      const double normalized = (controls[i] - baseline_[i]) / 35.0;
      control_cost += normalized * normalized;
    }
    const double vy = static_cast<double>(work_->qvel[1]);
    const double computed_cost = .5 * control_cost + .5 * (vy / .02) * (vy / .02);
    if (!std::isfinite(computed_cost)) throw NumericalError("nonfinite cost");
    for (int i = 0; i < kGSize; ++i) {
      if (!std::isfinite(values[static_cast<size_t>(i)])) throw NumericalError("nonfinite constraint");
    }
    *cost = computed_cost;
    for (int i = 0; i < kGSize; ++i) output[i] = values[static_cast<size_t>(i)];
  }
 private:
  void CheckData(const mjData* data) const {
    if (!data || !std::isfinite(static_cast<double>(data->time)) ||
        !AllFinite(data->qpos, model_->nq) || !AllFinite(data->qvel, model_->nv) ||
        !AllFinite(data->qacc, model_->nv)) {
      throw NumericalError("nonfinite dynamics state");
    }
    const int bad[] = {mjWARN_BADQPOS, mjWARN_BADQVEL, mjWARN_BADQACC, mjWARN_BADCTRL};
    for (int warning : bad) {
      if (data->warning[warning].number) throw NumericalError("MuJoCo numerical warning");
    }
  }
  void Contact(const mjData* data, std::array<double, 4>* forces, double* nonfoot) const {
    forces->fill(0.0);
    *nonfoot = 0.0;
    for (int i = 0; i < data->ncon; ++i) {
      const mjContact& contact = data->contact[i];
      mjtNum force[6]{};
      mj_contactForce(model_.get(), data, i, force);
      std::array<int, 4> legs{};
      int leg_count = 0;
      for (int leg = 0; leg < 4; ++leg) {
        if (gids_[leg] == contact.geom1 || gids_[leg] == contact.geom2) {
          legs[leg_count++] = leg;
          (*forces)[leg] += static_cast<double>(force[0]);
        }
      }
      bool allowed = leg_count == 1;
      if (allowed) {
        const int foot = gids_[legs[0]];
        const int other = contact.geom1 == foot ? contact.geom2 : contact.geom1;
        const int foot_body = model_->geom_bodyid[foot];
        const int other_body = model_->geom_bodyid[other];
        allowed = model_->body_rootid[other_body] != model_->body_rootid[foot_body];
      }
      if (!allowed) {
        *nonfoot += std::sqrt(static_cast<double>(force[0]) * static_cast<double>(force[0]) +
                              static_cast<double>(force[1]) * static_cast<double>(force[1]) +
                              static_cast<double>(force[2]) * static_cast<double>(force[2]));
      }
    }
  }
  void Observe(const mjData* data, double* output) const {
    std::array<double, 4> forces{};
    double nonfoot = 0;
    Contact(data, &forces, &nonfoot);
    int offset = 0;
    for (double force : forces) output[offset++] = (180.0 - force) / 180.0;
    for (double force : forces) output[offset++] = force / 180.0;
    for (int i = 0; i < kNu; ++i) {
      output[offset++] = (30.0 - std::abs(static_cast<double>(data->qvel[va_[i]]))) / 30.0;
    }
    for (int i = 0; i < kNu; ++i) {
      const double q = static_cast<double>(data->qpos[qa_[i]]);
      output[offset++] = q - static_cast<double>(model_->jnt_range[2 * model_->actuator_trnid[2 * i]]);
    }
    for (int i = 0; i < kNu; ++i) {
      const double q = static_cast<double>(data->qpos[qa_[i]]);
      output[offset++] = static_cast<double>(model_->jnt_range[2 * model_->actuator_trnid[2 * i] + 1]) - q;
    }
    const double w = static_cast<double>(data->qpos[3]);
    const double x = static_cast<double>(data->qpos[4]);
    const double y = static_cast<double>(data->qpos[5]);
    const double z = static_cast<double>(data->qpos[6]);
    const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double pitch_arg = std::max(-1.0, std::min(1.0, 2.0 * (w * y - z * x)));
    const double pitch = std::asin(pitch_arg);
    output[offset++] = static_cast<double>(data->qpos[2]) - .28;
    output[offset++] = kPi / 12.0 - std::abs(roll);
    output[offset++] = kPi / 12.0 - std::abs(pitch);
    output[offset++] = 1e-6 - nonfoot;
    if (offset != kObserveSize) throw NumericalError("observation size mismatch");
  }
  ModelPtr model_;
  DataPtr initial_;
  DataPtr work_;
  DataPtr pre_;
  DataPtr post_;
  std::array<double, kControlSize> baseline_;
  double terminal_vy_;
  int state_size_;
  std::array<int, kNu> va_;
  std::array<int, kNu> qa_;
  std::array<int, 4> gids_;
};
}  // namespace
extern "C" {
void* gh_create(const char* xml, const double* state, int state_n,
                const double* baseline192, double terminal_vy,
                char* error, int cap) noexcept {
  try {
    return new Handle(xml, state, state_n, baseline192, terminal_vy);
  } catch (const std::exception& exception) {
    WriteError(error, cap, exception.what());
  } catch (...) {
    WriteError(error, cap, "unknown native evaluator failure");
  }
  return nullptr;
}
void gh_destroy(void* opaque) noexcept {
  try {
    delete static_cast<Handle*>(opaque);
  } catch (...) {
  }
}
int gh_gsize(void* opaque) noexcept {
  return opaque ? kGSize : 0;
}
int gh_eval(void* opaque, const double* controls192, double* cost,
            double* g, int gsize, char* error, int cap) noexcept {
  if (!opaque || !controls192 || !cost || !g) {
    WriteError(error, cap, "null evaluator handle");
    return 1;
  }
  if (gsize != kGSize) {
    WriteError(error, cap, "constraint size mismatch");
    return 1;
  }
  try {
    std::array<double, kGSize> values{};
    double value = 0;
    static_cast<Handle*>(opaque)->Evaluate(controls192, &value, values.data());
    *cost = value;
    for (int i = 0; i < kGSize; ++i) g[i] = values[static_cast<size_t>(i)];
    return 0;
  } catch (const NumericalError& exception) {
    WriteError(error, cap, exception.what());
    return 2;
  } catch (const std::exception& exception) {
    WriteError(error, cap, exception.what());
    return 1;
  } catch (...) {
    WriteError(error, cap, "unknown native evaluator failure");
    return 1;
  }
}
}  // extern "C"
