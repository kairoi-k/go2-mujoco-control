// Research-only native evaluator for a whole-body fixed-step horizon.
// The caller owns terrain privilege, optimization and final torque checks. This
// file only replays the unchanged MuJoCo model and emits the existing 48-entry
// pre/post observation inequalities for each requested step.
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
constexpr int kNu = 12;
constexpr int kNq = 19;
constexpr int kNv = 18;
constexpr int kNa = 0;
constexpr int kObserveSize = 48;
constexpr int kBodyReferenceSize = 13;
constexpr int kFootReferenceSize = 12;
constexpr double kPi = 3.141592653589793238462643383279502884;
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
  std::snprintf(error, static_cast<size_t>(cap), "%s",
                message ? message : "native evaluator failure");
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
  // Keep the host timer callback available. Reject callbacks that can change
  // controls, forces, contacts, sensors or actuator dynamics.
  if (mjcb_passive || mjcb_control || mjcb_contactfilter || mjcb_sensor ||
      mjcb_act_dyn || mjcb_act_gain || mjcb_act_bias) {
    throw InputError("MuJoCo callback active");
  }
}
class Handle {
 public:
  Handle(const char* xml, const double* state, int state_n, int steps,
         int prefix, const double* baseline, const double* body_refs,
         const double* foot_refs, bool top_support_only = false)
      : model_(nullptr, mj_deleteModel),
        initial_(nullptr, mj_deleteData),
        work_(nullptr, mj_deleteData),
        pre_(nullptr, mj_deleteData),
        post_(nullptr, mj_deleteData),
        top_support_only_(top_support_only),
        steps_(steps),
        prefix_(prefix),
        state_size_(0),
        gsize_(0),
        baseline_(),
        body_refs_(),
        foot_refs_(),
        va_(),
        qa_(),
        gids_() {
    if (!xml || !*xml) throw InputError("xml path required");
    if (!state || state_n < 0) throw InputError("invalid integration state");
    if (steps < 1 || steps > 200) throw InputError("steps must be in 1..200");
    if (prefix < 0 || prefix > steps) throw InputError("prefix out of range");
    if (!baseline || !body_refs || !foot_refs)
      throw InputError("reference buffers required");
    char load_error[2048]{};
    model_.reset(mj_loadXML(xml, nullptr, load_error, sizeof(load_error)));
    if (!model_)
      throw InputError(load_error[0] ? load_error : "mj_loadXML failed");
    RequireNoCallbacks();
    if (model_->nq != kNq || model_->nv != kNv || model_->nu != kNu ||
        model_->na != kNa || model_->nplugin != 0) {
      throw InputError("requires direct-torque Go2 nq19 nv18 nu12 na0 nplugin0");
    }
    if (std::abs(static_cast<double>(model_->opt.timestep) - .002) > 1e-15)
      throw InputError("unchanged2ms model required");
    if (top_support_only_) {
      for (int g=0; g<model_->ngeom; ++g) {
        if (model_->geom_bodyid[g]!=0 || (!model_->geom_contype[g] && !model_->geom_conaffinity[g])) continue;
        if (model_->geom_type[g]!=mjGEOM_PLANE && model_->geom_type[g]!=mjGEOM_BOX)
          throw InputError("unknown world support geometry");
        const mjtNum* q=model_->geom_quat+4*g;
        if (std::abs(q[1])+std::abs(q[2])+std::abs(q[3])>1e-12)
          throw InputError("only axis-aligned known top surfaces supported");
      }
    }
    state_size_ = mj_stateSize(model_.get(), mjSTATE_INTEGRATION);
    if (state_n != state_size_)
      throw InputError("integration state size mismatch");
    gsize_ = steps_ * 2 * kObserveSize;
    const int control_size = steps_ * kNu;
    const int body_size = steps_ * kBodyReferenceSize;
    const int foot_size = steps_ * kFootReferenceSize;
    if (!AllFinite(baseline, control_size) ||
        !AllFinite(body_refs, body_size) || !AllFinite(foot_refs, foot_size)) {
      throw InputError("nonfinite controls or references");
    }
    baseline_.assign(baseline, baseline + control_size);
    foot_refs_.assign(foot_refs, foot_refs + foot_size);
    body_refs_.resize(static_cast<size_t>(body_size));
    for (int step = 0; step < steps_; ++step) {
      const double* input = body_refs + step * kBodyReferenceSize;
      double* stored = body_refs_.data() + step * kBodyReferenceSize;
      for (int i = 0; i < kBodyReferenceSize; ++i) stored[i] = input[i];
      const double norm = std::sqrt(
          input[3] * input[3] + input[4] * input[4] +
          input[5] * input[5] + input[6] * input[6]);
      if (!std::isfinite(norm) || norm <= 1e-12)
        throw InputError("body reference quaternion has zero norm");
      for (int i = 3; i < 7; ++i) stored[i] /= norm;
    }
    for (int i = 0; i < kNu; ++i) {
      const int joint = model_->actuator_trnid[2 * i];
      if (joint < 0 || joint >= model_->njnt)
        throw InputError("invalid actuator joint");
      qa_[i] = model_->jnt_qposadr[joint];
      va_[i] = model_->jnt_dofadr[joint];
      if (qa_[i] < 0 || qa_[i] >= model_->nq || va_[i] < 0 || va_[i] >= model_->nv)
        throw InputError("invalid actuator address");
    }
    const char* foot_names[4] = {"FR", "FL", "RR", "RL"};
    for (int i = 0; i < 4; ++i) {
      gids_[i] = mj_name2id(model_.get(), mjOBJ_GEOM, foot_names[i]);
      if (gids_[i] < 0) throw InputError("foot geometry missing");
    }
    if (!AllFinite(state, state_n))
      throw InputError("nonfinite integration state");
    initial_.reset(mj_makeData(model_.get()));
    work_.reset(mj_makeData(model_.get()));
    pre_.reset(mj_makeData(model_.get()));
    post_.reset(mj_makeData(model_.get()));
    if (!initial_ || !work_ || !pre_ || !post_)
      throw InputError("mjData allocation failed");
    std::vector<mjtNum> state_buffer(static_cast<size_t>(state_n));
    for (int i = 0; i < state_n; ++i)
      state_buffer[static_cast<size_t>(i)] = static_cast<mjtNum>(state[i]);
    mj_setState(model_.get(), initial_.get(), state_buffer.data(),
                mjSTATE_INTEGRATION);
    if (!std::isfinite(static_cast<double>(initial_->time)) ||
        !AllFinite(initial_->qpos, model_->nq) ||
        !AllFinite(initial_->qvel, model_->nv) ||
        !AllFinite(initial_->qacc, model_->nv)) {
      throw InputError("invalid initial dynamics state");
    }
  }
  int gsize() const noexcept { return gsize_; }
  // A Handle owns mutable MjData and is intentionally single-owner; callers
  // must not invoke Evaluate concurrently on the same handle.
  void Evaluate(const double* controls, double* cost, double* output) {
    RequireNoCallbacks();
    if (!controls || !cost || !output)
      throw InputError("null evaluation buffer");
    const int control_size = steps_ * kNu;
    if (!AllFinite(controls, control_size))
      throw InputError("nonfinite control");
    for (int i = 0; i < prefix_ * kNu; ++i) {
      if (controls[i] != baseline_[static_cast<size_t>(i)])
        throw InputError("fixed prefix altered");
    }
    if (!mj_copyData(work_.get(), model_.get(), initial_.get()))
      throw NumericalError("initial data copy failed");
    std::vector<double> values(static_cast<size_t>(gsize_));
    int offset = 0;
    double total_cost = 0.0;
    for (int step = 0; step < steps_; ++step) {
      const double* tau = controls + step * kNu;
      if (!mj_copyData(pre_.get(), model_.get(), work_.get()))
        throw NumericalError("pre-step data copy failed");
      for (int j = 0; j < kNu; ++j)
        pre_->ctrl[j] = static_cast<mjtNum>(tau[j]);
      mj_forward(model_.get(), pre_.get());
      CheckData(pre_.get());
      Observe(pre_.get(), values.data() + offset);
      offset += kObserveSize;
      for (int j = 0; j < kNu; ++j)
        work_->ctrl[j] = static_cast<mjtNum>(tau[j]);
      mj_step(model_.get(), work_.get());
      if (!mj_copyData(post_.get(), model_.get(), work_.get()))
        throw NumericalError("post-step data copy failed");
      mj_forward(model_.get(), post_.get());
      CheckData(post_.get());
      Observe(post_.get(), values.data() + offset);
      offset += kObserveSize;
      total_cost += StepCost(step, post_.get(), tau);
      const double expected_time = static_cast<double>(initial_->time) +
          (step + 1) * static_cast<double>(model_->opt.timestep);
      if (std::abs(static_cast<double>(work_->time) - expected_time) > 1e-10)
        throw NumericalError("absolute clock mismatch");
    }
    if (offset != gsize_) throw NumericalError("constraint size mismatch");
    const double computed_cost = .5 / static_cast<double>(steps_) * total_cost;
    if (!std::isfinite(computed_cost))
      throw NumericalError("nonfinite cost");
    for (double value : values) {
      if (!std::isfinite(value))
        throw NumericalError("nonfinite constraint");
    }
    *cost = computed_cost;
    for (int i = 0; i < gsize_; ++i)
      output[i] = values[static_cast<size_t>(i)];
  }
 private:
  void CheckData(const mjData* data) const {
    if (!data || !std::isfinite(static_cast<double>(data->time)) ||
        !AllFinite(data->qpos, model_->nq) ||
        !AllFinite(data->qvel, model_->nv) ||
        !AllFinite(data->qacc, model_->nv)) {
      throw NumericalError("nonfinite dynamics state");
    }
    const int bad[] = {mjWARN_BADQPOS, mjWARN_BADQVEL,
                       mjWARN_BADQACC, mjWARN_BADCTRL};
    for (int warning : bad) {
      if (data->warning[warning].number)
        throw NumericalError("MuJoCo numerical warning");
    }
  }
  void Contact(const mjData* data, std::array<double, 4>* forces,
               double* nonfoot) const {
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
          if (leg_count >= 4)
            throw NumericalError("contact foot index overflow");
          legs[leg_count++] = leg;
          (*forces)[leg] += static_cast<double>(force[0]);
        }
      }
      bool allowed = leg_count == 1;
      if (allowed) {
        const int foot = gids_[legs[0]];
        const int other = contact.geom1 == foot ? contact.geom2 : contact.geom1;
        if (other < 0 || other >= model_->ngeom)
          throw NumericalError("contact geometry index invalid");
        const int foot_body = model_->geom_bodyid[foot];
        const int other_body = model_->geom_bodyid[other];
        allowed = model_->body_rootid[other_body] !=
                  model_->body_rootid[foot_body];
        if (allowed && top_support_only_) {
          if (other_body!=0) throw InputError("unknown moving support");
          const double top=model_->geom_pos[3*other+2]+(model_->geom_type[other]==mjGEOM_BOX ? model_->geom_size[3*other+2] : 0.0);
          allowed=std::abs(contact.frame[2])>=1.0-1e-6 && data->geom_xpos[3*foot+2]>=top-1e-9;
        }
      }
      if (!allowed) {
        *nonfoot += std::sqrt(
            static_cast<double>(force[0]) * static_cast<double>(force[0]) +
            static_cast<double>(force[1]) * static_cast<double>(force[1]) +
            static_cast<double>(force[2]) * static_cast<double>(force[2]));
      }
    }
  }
  void Observe(const mjData* data, double* output) const {
    std::array<double, 4> forces{};
    double nonfoot = 0.0;
    Contact(data, &forces, &nonfoot);
    int offset = 0;
    for (double force : forces) output[offset++] = (180.0 - force) / 180.0;
    for (double force : forces) output[offset++] = force / 180.0;
    for (int i = 0; i < kNu; ++i)
      output[offset++] = (30.0 - std::abs(static_cast<double>(data->qvel[va_[i]]))) / 30.0;
    for (int i = 0; i < kNu; ++i) {
      const double q = static_cast<double>(data->qpos[qa_[i]]);
      const int joint = model_->actuator_trnid[2 * i];
      output[offset++] = q - static_cast<double>(model_->jnt_range[2 * joint]);
    }
    for (int i = 0; i < kNu; ++i) {
      const double q = static_cast<double>(data->qpos[qa_[i]]);
      const int joint = model_->actuator_trnid[2 * i];
      output[offset++] = static_cast<double>(model_->jnt_range[2 * joint + 1]) - q;
    }
    const double w = static_cast<double>(data->qpos[3]);
    const double x = static_cast<double>(data->qpos[4]);
    const double y = static_cast<double>(data->qpos[5]);
    const double z = static_cast<double>(data->qpos[6]);
    const double roll = std::atan2(2.0 * (w * x + y * z),
                                   1.0 - 2.0 * (x * x + y * y));
    const double pitch_arg = std::max(-1.0, std::min(1.0,
        2.0 * (w * y - z * x)));
    const double pitch = std::asin(pitch_arg);
    output[offset++] = static_cast<double>(data->qpos[2]) - .28;
    output[offset++] = kPi / 12.0 - std::abs(roll);
    output[offset++] = kPi / 12.0 - std::abs(pitch);
    output[offset++] = 1e-6 - nonfoot;
    if (offset != kObserveSize)
      throw NumericalError("observation size mismatch");
  }
  double StepCost(int step, const mjData* data, const double* tau) const {
    const double* body = body_refs_.data() + step * kBodyReferenceSize;
    const double* foot = foot_refs_.data() + step * kFootReferenceSize;
    double sum = 0.0;
    for (int i = 0; i < 3; ++i) {
      const double e = (static_cast<double>(data->qpos[i]) - body[i]) / .025;
      sum += e * e;
    }
    mjtNum actual_quat[4] = {
        data->qpos[3], data->qpos[4], data->qpos[5], data->qpos[6]};
    mjtNum reference_quat[4] = {
        static_cast<mjtNum>(body[3]), static_cast<mjtNum>(body[4]),
        static_cast<mjtNum>(body[5]), static_cast<mjtNum>(body[6])};
    mjtNum tangent[3]{};
    mju_subQuat(tangent, actual_quat, reference_quat);
    for (double value : tangent) {
      const double e = value / .10;
      sum += e * e;
    }
    for (int i = 0; i < 3; ++i) {
      const double e = (static_cast<double>(data->qvel[i]) - body[7 + i]) / .30;
      sum += e * e;
    }
    for (int i = 0; i < 3; ++i) {
      const double e = (static_cast<double>(data->qvel[3 + i]) - body[10 + i]) / .60;
      sum += e * e;
    }
    for (int leg = 0; leg < 4; ++leg) {
      const int geom = gids_[leg];
      for (int axis = 0; axis < 3; ++axis) {
        const double e = (static_cast<double>(data->geom_xpos[3 * geom + axis]) -
                          foot[3 * leg + axis]) / .025;
        sum += e * e;
      }
    }
    for (int i = 0; i < kNu; ++i) {
      const double e = (tau[i] - baseline_[static_cast<size_t>(step * kNu + i)]) / 35.0;
      sum += .01 * e * e;
    }
    if (!std::isfinite(sum)) throw NumericalError("nonfinite step cost");
    return sum;
  }
  ModelPtr model_;
  DataPtr initial_;
  DataPtr work_;
  DataPtr pre_;
  DataPtr post_;
  bool top_support_only_;
  int steps_;
  int prefix_;
  int state_size_;
  int gsize_;
  std::vector<double> baseline_;
  std::vector<double> body_refs_;
  std::vector<double> foot_refs_;
  std::array<int, kNu> va_;
  std::array<int, kNu> qa_;
  std::array<int, 4> gids_;
};
}  // namespace
extern "C" {
void* wm_create(const char* xml, const double* state, int state_n, int steps,
                int prefix, const double* baseline, const double* body_refs,
                const double* foot_refs, char* error, int cap) noexcept {
  try {
    return new Handle(xml, state, state_n, steps, prefix, baseline, body_refs,
                      foot_refs);
  } catch (const std::exception& exception) {
    WriteError(error, cap, exception.what());
  } catch (...) {
    WriteError(error, cap, "unknown native evaluator failure");
  }
  return nullptr;
}
void* wm_create_top_support(const char* xml, const double* state, int state_n, int steps,
                int prefix, const double* baseline, const double* body_refs,
                const double* foot_refs, char* error, int cap) noexcept {
  try {
    return new Handle(xml, state, state_n, steps, prefix, baseline, body_refs,
                      foot_refs, true);
  } catch (const std::exception& exception) {
    WriteError(error, cap, exception.what());
  } catch (...) {
    WriteError(error, cap, "unknown native evaluator failure");
  }
  return nullptr;
}
void wm_destroy(void* opaque) noexcept {
  try {
    delete static_cast<Handle*>(opaque);
  } catch (...) {
  }
}
int wm_gsize(void* opaque) noexcept {
  return opaque ? static_cast<Handle*>(opaque)->gsize() : 0;
}
int wm_eval(void* opaque, const double* controls, double* cost, double* g,
            int gsize, char* error, int cap) noexcept {
  if (!opaque || !controls || !cost || !g) {
    WriteError(error, cap, "null evaluator handle");
    return 1;
  }
  try {
    const int expected = static_cast<Handle*>(opaque)->gsize();
    if (gsize != expected) {
      WriteError(error, cap, "constraint size mismatch");
      return 1;
    }
    std::vector<double> values(static_cast<size_t>(expected));
    double value = 0.0;
    static_cast<Handle*>(opaque)->Evaluate(controls, &value, values.data());
    *cost = value;
    for (int i = 0; i < expected; ++i)
      g[i] = values[static_cast<size_t>(i)];
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
