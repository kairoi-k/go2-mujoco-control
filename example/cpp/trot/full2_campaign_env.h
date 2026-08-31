#pragma once

#include <cstdlib>
#include <cstring>
#include <string>

// One-factor full2 campaign knobs. Unset = 8.15 cartesian-world baseline.
inline double Full2EnvDouble(const char *name, double fallback)
{
    const char *s = std::getenv(name);
    if (s == nullptr || s[0] == '\0')
        return fallback;
    char *end = nullptr;
    const double v = std::strtod(s, &end);
    if (end == s)
        return fallback;
    return v;
}

inline bool Full2UseCheetahTable()
{
    const char *s = std::getenv("FULL2_TABLE");
    return s != nullptr && std::strcmp(s, "cheetah") == 0;
}

inline bool Full2UseRunGait()
{
    return Full2EnvDouble("FULL2_RUN", 0.0) > 0.5;
}

inline bool Full2UseSlew()
{
    return Full2EnvDouble("FULL2_SLEW", 0.0) > 0.5;
}

// Order-088 diagnostic channel. Unset/zero keeps the normal CSV values
// suppressed; this flag never changes control decisions.
inline bool Full2TerrainTelemetryEnabled()
{
    return Full2EnvDouble("TROT_TERRAIN_TELEMETRY", 0.0) > 0.5;
}

// FULL2_INC: override cycle speed increment (baseline 0.03, or 0.06 if v>=0.40).
// FULL2_YPOS: lateral world-Y capture gain (baseline 0). Closed: 0.35 on T1
//   shortened survival; T1 Y was heading curve, not crab.
// FULL2_RAIBERT_X / FULL2_RAIBERT_Y: split sagittal vs lateral gain.
// G2 uses 0.12 isotropic and sits; T1 uses 0.03 isotropic, climbs, yaws.
// FULL2_AX_LIM / FULL2_AX_GAIN: cartesian CoM ax (baseline 1.0 / 2.0).
// FULL2_STUMBLE: hold cmd/gait across a one-cycle v_meas dip >0.08 m/s.
// FULL2_RUN: CheetahTrotEarly morph from 0.15. Pin-at-2.0 (RN) sat at 0.016.
// FULL2_NO_LOCK: cart_lock=0. G2 lock grows with speed and raises no-slip.
// FULL2_W_LIN: ID-WBC w_base_lin (baseline 80).
// FULL2_W_SWING: ID-WBC w_swing (baseline 80). Keep blows up at 1.1 with
//   foot_error 0.068 / tau sat; one-factor heavier swing tracking.
// FULL2_SWING_KP / FULL2_SWING_KD / FULL2_SWING_ACC: cartesian swing PD
//   (baseline 180 / 16 / 50).
// FULL2_SCHED_LEAD: v_sched = v_meas+this (baseline 0.12). Cmd lead is 0.20.
// FULL2_FOOT_HOLD: hold cmd if previous-cycle foot_error exceeds this.
//   Keep c40 foot=0.033 then c41 roll 15° / foot 0.068.
// FULL2_FOOT_V: only apply FOOT_HOLD once v_meas >= this (FH030 sat 2/3).
// FULL2_AX_FOOT: zero extra CoM ax if last-cycle foot_error exceeds this.
// FULL2_AX_FOOT_V: only apply AX_FOOT once body vx >= this.
// FULL2_W_ANG: ID-WBC w_base_ang (baseline 80 with NO_LOCK).
// FULL2_NO_BLEND: force_blend=0. FULL2_PITCH: pitch acc gain (baseline 12).
// FULL2_LIFT: swing foot lift m (cheetah table 0.040–0.052).
// FULL2_ROLL: roll acc gain (baseline 40).
// FULL2_DUTY_FLOOR: min Cheetah duty. Lead 0.25 hit 0.97 then roll 22°
//   at duty 0.40 / contact fraction 0.63.
// FULL2_DUTY_V: only apply DUTY_FLOOR once v_sched >= this (morph-safe).
// FULL2_EARLY_SPAN: CheetahTrotEarly lerp denominator. Unset = 0.85
//   (t=1 at 1.00 m/s). Keep saturates run-duty by cycle 29.
// FULL2_FAST_SPAN: after FAST_FROM, lerp toward duty 0.36 / period 0.22.
// FULL2_FAST_FROM: start of that lerp (unset=1.00). FROM=0.85 covers the
//   keep 0.85–1.04 crest where SPAN-from-1.00 never fired.
// FULL2_TAU: ID-WBC tau_limit_nm (baseline 35). Keep tau_est hits 40
//   whenever v_meas tries to crest ~1.04.
// FULL2_PERIOD_FLOOR: min gait period_s after schedule.
// FULL2_ATT: attitude_ok deg (baseline 5). Slam at att+3 pitch / att+2 roll.
// FULL2_ATT_V: only apply ATT once v_meas >= this. ATT=10 from t=0 sat.

