"""Headless open-loop gait probe for separating leg-trajectory limits from WBC.

This is an experiment tool, not a release controller.  It uses the same Go2
MJCF and motor limits as the C++ simulator, but drives joint PD directly so a
candidate gait can be screened without DDS or the WBC/MPC stack.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import mujoco
import numpy as np


STAND = np.array(
    [
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
    ],
    dtype=float,
)
DOWN = np.array(
    [
        0.0473455, 1.22187, -2.44375,
        -0.0473455, 1.22187, -2.44375,
        0.0473455, 1.22187, -2.44375,
        -0.0473455, 1.22187, -2.44375,
    ],
    dtype=float,
)
OFFSETS = {
    "diagonal-trot": np.array([0.0, 0.5, 0.5, 0.0]),
    "bound": np.array([0.0, 0.0, 0.42, 0.42]),
}
GEOM = np.array(
    [
        [0.1934, -0.0465],
        [0.1934, 0.0465],
        [-0.1934, -0.0465],
        [-0.1934, 0.0465],
    ]
)
HIP_LINK = np.array([-0.0955, 0.0955, -0.0955, 0.0955])


def smoothstep(x: float) -> float:
    x = min(1.0, max(0.0, x))
    return x * x * (3.0 - 2.0 * x)


def foot_from_q(leg: int, q: np.ndarray) -> np.ndarray:
    hip, thigh, calf = q
    side = 1.0 if leg in (1, 3) else -1.0
    thigh_len = calf_len = 0.213
    leg_x = -thigh_len * math.sin(thigh) - calf_len * math.sin(thigh + calf)
    leg_z = -thigh_len * math.cos(thigh) - calf_len * math.cos(thigh + calf)
    y = math.cos(hip) * HIP_LINK[leg] - math.sin(hip) * leg_z
    z = math.sin(hip) * HIP_LINK[leg] + math.cos(hip) * leg_z
    return np.array([GEOM[leg, 0] + leg_x, GEOM[leg, 1] + y, z])


def ik(leg: int, foot: np.ndarray) -> np.ndarray:
    hip_x, hip_y = GEOM[leg]
    side = 1.0 if leg in (1, 3) else -1.0
    hlink = side * 0.0955
    x = foot[0] - hip_x
    y = foot[1] - hip_y
    z = foot[2]
    leg_z_sq = y * y + z * z - hlink * hlink
    if leg_z_sq < 0.0:
        raise ValueError("hip reach")
    leg_z = -math.sqrt(leg_z_sq)
    q_hip = math.atan2(z, y) - math.atan2(leg_z, hlink)
    planar_sq = x * x + leg_z * leg_z
    c = (planar_sq - 0.213**2 - 0.213**2) / (2.0 * 0.213 * 0.213)
    if c < -1.0 or c > 1.0:
        raise ValueError("leg reach")
    q_calf = -math.acos(max(-1.0, min(1.0, c)))
    q_thigh = math.atan2(-x, -leg_z) - math.atan2(
        0.213 * math.sin(q_calf), 0.213 + 0.213 * math.cos(q_calf)
    )
    return np.array([q_hip, q_thigh, q_calf])


def quat_rpy(q: np.ndarray) -> tuple[float, float]:
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return roll, math.asin(s)


def run(args: argparse.Namespace) -> Path:
    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    qadr = np.array(
        [model.jnt_qposadr[model.actuator_trnid[i, 0]] for i in range(12)],
        dtype=int,
    )
    dadr = np.array(
        [model.jnt_dofadr[model.actuator_trnid[i, 0]] for i in range(12)],
        dtype=int,
    )
    q0 = data.qpos[qadr].copy()
    feet0 = np.array([foot_from_q(i, STAND[3 * i : 3 * i + 3]) for i in range(4)])
    offsets = OFFSETS[args.pattern]
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s", "velocity_x_mps", "roll_rad", "pitch_rad", "base_z_m"])
        steps = int(args.duration / model.opt.timestep)
        for step_index in range(steps):
            t = step_index * model.opt.timestep
            if t < args.start_s:
                q_des = q0 + (STAND - q0) * smoothstep(t / 1.5)
            else:
                q_des = STAND.copy()
                phase = ((t - args.start_s) / args.period) % 1.0
                travel = args.step * args.duty
                for leg in range(4):
                    p = (phase + offsets[leg]) % 1.0
                    if p < args.duty:
                        xoff = 0.5 * travel - travel * smoothstep(p / args.duty)
                        zoff = 0.0
                    else:
                        s = (p - args.duty) / (1.0 - args.duty)
                        xoff = -0.5 * travel + travel * smoothstep(s)
                        zoff = args.lift * math.sin(math.pi * s) ** 2
                    target = feet0[leg] + np.array([xoff, 0.0, zoff])
                    # Positive --lean means nose-down body pitch.  A fixed
                    # body-frame foot slope supplies the same support moment
                    # that a sprint controller normally gets from its WBC
                    # angular task.
                    target[2] -= args.lean * target[0]
                    try:
                        q_des[3 * leg : 3 * leg + 3] = ik(leg, target)
                    except ValueError:
                        q_des[3 * leg : 3 * leg + 3] = q0[3 * leg : 3 * leg + 3]
            q = data.qpos[qadr]
            dq = data.qvel[dadr]
            torque = args.kp * (q_des - q) - args.kd * dq
            limits = np.array([40.0, 40.0, 45.43] * 4)
            data.ctrl[:] = np.clip(torque, -limits, limits)
            mujoco.mj_step(model, data)
            if step_index % 5 == 0:
                roll, pitch = quat_rpy(data.qpos[3:7])
                writer.writerow([t, data.qvel[0], roll, pitch, data.qpos[2]])
    return out


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--pattern", choices=tuple(OFFSETS), default="bound")
    parser.add_argument("--period", type=float, default=0.12)
    parser.add_argument("--duty", type=float, default=0.55)
    parser.add_argument("--step", type=float, default=0.32)
    parser.add_argument("--lift", type=float, default=0.14)
    parser.add_argument("--lean", type=float, default=0.0)
    parser.add_argument("--kp", type=float, default=100.0)
    parser.add_argument("--kd", type=float, default=3.5)
    parser.add_argument("--start-s", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()
    print(run(args))


if __name__ == "__main__":
    main()
