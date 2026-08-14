# Research history

This document summarizes milestone-level research progress. It intentionally excludes chat transcripts, agent handoffs, private planning notes, and chronological workspace diaries.

## 1. Low-level control and instrumentation

**Question.** Can the Unitree Go2 simulator/runtime stack support repeatable low-level action experiments with enough instrumentation to distinguish commanded motion from realized motion?

**Work.** The project established the `LowCmd → MuJoCo → LowState` loop, joint/IMU/contact logging, forward and inverse kinematics, world-frame foot-clearance measurements, and parameterized stand / weight-shift / leg-lift sequences.

**Result.** Repeatable stand-up, settle, trot, return-to-stand, and lie-down run as one LowCmd state machine. Smoothstep interpolation and a stand-pose settle are what stitch the segments. Reliable cruise is about 0.15 m/s; 0.18 m/s is a torque-gate edge; 0.21 m/s was rejected. This is the onboarding task (Go2 motion sequencing in MuJoCo), not a speed or natural-gait result.

**Evidence.** `example/cpp/` and the retained artifacts under `example/cpp/experiments/`.

## 2. Continuous trot and dynamics-informed control

**Question.** How far can a hand-designed model-based locomotion stack be pushed while retaining interpretable control structure and measurable failure modes?

**Work.** The controller was extended with diagonal-trot phase generation, smooth swing trajectories, Raibert landing adjustment, world/support feedback, constrained contact-force allocation, runtime gating, and incremental dynamics-informed feedforward.

**Result.** Under the recorded gates, 0.15 m/s is the reliable cruise, 0.18 m/s is marginal, and 0.21 m/s is out of range.

**Evidence.** `example/cpp/real_trot_go2.cpp`, related controller sources, and the retained experiment artifacts.

## 3. RL exploration

**Question.** Could learned locomotion provide a useful alternative to continued hand tuning for higher-speed and more dynamic behavior?

**Work.** The repository explored a small MuJoCo PPO implementation and later Isaac Lab / RSL-RL training. The experiments exposed evaluation and training-design pitfalls, including policies that could score well on coarse velocity metrics while producing undesirable or unstable motion.

**Result.** Isaac Lab velocity curricula reached commanded speeds up to ±3.5 m/s with a short-stride gait (`model_54950`). That is a useful speed result and not a natural-gait result. Configs are in `rl/isaaclab_custom/`; checkpoints stay in the archive.

**Evidence.** `rl/README.md`, `rl/isaaclab_custom/`.

## 4. Motion imitation moved to a companion repository

Kine2Go / Genesis imitation, the seam JSON record, and the conditional-AMP negative baseline are in [`kairoi-k/kine2go-research`](https://github.com/kairoi-k/kine2go-research).
