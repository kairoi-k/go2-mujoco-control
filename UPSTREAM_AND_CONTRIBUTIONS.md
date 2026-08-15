# Upstream and research contributions

This repository combines the Unitree MuJoCo simulator with research-specific C++ control and experiment code.

## Upstream components

| Component | Upstream | Role here |
|---|---|---|
| Unitree Go2 MuJoCo simulator | Unitree Robotics `unitree_mujoco` | simulator base |
| Unitree SDK2 / DDS | Unitree Robotics | runtime dependency |
| MuJoCo | DeepMind | physics; 3.3.6 in the research environment |
| Robot/URDF assets | Unitree | this checkout vendors Go2 only; H1/G1/B2/… remain upstream |

Isaac Lab is present here as a gym-registered velocity-curriculum package under `rl/` (`pip install -e rl`). Genesis / Kine2Go imitation is a companion repository.

Original Unitree READMEs: `docs/upstream/`. The repository root README is the research-project README (avoids case-only collisions with `readme.md`).

## Research contributions in this repository

- stand/walk/lie sequencing and 500 Hz low-level control experiments;
- diagonal-trot gait generation and Raibert planning;
- world/support feedback and simulation instrumentation;
- constrained contact-force/wrench allocation;
- incremental dynamics-informed WBC components with guarded fallback;
- Isaac Lab Go2 velocity-curriculum gym tasks (second track);
- controlled experiment runners and retained evidence.

Reliable cruise is about 0.15 m/s in this stack. This is not a claim of demo-speed locomotion or a complete full-dynamics WBC.

## Licensing and citation

Root `LICENSE` is BSD 3-Clause from Unitree, with the upstream copyright notice preserved. Cite upstream work separately from the research extensions.
