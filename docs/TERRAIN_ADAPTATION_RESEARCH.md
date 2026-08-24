# Terrain adaptation research and design decision

## Evidence reviewed

- Mastalli et al., *Motion Planning for Quadrupedal Locomotion: Coupled Planning, Terrain Mapping and Whole-Body Control*: footholds, CoM motion, step timing and trunk attitude are planned jointly; the controller tracks the plan while enforcing torque, kinematic and friction constraints.
- Focchi et al., *Robust Footstep Planning and LQR Control for Dynamic Quadrupedal Locomotion*: foothold planning is repeatedly updated from the current state and tracked with feedback, rather than executed as an open-loop foot jump.
- Sun et al., *Joint-Space CPG for Safe Foothold Planning and Body Pose Control during Locomotion and Climbing*: predicted footholds are iteratively pushed away from high-gradient/edge regions; body-pose correction is part of stair traversal.
- ETH/ANYmal perceptive locomotion work: an elevation map is searched around nominal footholds and the resulting sequence is executed with whole-body planning and collision/stability checks.
- `robot-locomotion/terrain-server`: terrain is represented as a risk/cost map using height deviation, slope and curvature, not height alone.

## Design decision for Go2

The current per-leg “target then interpolate” experiment is not an acceptance design. It can create an unreachable unilateral support transfer, alter lateral foothold placement, and leave the trunk reference flat while the support plane changes.

The replacement should be a short preview transaction: detect a step, stop/slow before commitment, select a paired front-foot landing patch with lateral and edge margins, generate the next front/rear foothold sequence, and ramp body height/pitch with bounded rates while keeping the projected CoM inside a shrunken support polygon. Every target must remain reserved until touchdown/contact and a stability dwell are verified. Only then may the next leg group be released.

Acceptance must include target reachability, lateral displacement, support-polygon margin, touchdown error, body attitude, contact fraction, torque/safety limits, full-duration completion, and repeated trials. A video without these logs is not evidence of successful terrain adaptation.

## Current experiment evidence (2026-08-24)

- The original target reservation accepted a foothold near the riser edge and
  then transferred one front leg while the trunk remained on the flat-ground
  reference. Repeated 10 cm runs reached roughly 20--30 degrees of roll or
  pitch and were rejected by the hard safety guard.
- The planner was using the FR inverse-kinematics branch for the FL target;
  this has been corrected. A full swing preview now checks IK and sampled
  height-map clearance, and execution uses the same terrain-aware clearance
  profile.
- The preview rejects the old bell-shaped swing solely on clearance in the
  physical barrier scene. A landing-patch test without swept-volume checking
  is therefore unsafe.
- The remaining failure is dynamic: a kinematically valid unilateral transfer
  is not dynamically stable for this WBC/contact schedule. The next accepted
  design must use a staged mount/body approach (edge foothold, body advance,
  then the next foothold), or reject the step and stop before contact. No 10 cm
  crossing is accepted yet.

## Source links

- https://arxiv.org/abs/2003.05481
- https://arxiv.org/abs/2010.12326
- https://www.marmotlab.org/publications/37-RAL2022-footholdCPG.pdf
- https://github.com/robot-locomotion/terrain-server
