# Phase1 baseline reproduction gate and delayed PD pulse A/B

## Outcome

Result: INCONCLUSIVE for PD causality.

The A baseline gate passed on the current source and binary. The single authorized B run entered the pulse, suppressed only LowCmd kp/kd in the declared interval, then suffered a genuine hard-posture safety stop before a clean recovery interval. No B retry was made and no sole-root-cause claim is made.

The previous failed checkpoint remains intact in history and in docs/validation/phase1_pd_pulse_ab_20260913/.

## Scope and source plumbing

Target branch: research/phase1-pd-pulse-baseline-gate-20260913.
Run source git_head: 318f592e24d1600d1a2350073e8d4e2df9e4feab, git_dirty=true.
Contract source base: 1fc68a34233551ac9ed1e57f68cc86357fd560ad.

Only the experiment plumbing changed after HEAD: pd_pulse_enabled_ is initialized once in TrotExperiment::Init from TROT_PD_PULSE_AB; WriteMotorCommands reads that cached boolean and the active-relative half-open window [32.10,32.40). No controller mathematics, gains, timing, gait, WBC/SRBD, contact logic, model, scene, profile, or thresholds changed.

Modified source SHA256:
- example/cpp/trot/trot_experiment.h: 77106d4ad85b901b7e6dfb6cfdaf21319cf9c8da3e82667c77b86bfefb2e08ef
- example/cpp/trot/trot_experiment_lifecycle.cpp: 74a7ad1a3c88a533774187fd22d31b18f3ff20528e38e53324f93623ca7dfa85
- example/cpp/trot/trot_experiment_control.cpp: b13e9c2f3c93dbc1b4259bbc0914280cbcfbfbd5fc5a94dd1dc7d5db4989bf66

Both runs used the same controller binary 42730bf8a5076e5fd52aab7bf0dee8ee4f9fed35f356bfeb0caac6792758a255 and simulator binary 1b9a3ff25a9839ac788401666c656a89d96592a89f70dc2fdb65f7a8be954994. Scene SHA256: 12286418247d0e240ae131b5ae5c60f3a7a481d4754aefe4517476e937aa05b8. Profile SHA256: 9efcc3b2d89fb349a12990ace1cf6ceb45e0d731deb470bdf2af084d82449d74.

## Invocation and environment

A run: varying_20260913_204709, DDS domain 232, no explicit TROT_SEED/RUN_SEED, TROT_PD_PULSE_AB unset.
B run: varying_20260913_205159, DDS domain 232, no explicit TROT_SEED/RUN_SEED, TROT_PD_PULSE_AB=1.

The shared controller argv was:
--headless --wall-clock-motion --controller-duration 86 --wbc-full --gait-pattern running-trot --kernel raibert-trot --period 0.14 --duty 0.44 --step-length 0.50 --foot-lift 0.20 --tau-limit 45 --raibert-velocity-gain 0.010 --raibert-max-adjustment 0.06 --preview-horizon 4 --support-anchor-feedback --support-anchor-gain 0.35 --velocity-max-accel 0.80 --velocity-max-decel 1.20 --velocity-max-jerk 4.0 --velocity-command-script example/cpp/configs/phase1_velocity_varying.csv --velocity-max-tracking-lead 0.20 --domain-id 232

The complete effective semantic environment is retained in each run environment.txt. It was identical except for TROT_PD_PULSE_AB=1 in B: TROT_CPU_AUTOPIN=1, TROT_DIAG_ID_CLOSURE=1, TROT_DYNAMICS_TOLERANCE_N=20, TROT_HS_ACC_GAIN=10, TROT_HS_ACC_LIMIT=4, TROT_HS_HYBRID_CONTACT=2, TROT_HS_PITCH_DAMP=6, TROT_HS_PITCH_GAIN=24, TROT_HS_ROLL_DAMP=10, TROT_HS_ROLL_GAIN=20, TROT_HS_SPEED_LEAD=0.25, TROT_HS_STABILITY_GOV=1, TROT_HS_START_DUTY=0.50, TROT_HS_START_PERIOD=0.20, TROT_HS_STEP_CAP=0.52, TROT_HS_SWING_REACH=0.90.

## A baseline gate

A reached active-relative 80.000079 s and closure capture 31.900078–33.090062 s with 120 closure rows. controller_status=0, safety_status=0, quality_status=0, analysis_status=0, completion_status=0. Overall roll/pitch absolute maxima were 3.352/9.268 degrees; no hard-posture failure occurred.

For A [32,33), requested/shaped velocity was 2.300000 m/s; applied/kernel nominal median was 2.276882 m/s; measured median was 2.523118 m/s; measured-minus-applied median was +0.246237 m/s. WBC desired ax median was -2.462367 m/s2, SRBD ax median -1.668977 m/s2, and ID qdd[0] median -2.149989 m/s2. The 100 ms past local-fit realized ax median was +0.091385 m/s2, p05/p95 -0.518843/+0.509504 m/s2.

Against the known good 190838 reference, the current [32,33) medians differed by measured -0.000769 m/s, applied +0.000769 m/s, WBC ax +0.015373 m/s2, SRBD ax -0.037394 m/s2, ID qdd[0] -0.008947 m/s2, and realized ax +0.070788 m/s2. The qualitative residual-overspeed/braking mismatch reproduced.

## B pulse evidence

B reached active-relative 32.427994 s and produced 53 closure rows from 31.901996 to 32.421992 s. In the closure capture, all sampled B pulse rows had motor kp=kd=0 from 32.101996 through 32.391992 s; normal nonzero kp/kd resumed at 32.401990 s. This verifies the declared suppression and restoration boundaries.

B first exceeded 15 degrees in the raw state at active 32.361994 s, with absolute roll 9.880 degrees and pitch 15.293 degrees. The first hard-posture log entry reports roll -16.9085 degrees and pitch 22.1582 degrees, followed by hard safety stop; B metadata has safety_status=1 and completion_status=1.

Before the pulse, A/B used the same source/binary/argv and both had normal kp/kd; no seed was set, so bitwise replay is not claimed. The pre-pulse measured-minus-applied medians were A +0.208360 and B +0.231991 m/s. At 32.20/32.30/32.40 s, v(t)-v(32.10) was A -0.035117/+0.005666/-0.002953 m/s and B -0.201096/-0.449201/-0.347335 m/s. This B velocity decrease coincided with posture/contact degradation, not an isolated stable actuator effect.

In early pulse [32.10,32.20), realized ax medians were A -0.318797 and B -0.088403 m/s2, while WBC/SRBD/ID qdd medians were A -1.969745/-1.473412/-2.049899 and B -2.002143/+0.070704/-1.998804 m/s2. In late pulse [32.20,32.40), B realized ax was -2.334745 m/s2, but B WBC/SRBD/ID qdd became +1.872134/+1.350685/+0.398334 m/s2 and solver/physical contact masks mismatched on 18/20 closure rows. The late result is therefore confounded by the posture/contact topology change. Full four-window values, including ID tau, LowCmd tau_ff, effective command, tau_est, kp/kd, masks, and posture, are in ab.csv.

## Interpretation

The pulse does not support the claim that simulator PD causes the residual overspeed. It shows that removing PD in this closed-loop gait leads to rapid contact/model-side divergence and a genuine hard-posture stop; the observed velocity fall is not a clean causal braking comparison. PD may be required for this gait's stability, while PD/ID coordination remains unresolved. Confidence: medium for the instability observation, low for any isolated PD body-axis causal attribution.

## Raw evidence hashes

A run_id=varying_20260913_204709:
- data.csv: b96494bdb69f6f4376319dbe583c3de12901feb3090a461ca0af7ffca51e984e
- data.csv.id_closure.csv: f5ef3c738276150db6d4be08b8b99b63f8342bba2c61173b52941a3a347cd693
- controller.log: 2bb1a5980cc47ee94646ab0456c33ede842a8efa3017a692838a62a17b40247f
- simulator.log: cc9843b4bfcb0d4df24fb2395a125039fbefd1c99bdcbf32e76a2359ca66bd86
- run_metadata.txt: dd673f03b357774a26f11c0a974fe2a644054e78e6c9f04c4c79f29a200aec53
- run_manifest.json: 18e53e9edef67ea8339c7bc53ffd3d0b2c53d8f4abf3d89fcb12ac3c99190c68
- environment.txt: f83fc4738dcb486d7e61b6c267246bd80210652ca287a504490ee5b3f01bb57c
- contact_ground_truth.csv: c75f6e0202b7d35e7d17e6135975bae2e8e0d468f3b38ea7ec14a52e02b60d48

B run_id=varying_20260913_205159:
- data.csv: 7a1183a0e096bf3455695a38f5b94ebfc9a3c82154f78c97039267d2c9bddc4c
- data.csv.id_closure.csv: e18c91e7b9952889c43fe6f5cdfee152e6939200eae67ed9c2798b95a275f35d
- controller.log: 101b16d65cc9bea2b7c4684f93225ee6c492596ce7bdde56936238dd9d9730ea
- simulator.log: 04cce7303108cd5c96245c72dbf1d434ff8d3e658a0abf543be7115f7a27b4d3
- run_metadata.txt: a45bd579f61c4323af79da7e186c146f335329bfc155c97e37c6767de18099eb
- run_manifest.json: a22a504e74c6fee8ebcf44d999e3fad637b8b96639dfb8be5cc9c2d27ce65ec0
- environment.txt: 4f9b123923952c167ba2a5c071e276c0bd331e5c54818be325e26ac96c6bcefa
- contact_ground_truth.csv: 91d5a47d64f43af30f9a4a5360eacc5ee446b66aba5d25ff45afadf4b6469360

## Next step

One next step only, not executed: design and approve a safer actuator-composition isolation that cannot immediately collapse this gait, then repeat the same one-variable causal window. No gain tuning, WBC/SRBD/contact changes, terrain/Phase2 work, or further runs were performed.
