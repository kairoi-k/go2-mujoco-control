# Conditional terminal swing coverage audit
Source b49e9e879dc732efa2de010e5b4011accaea4efb. Original runtime input is
attempt_0006/articulated_source.txt (STATE21.018, runtime0a853346).
No mj_step, controller activation, initial-state projection or threshold change.
The original --articulated-audit JSON remains semantically identical after
extracting the shared production candidate-reference calculation.
The new CLI --articulated-tail-audit PREDICTION_END_S CHOICES_CSV uses the same
actual model, phase clock semantics and immutable terrain/history. It generates
next events from the fixed schedule and queries existing terrain candidates.
The explicit21.5 prediction end is a NEW stationary-terrain prediction
assumption for this diagnostic, not measured future terrain or an extension
of old surfaces. The original core problem, forces and combination stay fixed.
Binder accepts explicit candidate choices and copies their original observation
provenance. It does not search, optimize or certify them as executable.
All25 combinations of the two five-candidate terminal sets cover the complete
21.018--21.298 horizon, then fail initial_condition_conflict in body
reconstruction, with0 trajectory samples. This is exhaustive only over those
terminal sets and this fixed core combination. It does not prove the task
physically impossible. The retained initial-motion audit independently shows
moving measured support feet; the current nominal stationary support reference
cannot reproduce that actual initial q/dq. A coherent initial contact transition
or integrated articulated formulation must resolve this without resetting state.
Prediction end21.35 fails coverage_incomplete because the full terminal contact
interval is not covered. Candidate index5 fails invalid_input. Four focused
CTest targets pass, including new binding tests for atomic failure, explicit
selection, conflicting targets, map epochs and unavailable terrain.
Reproduce the selected case using the exact command in selected_audit.json.
Enumeration command varies CHOICES_CSV over0..4 for each of two entries.
No new flat or B1 closed-loop evidence; neither5cm nor10cm is accepted.
