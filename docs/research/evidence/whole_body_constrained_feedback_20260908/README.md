# Constrained full-body initialized feedback
Clean runtime714bf3; same model,2ms physics,20periods each; protocol WHOLE_BODY_CONSTRAINED_FEEDBACK_V1. Nominal,vy0.05m/s androll0.02rad all finish1400steps with19/19fully covered running-contact cycles, exact independent state/force/control plant replay, and all frozen physical/terminal checks passing. Each advances2.383745m. Original V1 remains FAIL for absolute dynamics residual and accumulated floating-clock tolerance; no numerical thresholds changed.

0001 nominal: no nonlinear correction required; post force171.989N.
0002 vy: one strict feasible evaluated correction atstep7, postforce179.999994498747N, preforce maximum171.517N overrun. Full feedback latency median1847.69us,p952204.01us,max133427.85us. Corrective solve exceeds2ms; not realtime ready.
0003 roll: no nonlinear correction, postforce173.229N; median1828.07us,p952155.49us,max5507.07us.

These are initialized privileged flat model replays with a fixed prior-cycle-seeded nominal. No new controller runtime, sensor/observed-terrain model construction, atomic new trajectory payload,5cm B1 or10cm result exists. One-step constraints have no horizon viability proof. Exact original JSON and full contact diagnostics preserved losslessly in gzip; portable JSON retains unchanged numerical arrays and bound source snapshots.

Independent all4200step pre/post force audit reproduces recorded metadata exactly and finds no strict180N violation. A subsequent matched fixed-state5pair scratch-restoration benchmark gives old/new solve median134.075/18.108ms and fastpath1.752/.730ms, identical controls/forces/call counts. This WIP-source benchmark is not full new-source runtime or2ms deadline acceptance.
