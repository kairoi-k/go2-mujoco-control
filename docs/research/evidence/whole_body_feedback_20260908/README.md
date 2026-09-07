# Actual full-body periodic feedback diagnostics
Source6a996ce28cfb5efd8201c040f8ca8d40d6349c4b, clean at every run. Same full MuJoCo3.3.6, original2ms timestep, checked local derivatives, feedback effort weight0.01,35Nm clip. All begin with the recorded0012 source state; no live controller or terrain input integration. Protocol WHOLE_BODY_FEEDBACK_DIAGNOSTIC_V1.md.

0001:5periods/0.70s; all four fully covered phase-zero cycles contain both diagonals>=10ms and GRF<10N>=4ms. Force171.989N; all physical/terminal checks pass, original V1 dynamics3.71828e-7 fails. Independent saved-control replay exact. Independent algebraic reconstruction confirms all350commands equal clip(u-Ke) exactly.
0002:20periods/2.80s,19/19complete running-contact cycles; force171.989N,torque28.3254Nm, no saturation, terminalbody0.879mm. V1 fails dynamics and accumulated floating-clock difference1.55209e-12>1e-12; retained, no numerical threshold change.
0003:20periods,+initialvy0.05m/s;19/19complete running-contact cycles and later converges, but early normal force188.714892N at21.036 exceeds180N. V1 physical FAIL. Planned roll0.02 perturbation stopped and NOT RUN. Next locate changed-state vs feedback-torque cause before constrained feedback research.

Per-attempt result.json contains exact numerical arrays with portable paths; raw_result.json.gz losslessly preserves original JSON. Runtime Python sources copied and bound. Full contact diagnostics are losslessly compressed with portable summary and raw hash. No B1 or10cm claim.
