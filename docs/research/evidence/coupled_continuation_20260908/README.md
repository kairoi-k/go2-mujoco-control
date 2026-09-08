# Matched continuation, 2026-09-08
Genuine5cm B1 NOT_CERTIFIED. This privileged initialized flat diagnostic changes
one specific next decision: terminal angular-rate saturation in coupled0002 does
NOT cause failure under the existing periodic feedback in this tested state.
Both baseline and coupled16-step heads are continued140steps, two full140ms
periods, with the SAME absolute-phase tau-K*error feedback (35Nm clip). No native
one-step force projection, reset, retiming, or altered simulator settings.
Clean producer source e21bdd0; independent verifier source b25edda (exact commit
objects retained in history). All11 checks pass for each trajectory. Full state,
force and head/feedback command-law residuals0; clock residual1.72917e-13s.
Baseline/coupled peakforce180/174.249136N, max torque28.355897Nm both, minimum
baseheight0.367534/0.368058m, max absolute roll/pitch0.005894/0.008580rad.
Final vy -0.009293/-0.004922m/s, final angular rates below0.032rad/s.
The coupled action is recoverable by THIS policy at THIS initialization. It does
not establish a recovery region, global viability, whole gait quality, observed
terrain command production, real-time scheduling, or B1. The17.3s solver cost is
still unchanged; speed and nonprivileged event-spanning terrain planning remain.
Do not spend the next experiment fixing an unobserved terminal-instability failure.
Reproduce from the source commits, native WSL MuJoCo3.3.6, OPENBLAS_NUM_THREADS=1,
OMP_NUM_THREADS=1. Scripts self-lock /tmp/go2_mujoco_experiment.lock. Use
example/cpp/tools/research/audit_coupled_continuation.py with coupled0002/result.json,
--packet pointing at the original hash-bound trajectory_packet_0002, --out NEW.
Then verify_coupled_continuation.py NEW --head coupled0002/result.json --out OTHER.
Durable packet lives in ../whole_body_native_20260908/trajectory_packet_0002;
its original manifests retain absolute source paths. For a relocated checkout,
rebuild a new packet from the bound source artifacts and report new provenance;
do not rewrite the historic manifests. Raw _runs remain untouched. The gzip here
is a lossless copy, with original uncompressed hash in MANIFEST.json.
