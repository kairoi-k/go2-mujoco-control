# Exact-head sustained-running revalidation

Source commit: `66dc3e810dcf8766e4e2fd838e14fb772805c76d` (`gait/sustained-sprint-running-2026-08-21`). The isolated tracked worktree was clean. The MuJoCo simulator and `example/cpp` built successfully; `ctest --test-dir example/cpp/build --output-on-failure` passed 25/25 tests. `simulate/build` registered no tests.

Commands, with only run name and DDS domain varied:

```bash
SUSTAINED_SPRINT_NAME=codex_reval_3mps_r1 SUSTAINED_SPRINT_DOMAIN_ID=191 bash example/cpp/scripts/run_sustained_running.sh --headless
SUSTAINED_SPRINT_NAME=codex_reval_3mps_r2 SUSTAINED_SPRINT_DOMAIN_ID=192 bash example/cpp/scripts/run_sustained_running.sh --headless
SUSTAINED_SPRINT_NAME=codex_reval_3mps_r3 SUSTAINED_SPRINT_DOMAIN_ID=193 bash example/cpp/scripts/run_sustained_running.sh --headless
python3 example/cpp/tools/analysis/analyze_sustained_running.py example/cpp/experiments/_runs/<run-name>
```

Held-fixed semantics: `--wbc-full --gait-pattern running-trot --tau-limit 45 --period 0.14 --duty 0.44 --step-length 0.50 --foot-lift 0.20 --kernel raibert-trot --raibert-velocity-gain 0.010 --raibert-max-adjustment 0.06 --preview-horizon 4 --support-anchor-feedback --support-anchor-gain 0.35 --controller-duration 40 --headless --wall-clock-motion`; semantic environment snapshot is retained in each run's ignored `environment.txt`.

| run | analyzer | median m/s | accepted window s | roll/pitch P95 deg | aerial | pair-sync min | stop-tail P95 m/s | final m/s |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `codex_reval_3mps_r1` | PASS | 3.235254 | 61.352 | 2.818/2.325 | 0.290171 | 0.812456 | 0.003860 | 0.001954 |
| `codex_reval_3mps_r2` | PASS | 3.225791 | 61.342 | 2.971/2.500 | 0.284504 | 0.814020 | 0.003713 | 0.001901 |
| `codex_reval_3mps_r3` | PASS | 3.225737 | 61.630 | 3.081/2.562 | 0.289935 | 0.799793 | 0.003905 | 0.002013 |

Common binary hashes: simulator `dd6fcb1332f81872e3d47e6fabbb8e191caa263b70f90832d5246a477de4bd5e`; controller `a63ed9b908fd81979f5efab1a1c464658d32b4073b8b894dc87bc244ab017d81`; scene `12286418247d0e240ae131b5ae5c60f3a7a481d4754aefe4517476e937aa05b8`.

All lifecycle, safety, quality, analysis, ground-truth, dynamics, and completion statuses were zero. This is a simulation-only `running-trot` result; it does not claim natural-animal gait, hardware performance, or sim-to-real transfer. Raw `_runs/` and per-run artifact hashes remain in the local compact provenance record at `/tmp/go2_highspeed_revalidation_2026-08-24.md` and are not committed.
