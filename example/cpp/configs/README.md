# 序列配置

供 `run_leg_sequence.sh` / `real_leg_lift_go2 --sequence-file` 使用。

典型配置：

- `go2_four_step_fr_rl_fl_rr.txt` — 对角四步

格式见加载函数 `LoadStepSequence`（`leg_lift_types.h`）：每步指定抬腿、重心偏移、抬脚高、摆动与机身前进等。

跑法示例：

```bash
bash example/cpp/scripts/run_leg_sequence.sh 60 \
  go2_four_step_diagonal_rrx10_2026-08-02 \
  example/cpp/configs/go2_four_step_fr_rl_fl_rr.txt
```

## Versioned simulation profiles

JSON profiles in `profiles/` are schema-versioned and keep ordered controller
arguments beside the semantic environment snapshot used by a run. The
canonical `go2sim` launcher accepts either `--profile FILE` or the
`GO2_PROFILE_PATH` environment variable:

```bash
bash example/cpp/scripts/go2sim \
  --profile example/cpp/configs/profiles/go2sim_full.json --headless
```

Resolution is deterministic: compiled launcher defaults, then profile values,
then explicit command-line arguments. Existing scene wrappers remain valid;
the profile only replaces the preset argument set when selected. Each run
also writes `run_manifest.json` next to the legacy `run_metadata.txt`, with
the effective arguments, profile hash, repository state, semantic environment,
artifact hashes, analyzer identities, and lifecycle statuses.
