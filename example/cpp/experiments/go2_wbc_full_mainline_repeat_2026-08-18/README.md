# `--wbc-full` 主线重复测量（64 圈 + stand-walk-lie）

- 日期：2026-08-18 / 2026-08-19
- 状态：indexed（`docs/RESEARCH_INDEX.md` 当前巡航数字。2026-08-15 单次 0.149 m/s 仍留在 `docs/WBC_MPC.md` 作历史）
- 代码：`2b82dae`（`Refactor example/cpp into modules and extract TrotTask`）
- 目的：主线把 `go2sim task` 改成 `--wbc-full --tau-limit 35` 之后，用同一套入口做可审计的 n 次重复，记下速度散布，而不是只留 gitignored `_runs/`。

## 问题

同一 git、同一 `go2sim full` / `task` 标志，headless 重复能否走完，速度是多少、抖多少？

## 采用配置

入口：`example/cpp/scripts/go2sim`。测速：`analyze_locomotion_progress.py` 同类 OLS，`motion_stage==2`，目标 `0.091/0.60 = 0.151667` m/s。

```text
--period 0.60 --duty 0.75 --step-length 0.091 --foot-lift 0.020
--kp 63 --kd 2.8 --kernel raibert-trot
--raibert-velocity-gain 0.05 --raibert-max-adjustment 0.010
--world-feedback-max 0.060 --world-feedback-slew 0.004
--wbc-full --tau-limit 35 --headless
DDS domain 220（不用 232）
TROT_CPU_AFFINITY_SIM=0 TROT_CPU_AFFINITY_CTRL=1
```

- `full`：`--max-cycles 64`
- `task`：`--task stand-walk-lie`；本批走路窗约 8 s（runner 另传了 `--controller-duration 8`）

哈希见 `hashes.txt`。仿真二进制是当晚 23:49 仓库内重编的 `simulate/build/unitree_mujoco`（sha256 `a2ccdbaa…`）；源码与 `/home/che/dev/unitree_mujoco` 树一致，`libmujoco.so.3.3.6` 也一致。

## 结果

`full` n=5，5/5 走完并回到站立。OLS 速度 **0.130 ± 0.011 m/s**（0.116–0.147）。侧向 20–29 mm。站高 z≈0.370 m。|pitch|≈3.9°，|roll|≈1.0–1.5°。`q_error` 门限 0.80；除 `full_235529` 到 0.223 外，其余约 0.168。

`task` n=3，3/3 完成 stand-walk-lie。走路 8 s OLS **0.139 ± 0.004 m/s**。

| run | 完成 | OLS m/s | ratio | 侧向 m | 走路时长 s | 路程 m |
|---|---|---|---|---|---|---|
| full_235346 | 是 | 0.136 | 0.895 | 0.026 | 38.41 | 5.23 |
| full_235437 | 是 | 0.129 | 0.851 | 0.028 | 38.42 | 4.94 |
| full_235529 | 是 | 0.116 | 0.768 | 0.023 | 40.79 | 4.83 |
| full_235624 | 是 | 0.120 | 0.794 | 0.020 | 38.53 | 4.74 |
| full_235912 | 是 | 0.147 | 0.970 | 0.029 | 38.40 | 5.60 |
| task_000014 | 是 | 0.144 | 0.949 | 0.014 | 8.00 | 1.13 |
| task_000038 | 是 | 0.136 | 0.894 | 0.012 | 8.00 | 1.07 |
| task_000150 | 是 | 0.137 | 0.904 | 0.015 | 8.01 | 1.08 |

墙钟每趟 `full` 约 48–51 s，和仿真时长同量级。慢的是地面距离，不是仿真放慢。`full_235529` 走路窗被拉到 40.8 s，最大 tick 间隙 38 ms；同一批里速度几乎跟着 motion-clock pause 走（最快 0.147 / pause 1300，最慢 0.116 / pause 5795）。

同一晚更早一趟（约 20:40，旧进程、旧仿真二进制 `c5430fb…`）`full` n=5 均值约 **0.142 ± 0.009**，`task` 约 **0.140**。那批 CSV 不在本目录。本记录只把有哈希的 23:53 批当作证据。

## 解释边界

- 证明：`2b82dae` 上 `--wbc-full` 能重复走完 64 圈和 stand-walk-lie；短序列约 0.14 m/s；长序列常见 0.12–0.15 m/s，受时钟暂停/打滑影响。
- 不证明：原主页 `--wbc-primary` 站立小跑仍可复现；A→B；2 m/s；`go2sim walk` 现在能过门。
- 不覆盖：2026-08-15 写入 `docs/WBC_MPC.md` 的单次 **0.149 m/s / ratio 0.985**。那是另一次会话的单点，不是这 n=5。
- 主页片段改为 `docs/media/stand_walk_lie_wbcfull.gif` / `.mp4`。旧 `stand_walk_lie.gif` 仍是 `--wbc-primary` 历史片段，没有覆盖。

## 证据

- 代码：`2b82dae`
- 表：`results.tsv`
- 每趟：`trials/<name>/run_metadata.txt`、`speed_audit.txt`（`task_000150` 的 audit 由 `last.json` 补写）
- 原始 `data.csv` / contact CSV（每趟约 50–130 MB）留在 gitignored `example/cpp/experiments/_runs/`，不进 git。`trials/` 这个名字是为了躲开根 `.gitignore` 的 `**/runs/`。
