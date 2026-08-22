# 控制脚本

## 维护入口

| 脚本 | 用途 |
|---|---|
| `go2sim` | 首选一键行走、快档、完整任务、扰动与负载入口 |
| `run_trot.sh` | trot 底层启动（`go2sim` 调用） |
| `run_leg_sequence.sh` | 多步抬腿 / 换腿序列 |
| `run_leg_lift_repeats.sh` | 抬腿重复实验 |
| `run_periodic_leg_lift.sh` | 周期抬腿 |
| `run_single_step.sh` / `run_two_step.sh` | 单步、两步实验 |
| `run_weight_shift_scan.sh` | 重心扫描 |
| `record_periodic_leg_lift.sh` | 周期抬腿录制（需显式配置捕获工具目录） |
| `run_natural_trot.sh` | 固定参数、直线约束的自然小跑基线入口 |
| `run_running_trot.sh` | 固定参数、直线约束的低占空比高速跑态入口 |
| `run_sustained_running.sh` | 墙钟相位、3 m/s 级持续 running-trot 入口 |

自动环境感知验收：`../tools/analyze_auto_environment.py` 会检查高度图新鲜度、自动事件、参考方向、姿态、WBC 残差和真实障碍物接触。
物理冲击→急停验收：`../tools/analyze_auto_impact.py`；它将 simulator.log 的施力时刻与 data.csv 的 state_tick_s 对齐并输出严格报告。
```bash
bash example/cpp/scripts/go2sim walk --view
bash example/cpp/scripts/go2sim walk
bash example/cpp/scripts/go2sim fast --view
```

实验输出：名称 `go2_*` → `experiments/`；其它临时输出 → `experiments/_runs/`。DDS domain 使用 203–207。

历史批量扫参和机器专属启动器不在 git。

自然小跑验收：

```bash
bash example/cpp/scripts/run_natural_trot.sh 100 natural_trot_rep1
python3 example/cpp/tools/analysis/analyze_natural_gait.py \
  example/cpp/experiments/_runs/natural_trot_rep1 \
  --min-cruise-speed 0.80 --min-speed-p05 0.60 --min-cycle-count 30
python3 example/cpp/tools/analysis/analyze_straightness.py \
  example/cpp/experiments/_runs/natural_trot_rep1
```

需要 GUI 时在末尾加 `--view --camera-follow`；需要换 DDS 域或录制时，
继续追加 `--domain-id <n>`、`--controller-duration <s>` 等参数。

跑态验收：

```bash
bash example/cpp/scripts/run_running_trot.sh 100 running_trot_rep1
python3 example/cpp/tools/analysis/analyze_running_gait.py \
  example/cpp/experiments/_runs/running_trot_rep1
```

该入口使用 `period=0.26 s`、`duty=0.45`、`step=0.320 m`，形成短暂腾空相；
末端急停用于验证跑态到四足 WBC 支撑的安全交接。

3 m/s 持续跑态验收：

```bash
bash example/cpp/scripts/run_sustained_running.sh --headless
python3 example/cpp/tools/analysis/analyze_sustained_running.py \
  example/cpp/experiments/_runs/<run-name>
```

该入口单独使用 `running-trot` 参考，不是对角小跑的改名；验收器还检查
腾空比例、对角同步、摆腿高度和支撑分布。

两个入口在 WSL 中默认把 MuJoCo 和控制器分别固定到 CPU 2、3；可用
`TROT_CPU_AFFINITY_SIM`、`TROT_CPU_AFFINITY_CTRL` 覆盖，或设
`TROT_CPU_AUTOPIN=0` 关闭自动固定。直线验收相对世界 +X，要求横向位置
p95≤0.10 m、末端≤0.12 m、航向 p95≤8°、横向速度 p95≤0.25 m/s。

## 相关文档

- [`../../../docs/CODE_GUIDE.md`](../../../docs/CODE_GUIDE.md)
- [`../MODULES.md`](../MODULES.md)
- [`../../../docs/RESEARCH_INDEX.md`](../../../docs/RESEARCH_INDEX.md)
