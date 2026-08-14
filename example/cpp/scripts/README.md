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

```bash
bash example/cpp/scripts/go2sim walk --view
bash example/cpp/scripts/go2sim walk
bash example/cpp/scripts/go2sim fast --view
```

实验输出：名称 `go2_*` → `experiments/`；其它临时输出 → `experiments/_runs/`。DDS domain 使用 203–207。

历史批量扫参和机器专属启动器不进入公开候选树；需要追溯时以完整开发仓库为档案来源。

## 相关文档

- [`../../../docs/CODE_GUIDE.md`](../../../docs/CODE_GUIDE.md)
- [`../MODULES.md`](../MODULES.md)
- [`../../../docs/RESEARCH_INDEX.md`](../../../docs/RESEARCH_INDEX.md)
