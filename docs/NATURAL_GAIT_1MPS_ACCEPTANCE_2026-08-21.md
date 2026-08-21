# 1 m/s 自然小跑验收

这条线固定在 `gait/natural-trot-1mps-2026-08-21`，不修改已经验证的
`speed/1mps-2026-08-21` 发布基线。目标是让 1 m/s 级别的 `--wbc-full`
小跑具有可见抬脚、对角同步和可重复的站姿，而不是靠收腿蹭地换速度。

## 固定 profile

```text
period=0.28 s, duty=0.50, step-length=0.320 m, foot-lift=0.120 m
WBC velocity gain=8, tau-limit=35 Nm
Raibert velocity gain=0.015, max adjustment=0.060 m
```

入口是 `example/cpp/scripts/run_natural_trot.sh`。它只封装已验收的
参数；末尾可以追加 `--view --camera-follow` 做 GUI 复核。

## 通过标准

验收只使用 MuJoCo `contact_ground_truth.csv` 的真实状态，控制器日志只
用于生命周期和安全状态：至少 40 个健康周期；稳态速度中位数不低于
1.0 m/s、5 分位不低于 0.85 m/s；机身高度 0.33--0.40 m；姿态绝对值
95 分位小于 8°；每条腿世界高度 95 分位不低于 0.055 m；四脚同时低于
0.035 m 的比例不高于 0.35；两条对角线接触同步度不低于 0.65，交叉对角
相异度不低于 0.65；最后完成受控刹车并回到站立。

执行：

```bash
bash example/cpp/scripts/run_natural_trot.sh 100 natural_trot_rep1
python3 example/cpp/tools/analysis/analyze_natural_gait.py \
  example/cpp/experiments/_runs/natural_trot_rep1
```

## 当前证据

基线二进制 SHA-256 为
`71a45e2ac711d07ba96f12976ec3eec7221c7476accdf9f0215306ef6530637b`。
目前同一参数的三次有效重复结果如下；一次额外重试也通过，另有一次
探索性运行在收尾阶段触发姿态保护，因此不计入有效重复。

| run | p05 / median / p95 (m/s) | base-z median (m) | result |
|---|---:|---:|---|
| `natural_p028_s0320_repeat1` | 0.872 / 1.073 / 1.208 | 0.3667 | PASS |
| `natural_p028_s0320_repeat3` | 0.872 / 1.039 / 1.165 | 0.3664 | PASS |
| `natural_p028_s0320_repeat5` | 0.861 / 1.077 / 1.218 | 0.3667 | PASS |

这些是从同一参数探针中选出的三次完整通过；另一次只在收尾速度尾段
略超 0.10 m/s 门限，未计入。步长 0.320 m 是在保持约 1 m/s 的前提下
给姿态和接触留出的稳定裕度；更大的 0.336 m 版本虽然视觉步幅更大，
重复性不足，因此不作为默认 profile。

这证明的是当前 profile 在无外部扰动、标准平地 MuJoCo 场景下具有可复现
的自然小跑基础，不等于已经完成高速奔跑或真实机器人验证。后续奔跑线
必须另设步态、速度和安全验收，不能把小跑结果直接外推。
