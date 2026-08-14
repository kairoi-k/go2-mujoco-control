# EXP-18 重复运行：24 步 x/y 闭环

这是 EXP-18 的独立重复运行，使用完全相同的配置和二进制入口，用于验证 24 步长序列不是偶然通过。

## 结果

- 24/24 步完成，24 次落脚均触发，支撑接触达标比例每步均为 1.000000。
- 全部步骤最大 roll=2.091°、pitch=1.493°、支撑漂移=13.609 mm、关节误差=0.1080 rad、估计力矩=10.903。
- 第 24 步终点修正前误差为 (26.260, 14.162) mm；3 秒平滑修正后为 2.909 mm，收敛判定通过。
- 最终世界误差为 (2.066, -2.049) mm，和 EXP-18 首次运行的 2.909 mm 几乎重合。

复现配置：example/cpp/configs/go2_24_step_continuation_060_rr_y0.txt。

库内证据是 sequence_summary.csv 和 sequence_overview.png。controller.log / data.csv / simulator.log 在归档，不进 git。
