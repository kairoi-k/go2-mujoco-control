# EXP-19 重复运行：32 步 x/y 闭环

这是 EXP-19 的独立重复运行，使用完全相同的配置和二进制入口，用于确认 32 步压力测试的重复性。

## 结果

- 32/32 步完成，32 次落脚均触发，支撑接触达标比例每步均为 1.000000。
- 最大 roll=2.092°、pitch=1.676°、支撑漂移=13.828 mm、关节误差=0.1080 rad、估计力矩=10.903。
- 第 32 步终点修正前世界误差为 (27.551, 20.115) mm；3 秒平滑修正后为 3.588 mm，收敛判定通过。
- 最终世界位置 actual=(0.574849, -0.011353) m，command=(0.572021, -0.009144) m，误差为 (2.827, -2.209) mm；与首次运行结果几乎重合。

复现配置：example/cpp/configs/go2_32_step_continuation_060_rr_y0.txt。

库内证据是 sequence_summary.csv 和 sequence_overview.png。controller.log / data.csv / simulator.log 不在 git。
