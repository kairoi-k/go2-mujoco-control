# EXP-17 重复运行：八步闭环与平滑终点收敛

这是 EXP-17 的第二次独立运行，用于验证结果重复性；完整方法、门槛和历史问题见相邻的首次正式运行 README。

## 结果

- 八步全部完成，8 次落脚均触发。
- 修正前终点误差：`23.850 mm`；3 秒平滑修正后：`1.918 mm`。
- 最终最大 roll/pitch：`1.715° / 1.290°`。
- 最大支撑漂移：`2.062 mm`；最大关节误差：`0.0794 rad`；最大估计力矩：`7.88`。
- 最终健康门禁和 `10 mm` 终点收敛判定均通过。

复现配置：`example/cpp/configs/go2_eight_step_terminal_precomp_210.txt`；运行入口使用 `--world-feedback`。

库内证据是 `sequence_summary.csv` 和 `sequence_overview.png`。`controller.log` / `data.csv` / `simulator.log` 不在 git。
