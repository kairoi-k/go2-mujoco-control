# WBC Full 反应式事件实验

这是确定性事件脚本的最终可视化证据。脚本按 gait 起点计时，依次包含转向、急停、障碍转向、打滑和冲击。

- 视频：`media/reactive_event_final.mp4`（1280×720，20 fps，15 s）
- 数据：`data.csv`
- 仿真真值：`contact_ground_truth.csv`
- 控制器日志：`controller.log`
- 运行元数据：`run_metadata.txt`

最终回归结果：12 周期，controller/safety/quality=`0/0/0`；最大 roll 约 1.23°，最大 pitch 约 9.27°。事件从 CSV 中按脚本顺序出现，参考更新延迟约 2 ms。

汇总：

```bash
python3 example/cpp/tools/analyze_reactive_events.py \
  example/cpp/experiments/go2_reactive_event_final_2026-08-19
```
