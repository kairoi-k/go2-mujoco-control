# Go2 WBC Full：环境变化下的反应式动作编排

## 目标

这条线验证的不是“把急停动作接在小跑动作后面”，而是让同一套 WBC/MPC 植物持续运行，只把事件转换成连续的速度、转向、步长、占空比和抬脚高度参考。这样不需要为每一对动作制作独立关节轨迹。

当前站起/趴下仍沿用原有插值；进入 gait 后，脚本事件和传感器事件共用同一过渡层。

## 自动环境感知（当前主线）

模拟器通过 rt/go2/environment_heightmap 发布 base_link 坐标系高度图；控制器订阅后在每个控制周期检查地图新鲜度，并把前方中心、左、右三个扇区压缩成障碍扫描。检测到连续 40 ms 的有效障碍后，自动生成 obstacle_left/right token，送入同一个 MotionEventResponseLayer。这里没有拼接关节动作，也没有切换 WBC/MPC 植物。

障碍场景使用带碰撞的 reactive_obstacle 实体；验收同时要求：地图有效率不低于 95%、地图年龄不超过 150 ms、检测延迟（预热结束后）不超过 0.50 s、目标方向正确、机身发生同向横移、WBC 残差不超过 1e-3、姿态不超过 0.25 rad、障碍物真实接触力和接触次数均为 0。无障碍基线必须全程不产生 obstacle 事件。

可复现实验：先运行 baseline 与 scene_reactive_obstacle 两组，再用 example/cpp/tools/analyze_auto_environment.py 生成 JSON/Markdown 严格报告。原始 CSV、接触真值和 simulator/controller 日志只作为验收证据，不作为源码提交内容。
## 运行链路

```text
事件脚本 / IMU+速度+接触
          ↓
优先级选择（急停/冲击 100，障碍 80，打滑/低摩擦 60，转向 40）
          ↓
连续目标参考 + 速度/转向/步长变化率限制
          ↓
同一套 gait kernel → 同一套 WBC/MPC
```

实现位置：

- `example/cpp/trot/motion_event_response.h`：事件解析、自动检测、优先级、参考过渡和限幅。
- `example/cpp/trot/trot_experiment_control.cpp`：把事件层接入 gait 和传感器快照。
- `example/cpp/trot/trot_experiment_wbc.cpp`：MPC/WBC 读取统一参考。
- `example/cpp/trot/trot_experiment_diagnostics.cpp`：CSV 记录事件、目标和实际参考。
- `simulate/src/main.cc`：仿真中的速度冲击和地面摩擦突变入口。

## 事件策略

| 事件 | 参考目标 | 优先级 |
|---|---|---:|
| `emergency_stop` / `impact` | vx、vy、yaw→0，提高占空比，缩短步长 | 100 |
| `obstacle_left/right` | 降速并向对应方向转向 | 80 |
| `slip` / `low_friction` | 降速、缩短步长、提高占空比 | 60 |
| `turn_left/right` | 降速并改变 yaw 参考 | 40 |

参考变化率限制避免了“τ=0 直接跳到 τ*”式切换。自动检测默认预热 1.5 s；高度图障碍需连续 40 ms 且地图年龄不超过 150 ms；冲击需要至少两个接触且满足 IMU 加速度阈值；打滑需要持续速度误差。冲击信号有释放滞回，避免同一次冲击在高频重复触发。

事件脚本时间以 gait 开始为零，不是进程启动时间。--reactive-events 可在没有脚本时单独开启 IMU、速度和接触检测；--auto-environment 额外开启高度图订阅和障碍检测；--event-script path 只用于确定性复现实验。

## 可复现实验

```bash
# 无事件回归
bash example/cpp/scripts/run_trot.sh 80 go2_reactive_baseline_final_2026-08-19 \
  --wbc-full --step-length 0.091 --period 0.60 --duty 0.75 --foot-lift 0.020 \
  --kernel raibert-trot --raibert-velocity-gain 0.05 --raibert-max-adjustment 0.010 \
  --tau-limit 35 --max-cycles 12 --headless --domain-id 130 \
  --controller-duration 14 --reactive-events

# 确定性混合事件
bash example/cpp/scripts/run_trot.sh 80 go2_reactive_event_final_2026-08-19 \
  --wbc-full --step-length 0.091 --period 0.60 --duty 0.75 --foot-lift 0.020 \
  --kernel raibert-trot --raibert-velocity-gain 0.05 --raibert-max-adjustment 0.010 \
  --tau-limit 35 --max-cycles 12 --headless --domain-id 131 --controller-duration 14 \
  --event-script example/cpp/experiments/go2_reactive_event_final_2026-08-19/event_script.txt

# 真实仿真摩擦突变：mu 1 -> 0.05，持续 1 s
bash example/cpp/scripts/run_trot.sh 80 go2_reactive_low_friction_push08_2026-08-19 \
  --wbc-full --step-length 0.091 --period 0.60 --duty 0.75 --foot-lift 0.020 \
  --kernel raibert-trot --raibert-velocity-gain 0.05 --raibert-max-adjustment 0.010 \
  --tau-limit 35 --max-cycles 12 --headless --domain-id 135 --controller-duration 14 \
  --reactive-events --push-time 8 --push-vel-x 0.8 --push-duration 0.2 \
  --friction-time 8 --friction-mu 0.05 --friction-duration 1
```

结果可用以下工具读取：

```bash
python3 example/cpp/tools/analyze_reactive_events.py \
  example/cpp/experiments/go2_reactive_event_final_2026-08-19
```

## 已验证结果

- 2026-08-20 自动高度图验收：无障碍基线与真实碰撞障碍两组均严格通过；地图有效率 100%、最大年龄 20 ms；障碍检测延迟 90 ms，自动选择 obstacle_left，机身横移 0.453 m，障碍物接触次数/法向力均为 0。
- 2026-08-20 物理冲击自动验收：0.8 m/s 仿真速度冲击被自动识别为 impact，随后 0.5 s 进入 emergency_stop；controller/safety/quality/completion = 0/0/0/0，最大 roll 4.35°、pitch 9.27°。
- 低摩擦负向验收保持无误报：仅降低地面摩擦但未产生可观测滑移时不生成事件；低摩擦 token 的响应和滑移确认由单元测试覆盖，不能把“潜在摩擦变化”冒充成已检测事件。


- `go2_reactive_baseline_final_2026-08-19`：12 周期，controller/safety/quality = `0/0/0`，最大 roll 1.27°、pitch 9.27°。
- `go2_reactive_event_final_2026-08-19`：转向、急停、障碍转向、打滑、冲击全部按脚本触发，12 周期 `0/0/0`，最大 roll 1.23°、pitch 9.27°。CSV 中事件相对 gait 起点约 2 ms 内出现。
- `go2_reactive_random_final_2026-08-19`：确定性混合序列 12 周期 `0/0/0`，最大 roll 1.19°、pitch 9.25°。
- `go2_reactive_low_friction_final_2026-08-19`：仿真日志确认 8.002 s 降到 μ=0.05、9.002 s 恢复，12 周期 `0/0/0`。
- `go2_reactive_low_friction_push08_2026-08-19`：摩擦突变叠加 0.8 m/s 速度冲击，自动冲击响应，12 周期 `0/0/0`，最大 roll 7.75°。
- `go2_reactive_auto_push_2026-08-19`：1.5 m/s 速度冲击触发安全失败（roll 105.89°、pitch 77.13°），作为当前控制器稳定包络的明确失败边界保留，未冒充成功。

这些结果证明了“事件→统一连续参考→同一 WBC/MPC”链路和最坏工况记录已经成立，但不等于对任意真实障碍都已完成感知；实机部署还需要把上游感知事件接入同一接口，并重新标定阈值。
