# Go2 WBC Full：环境变化下的反应式动作编排

## 目标

这条线验证的不是“把急停动作接在小跑动作后面”，而是让同一套 WBC/MPC 植物持续运行，只把事件转换成连续的速度、转向、步长、占空比和抬脚高度参考。这样不需要为每一对动作制作独立关节轨迹。

当前站起/趴下仍沿用原有插值；进入 gait 后，脚本事件和传感器事件共用同一过渡层。

## 自动环境感知（当前主线，代码提交 `239f940`）

模拟器通过 rt/go2/environment_heightmap 发布 base_link 坐标系高度图；控制器订阅后在每个控制周期检查地图新鲜度，并把前方中心、左、右三个扇区压缩成障碍扫描。检测到连续 40 ms 的有效障碍后，自动生成 obstacle_left/right token，送入同一个 MotionEventResponseLayer。这里没有拼接关节动作，也没有切换 WBC/MPC 植物。
障碍扫描短暂丢帧不会立即重置已确认障碍；只有连续清空达到 250 ms 才重新允许同一检测器触发，避免同一实体的瞬时丢帧造成重复动作。

支撑脚运动学也作为独立的滑移证据：控制器用高状态和腿端运动学计算处于接触/支撑相的脚在世界系的速度，至少两个支撑脚同时达到阈值并持续确认后才生成 `slip` 或 `low_friction`。证据使用 0.80 s 泄漏窗口，DDS 重复 tick 不会清零累计量；低摩擦另有确认/释放滞回、其他事件后的重新触发抑制，以及当前支撑脚速度门控。它补上了“机身速度尚未明显偏离、但支撑脚已经移动”的检测通道；单元测试覆盖了这条路径。当前实物摩擦仅下降而未形成可观测滑移的试验仍不宣称触发成功。

障碍场景使用带碰撞的 reactive_obstacle 实体；验收同时要求：地图有效率不低于 95%、地图年龄不超过 150 ms、检测延迟（预热结束后）不超过 0.50 s、目标方向正确、机身发生同向横移、WBC 残差不超过 1e-3、姿态不超过 0.25 rad、障碍物真实接触力和接触次数均为 0。无障碍基线必须全程不产生 obstacle 事件。

可复现实验：先运行 baseline 与 scene_reactive_obstacle 两组，再用 example/cpp/tools/analyze_auto_environment.py 生成 JSON/Markdown 严格报告。原始 CSV、接触真值和 simulator/controller 日志只作为验收证据，不作为源码提交内容。
物理冲击的自动检测与急停验收使用 `example/cpp/tools/analyze_auto_impact.py`；它把模拟器日志中的施力时刻与控制器 `state_tick_s` 对齐，单独检查检测延迟、急停延迟、速度突变、姿态、WBC 残差和最终保持。

CSV 的 `event_source` 记录事件来源：`1=scheduled`（脚本或自动安排的后续急停）、`2=sensor`（速度/IMU/高度图自动检测）、`3=safety_latch`（急停保持）。自动障碍和冲击验收要求来源必须为 `sensor`，不能用脚本事件冒充环境感知。
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

参考变化率限制避免了“τ=0 直接跳到 τ*”式切换。自动检测默认预热 1.5 s；高度图障碍需连续 40 ms 且地图年龄不超过 150 ms；有机身速度时，冲击以速度突变为主判据，避免把受控步态加速度误报为冲击；没有速度测量时才回退到 IMU 加速度判据。冲击信号有释放滞回，避免同一次冲击在高频重复触发。

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

实体低摩擦区域正向验收使用 `unitree_robots/go2/scene_low_friction_patch.xml`：正常平面后接入带碰撞的 `mu=0.0001` patch，不使用 `--friction-time`；为保证物理滑移可观测，验收采用更快的探测步态。控制器由支撑脚世界系运动学自动确认 `low_friction(sensor)`，并由 `analyze_auto_low_friction.py` 检查场景、事件来源、进入 patch 的时序、证据阈值、速度降低、姿态和求解器状态。

```bash
bash example/cpp/scripts/run_trot.sh 100 go2_auto_environment_low_friction_patch_sensor_fast_v14_2026-08-21 \
  --wbc-full --step-length 0.14 --period 0.55 --duty 0.65 --foot-lift 0.025 \
  --kernel raibert-trot --raibert-velocity-gain 0.05 --raibert-max-adjustment 0.010 \
  --tau-limit 35 --max-cycles 12 --headless --domain-id 222 \
  --controller-duration 12 --auto-environment --scene unitree_robots/go2/scene_low_friction_patch.xml
python3 example/cpp/tools/analyze_auto_low_friction.py \
  example/cpp/experiments/go2_auto_environment_low_friction_patch_sensor_fast_v14_2026-08-21
```

结果可用以下工具读取：

```bash
python3 example/cpp/tools/analyze_reactive_events.py \
  example/cpp/experiments/go2_reactive_event_final_2026-08-19
```

## 已验证结果

- 2026-08-21 自动高度图验收（提交 `239f940`）：无障碍基线与真实碰撞障碍两组严格通过；地图有效率 100%、最大年龄 20 ms；自动选择 obstacle_left，机身横移 0.222 m，障碍物接触次数/法向力均为 0。
- 2026-08-21 物理冲击自动验收（提交 `239f940`）：0.8 m/s 仿真冲击在 state_tick=8.002 s 被识别，2 ms 后进入 impact，0.5 s 后进入 emergency_stop；最大速度突变 0.802 m/s，最大 roll 0.028 rad、pitch 0.162 rad，WBC 残差 1.6964e-5，全部状态码为 0。
- 2026-08-21 实体低摩擦区域验收（提交 `239f940`）：正常地面进入 `mu=0.0001` 的碰撞 patch 后由支撑脚运动学自动生成 `low_friction(sensor)`；更快探测步态连续两次严格通过，证据峰值 0.2420/0.1571，姿态/求解器/质量状态码均为 0。全局摩擦仅下降但无可观测滑移仍保持负向边界，不冒充成功。
- 低摩擦负向验收保持无误报：仅降低地面摩擦但未产生可观测滑移时不生成事件；支撑脚运动学检测通道及低摩擦 token 的响应由单元测试覆盖，但当前摩擦-only 工况尚未形成严格的自动 `sensor` 事件，不把“潜在摩擦变化”冒充成已检测事件。


- `go2_reactive_baseline_final_2026-08-19`：12 周期，controller/safety/quality = `0/0/0`，最大 roll 1.27°、pitch 9.27°。
- `go2_reactive_event_final_2026-08-19`：转向、急停、障碍转向、打滑、冲击全部按脚本触发，12 周期 `0/0/0`，最大 roll 1.23°、pitch 9.27°。CSV 中事件相对 gait 起点约 2 ms 内出现。
- `go2_reactive_random_final_2026-08-19`：确定性混合序列 12 周期 `0/0/0`，最大 roll 1.19°、pitch 9.25°。
- `go2_reactive_low_friction_final_2026-08-19`：仿真日志确认 8.002 s 降到 μ=0.05、9.002 s 恢复，12 周期 `0/0/0`。
- `go2_reactive_low_friction_push08_2026-08-19`：摩擦突变叠加 0.8 m/s 速度冲击，自动冲击响应，12 周期 `0/0/0`，最大 roll 7.75°。
- `go2_reactive_auto_push_2026-08-19`：1.5 m/s 速度冲击触发安全失败（roll 105.89°、pitch 77.13°），作为当前控制器稳定包络的明确失败边界保留，未冒充成功。

这些结果证明了“事件→统一连续参考→同一 WBC/MPC”链路和最坏工况记录已经成立，但不等于对任意真实障碍都已完成感知；实机部署还需要把上游感知事件接入同一接口，并重新标定阈值。

## 当前自动感知优先级证据（commit `239f940`）

最新交付以当前提交的基线/障碍/冲击/抢占/低摩擦验收为准：实体障碍期间发生 0.8 m/s 物理冲击，`obstacle_left → impact` 在 2 ms 内完成抢占，随后约 0.5 s 进入 `emergency_stop`；实体低摩擦 patch 由支撑脚运动学自动检测。所有严格分析器通过，无碰撞且所有状态码为 0。

组合实验的物理时间统一按 MuJoCo `state_tick_s` 计算；`cmd_time_s` 只用于控制器事件顺序，避免把两个时钟误读成提前触发。
