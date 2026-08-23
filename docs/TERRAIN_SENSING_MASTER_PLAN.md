# 地形感知与适应主方案（2026-08-23）

> 执行者注意：本文件是给持续工作模型的唯一权威计划。原则部分每一条都来自
> 实际失败教训，不许绕过。聊天汇报永远短消息，不写长报告。

## 0. 已核实的事实基线

- 仓库：`~/dev/go2-mujoco-control-terrain`，分支 `terrain/adaptation-2026-08-21`，
  基线提交 `671a0d4`（在此之上工作）。`git status` 应只剩未跟踪的
  `docs/`、场景 XML、`_runs/`。
- 已验证成果（都有数据或视频证据）：
  - 爬行内核 + 地形覆写 + 机身抬升：5cm 隔离带稳定跨越（2/2，俯仰峰值≈9°）。
  - 纯小跑（无地形覆写）靠动量 2/3 过 5cm 隔离带，速度约为爬行十倍。
  - 高速自然步态线（1m/s 小跑、3m/s 冲刺）已合并进本分支，测试 27/27 绿。
  - 验收视频：OneDrive 收件箱 `go2_terrain_crawl_barrier05_2026-08-22`（爬行）
    与 `go2_terrain_trot_barrier05_2026-08-22`（小跑）。
- **关键事实：现有高度图是上帝视角**。`simulate/src/unitree_sdk2_bridge.h` 的
  `PublishEnvironmentHeightMap()` 直接遍历场景全部 geom 的解析外形填栅格，
  无射线、无遮挡、无噪声、无视场。它不是传感器。本方案的第一步就是换成真传感。
- 真机传感器参照：Unitree Go2 的 L1 4D 激光雷达：360°×90° 视场，
  0.05–30m 量程，21600 点/秒。
- 传感可行性已实测：`/mnt/c/Workspace/lidar_probe.py`（mj_ray 从头高 0.36m
  向下环形扫描）在 10cm 隔离带场景下，base≥0.2 就能看到障碍顶
  （读数 0.11 vs 真值 0.10，含 5mm 标记板），单帧覆盖约 45/320 格，
  遮挡自然出现。性能：每帧百级射线对仿真毫无压力。

## 1. 目标

狗用自己的（仿真的）传感器看到前方地形并做判断，依次攻克：
5cm 隔离带（已通，作回归）→ 10cm → 15cm 原版隔离带/高台 → 4 级连续台阶
→ 泛化地形（粗糙、跨栏）。最终目标是可迁移到真机的路线。

## 2. 架构决策（含调研依据，不许随意推翻）

1. **传感器仿真用 mj_ray 激光**，不用深度相机渲染。依据：API 稳定、CPU 成本
   低、社区有 MuJoCo-LiDAR / mjlab RayCastSensor 先例；深度渲染（mj_pc /
   mujoco_RGBD 路线）留作备选。L1 参数：头高≈0.36m，向下环组
   -15°/-30°/-45°/-60°/-75°，前向半球加密，总量 200–600 射线/帧，20–50Hz。
2. **地图是机器人系局部栅格 + 时序融合**：5cm 分辨率、1.0m×0.8m（与现图同
   尺寸），每格取射线命中 z 的滑动 max，未知格 NaN/未知标记。遮挡不填补
   （真实）。后续可加方差层，初期不要。
3. **双图并存**：真值图保留（改名 oracle 发布 topic 或直接内部对比用），
   用于控制侧开发和激光图误差评估；控制器验收时必须吃激光图。
4. **控制走经典接缝叠加**：在内核输出后、`commanded_world_feet_`/
   `commanded_body_feet_` 接缝处做地形修正。依据：内核相位图被篡改是前任
   13 小时卡死的直接死因，永不重犯。
5. **接近行为用实验层 FSM**：cruise → creep（缩步减速）→ mount → traverse →
   cruise。**相位永不冻结**，只调步长/目标。
6. **RL 是并行备选线**：Extreme Parkour（2023）证明单目深度+RL 可做过障
   迁移；`unitree_rl_lab` 在本 workspace 现成。经典线卡住两阶段以上再启动。

## 3. 分阶段计划（每阶段独立验收、独立提交）

### P0 激光高度图（传感真源）

改动点（都在 `simulate/src/unitree_sdk2_bridge.h`，加新函数不改旧函数；
插入位置参照 `PublishEnvironmentHeightMap`：成员变量声明区在文件尾部
`environment_heightmap` 成员旁，发布调用点与现有地图同一发布周期。
`mj_ray` 签名：`(model, data, origin3, dir3, geomgroup=nullptr, flg_static=1,
bodyexclude=base_body_id, geomid_out)`，返回命中距离（负值=未命中），调用前
确认仿真循环里已 mj_forward。伪码顺序：取 base xpos/xmat → 环组逐射线 →
命中点减 base 转局部坐标 → (ix,iy) 落栅 → 每格 max 融合进 1.5s 滑动窗 →
填 HeightMap_ 写新 topic）：
- 新增 `PublishLidarHeightMap()`：从 base_link 位姿发射线（环组见 §2.1），
  `mj_ray(..., flg_static=1, bodyexclude=base_body_id, ...)`；geomgroup 排除
  contype==0&&conaffinity==0 的装饰 geom（现桥的 marker 板会被误读）。
- 命中点转 base 系，填入 5cm 栅格，滑动窗口 max 融合（保留最近 1.5s）。
- 新 DDS topic `rt/go2/lidar_heightmap`，同 `HeightMap_` 类型，未知格填 NaN。
- CLI 标志 `--sensor-map`（controller 侧）切换订阅源；默认仍订阅真值图。
验收：场景 `scene_barrier_low10.xml`，狗静止，对比激光图与真值图：
被覆盖的真值格中 |误差|≤2cm 的比例 ≥85%；障碍顶被看到的最早 base_x ≤0.25。
指标 + 抽帧（脚本见 §6）双验收后提交。

### P1 三场景观测校验

`--terrain-observe` + `--sensor-map` 跑平地/10cm 隔离带/4 级楼梯三个场景，
前腿规划状态序列人工核对：平地恒 kValid(z≈0)，隔离带接近时前腿出
kStepTooHigh→kValid(z≈0.10)，楼梯逐级出现。任一场景误报即修 P0。

### P2 接近控制器（干净版，一次性成型）

在实验层加 `TerrainApproachFsm`，状态机四个状态，转移条件全部来自规划器
输出（前腿 plan 是否 elevated、前后支撑投票），动作只有两个旋钮：
`SetGaitStepLength(scale)` 和摆动腿世界高下限 `wz = max(wz, patch_z+0.025)`。
- cruise：scale=1。creep：scale=0.35（前腿 lookahead 0.10m 内出现 elevated
  plan 且无支撑脚在台上）。mount：前脚上台确认（前支撑投票成立）后保持
  scale=0.35 直到后脚也上过。traverse：四脚全过后 scale 回 1。
- 机身俯仰参考：pitch_ref=atan2(前支撑均值-后支撑均值, 0.40m)，限 ±0.2rad，
  0.6rad/s slew；WBC 姿态任务跟踪它（trot_experiment_wbc.cpp 的
  desired_angular_acc_body 目标从 0 改为 pitch_ref）。
- 摆动腿判定：内核有 schedule 用 schedule；没有就用对角相位表
  （`fmod(phase+pair*0.5) >= duty`）。**禁止**用 touchdown_target 判摆动
  （raibert 四腿恒非零）。
验收：5cm 隔离带，crawl 与小跑各 3 连跑，成功率分别 ≥2/3 和 ≥2/3，
跨越窗口 |pitch|≤12°，指标+抽帧双验收。

### P3 10cm → 15cm

场景 `scene_barrier_low10.xml` 已存在；15cm 用锁定的
`scene_barrier_acceptance.xml`。逐级加：规划器 `max_step_up_m` 提到 0.18、
覆写上限提到 0.16、foot-lift ≥0.14。每级 3 连跑 ≥2/3 过，俯仰 ≤15°，
再上一级。任何一级连续两次 0/3 → 停，回 §5 坑清单对照，不硬试。

### P4 4 级连续台阶（锁定场景）

`scene_stair_acceptance.xml`（台阶实际 0.2m 高，注意）。mount 循环每级一次。
验收：连续上 4 级、机身 y 向漂移 ≤0.15m、姿态 p95 ≤12°、无 quality guard
拒绝。通过后拍 720p30 全程视频进 OneDrive 收件箱。

### P5 泛化与噪声

高度图加三态噪声（高斯 σ=1cm、5% 野点、遮挡已知不补）、扫描 20Hz 时延。
过一遍 P2–P4 回归。高台（双侧通行）、跨栏（下方空隙）场景各建一个
（新文件，不动锁定场景）。

### P6 RL 并行线（仅当经典线两阶段卡死）

`unitree_rl_lab` Go2 任务加楼梯地形 + 高度观测（参考其 TerrainGeneratorCfg
和 issue #52 的失败记录——trimesh 参数是嫌疑），教师-学生结构参考 Extreme
Parkour。产物是独立策略，不回灌经典栈。

## 4. 工作铁律（每条都对应一次真实事故）

1. 一次只改一处，改完必：`cmake --build` 零 error → `ctest` 27/27 → 至少一次
   真跑。三步缺任何一步不许跑下一改。
2. 场景锁定文件 `scene_barrier_acceptance.xml`、`scene_stair_acceptance.xml`
   不许改内容；要变就建新文件。
3. 验收必须双标准：CSV 指标 + 抽帧亲眼确认。只有指标没有帧不算过。
4. 成功率声明必须 ≥3 次同参数运行，报 x/N。单次成功不许说“能过”。
5. 失败先回滚（`git checkout -- <file>`），不许在半成品上继续叠补丁。
   连续两个同方向改动都变差 → 这条思路判死刑，写进 §5 再换路。
6. 永不冻结步态相位；永不往内核里塞状态机；永不让支撑腿吃单腿地形偏移。
7. 跑批命令模板见 §6，域名用 212，run 名带日期和假设编号。
8. 每完成一阶段 `git commit`，消息写清验证证据（run 名、成功率、帧路径）。
9. 联网工具、SSH、视频管线先看 `C:\Users\w1881\.pi\agent\TOOLS.md`。
10. 长会话用 goal 工具追踪当前阶段，阶段切换时检查对齐。

## 5. 已知坑清单（新增坑按格式续写）

| 坑 | 症状 | 教训 |
|---|---|---|
| 内核相位冻结 | 相位钉死在固定值，4 秒窗口空转 | 前任 13h 死因；相位只读不写 |
| 支撑腿单腿 dz | 一脚判定高 11cm 整机被拽翻 22° | 支撑腿只吃多脚共识的慢速共享参考 |
| 落点重定向+前视+蠕行叠加 | 整机悬空 30cm 凝固 | 叠加层间互相打架；FSM 必须一次成型 |
| 摆动腿误用 touchdown 判定 | raibert 四腿全“摆动”，地形逻辑失灵 | 用内核 schedule 或相位表 |
| apex 叠加 | 脚飞到台面上方 10cm 悬空 | 用 `max(内核波形, 台面+余量)` 下限形式 |
| 前视 0.20m | 越过台面后沿，查到空区 | 用 0.10m 且不盖掉本地更优解 |
| 对角半径脚印 | 宽障碍提前几分米进图 | 已修：分轴半长（b07103c） |
| 机器时序方差 | 同参数结果 0.9m 与 0.2m 并存 | 验收看多次，时钟暂停期间别下结论 |
| 文件名秒级命名 | 30Hz 抽帧互相覆盖只剩 1/3 | 帧用序号命名 |
| heredoc 引号 | bash 内嵌 python 脚本爆炸 | 补丁脚本一律写 /mnt/c/Workspace/tmp_*.py 再执行 |

## 6. 命令手册（直接可复制）

```bash
# 构建+测试（controller）
cd ~/dev/go2-mujoco-control-terrain/example/cpp/build && cmake --build . -j$(nproc) && ctest
# 模拟器重建（P0 改了 bridge 后必须执行；否则跑的还是旧传感）
cd ~/dev/go2-mujoco-control-terrain/simulate/build-terrain && cmake --build . -j$(nproc)
# 跑批（参数按阶段替换；域 212）
cd ~/dev/go2-mujoco-control-terrain && bash example/cpp/scripts/run_trot.sh 90 <run名> \
  --headless --kernel crawl --wbc-full --terrain-act --task stand-walk-lie \
  --controller-duration 30 --period .80 --duty .75 --step-length .13 \
  --foot-lift .09 --kp 63 --kd 2.8 --direction 1 --domain-id 212 \
  --scene-file unitree_robots/go2/scene_barrier_low05.xml
# 指标（_runs/<run名>/data.csv，motion_stage==2 是行走段）
# 抽帧视频（改 replay_frames.py 参数；帧必须序号命名）
cd ~/dev/go2-mujoco-control-terrain && ~/isaac_env/bin/python /mnt/c/Workspace/replay_frames_hd.py \
  unitree_robots/go2/<场景>.xml example/cpp/experiments/_runs/<run名>/data.csv \
  <t0> <t1> 0.0333 /mnt/c/Workspace/<帧目录>
# 编码进收件箱（cv2 mp4v 可用；ffmpeg 缺库别用）
```

## 7. 小模型立刻执行项

从 P0 开始。P0 的射线参数、融合规则、topic 名、验收数字上面全有，照着做；
不确定就查 §5；跑通前不许进 P1。
