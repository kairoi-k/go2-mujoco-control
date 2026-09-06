# Terrain frame audit v2：map capture pose registration（fa84f51 provenance）

本稿是 2026-09-07 的新审计草稿，写入 Windows cwd，保留旧 `terrain_frame_audit.md` 不覆盖。涉及生产源码的行号均由 Windows Git 对 `fa84f51` 的 `git show` 精确核对；运行目录 `example/cpp/experiments/_runs/b1_commitment_flat_fa84f51_20260907_0001` 的 metadata 也记录 `git_head=fa84f5185cb52de8fa2ef62d28cf3809998be23a`、`git_dirty=false`。此前 draft 开头的“source parent aa14604/后续工作树 gait lift WIP”已过时，不能作为本审计 provenance；本稿不把当前工作树改动归入 fa84f51。

## 已核实的时间、坐标与数据链

1. `simulate/src/unitree_sdk2_bridge.h`（fa84f51）`TerrainLidarLoop` 约 390-430 行在仿真锁内复制 `qpos/qvel/time`，随后用这份 snapshot 做正向运动学和发布。因此 `sensor_data->time` 与 `sensor_data->xpos/xmat` 是同一仿真时刻的生产者姿态。`PublishLidarHeightMap` 约 639-645 行写入 `stamp(sim_time)`、`frame_id("base_link")`、resolution 0.05、window 32x10、origin (-0.45,-0.225)。约 648-662 行用该 snapshot 的 base x/y/yaw 把窗口 cell 投到世界 x/y。

2. 同文件约 663-728 行先以世界射线得到 `hit_z`，按世界缓存 `lidar_world_z_/lidar_world_t_` 保存，发布时把 finite cell 写为 `world_z - base_pos[2]`（约 725-728 行）。缓存保留窗口外已见 cell 最长 1.5 s（约 697-700、703-724 行），但 HeightMap 只带本次 `sim_time`，没有每 cell 的观测时刻。因此 map stamp 不是所有 cell 的真实 capture age。

3. `example/cpp/trot/trot_experiment_lifecycle.cpp`（fa84f51）`HighStateMessageHandler` 约 79-93 行只复制最新 SportModeState、记录 arrival wall time，并从 `msg->stamp()` 计算 `highstate_stamp_s_`。fa84 仿真 publisher 约 `simulate/src/unitree_sdk2_bridge.h:965-976` 只填 position/velocity，没有给 HighState stamp 赋仿真时钟；故该 run 的 highstate stamp 为 0。`LidarHeightMapMessageHandler` 约 105-131 行只保存最新 HeightMap、map stamp 和 arrival 时间，无 capture pose/history。

4. `example/cpp/trot/trot_experiment_control.cpp`（fa84f51）`PublishTerrainControlSnapshot` 约 218-230 行把当前 LowState tick 作为 `input.state_stamp_s`，当前 IMU 姿态作为 yaw/quaternion，并用 HighState position 减 `RotateByQuaternion(quaternion, {-0.02557,0,0.04232})` 得当前 base position。约 327-335 行以这个当前 state stamp 调 `BuildTerrainModel(&work.map, work.input.state_stamp_s, ...)`。这与 map producer capture pose 是两条独立的 latest-only 路径。

5. `example/cpp/terrain/terrain_model.h`（fa84f51）约 265-320 行的 `BuildTerrainModel` 原样复制 frame、stamp、origin、cell data，并计算 `age_s=max(0,state_stamp-map_stamp)`。约 398-416 行的 `RereferenceHeightMapZ` 只对 z 做统一平移，保留 NaN；全仓库 fa84 源码中没有看到其生产调用。`example/cpp/terrain/terrain_planner.h:78-87` 的 `RotateBaseToWorld` 和约 489-495 行在 planner 当前输入姿态下把 candidate local foot 转成 world foot。

6. `HeightMap_` IDL（`/opt/unitree_robotics/include/unitree/idl/go2/HeightMap_.hpp`）仅有 stamp、frame_id、resolution、width/height、origin[2]、data，没有 capture xyz/yaw、sequence 或 cell age。`SportModeState_` 有 stamp 字段，但 fa84 simulator 没有填它。不能把 capture pose 偷塞入 frame 字符串，也不能把当前 LowState tick 复制成 map stamp。

## fa84 run 中的可复核现象与因果边界

`data.csv` row 6661 的 state tick 为 15.532，terrain map epoch/plan 为 201，map age 0.054，telemetry lidar stamp 15.476；当前 `world_base_z_m=0.356020178`。同 run state tick 15.476 的 base z 为 0.385326493，差值 `B_z(now)-B_z(map)=-0.029306315 m`。该行 FL/RR target world z 为 -0.005328632，而实际 FL/RR foot world z 为 0.028486328/0.023078319。平地 nominal foot-site 高度约 0.022 m 时，若把 map 的 capture-relative z 直接配当前 base z，未校正预测约 `0.022-0.029306315=-0.007306315 m`；将 `RereferenceHeightMapZ` 的等效 z 平移应用后约为 0.021999999 m（具体 target 与 nominal 尚有约 1.98 mm residual）。这支持“capture-relative z 与当前 base z 混用”的诊断，但不能单凭 CSV 证明 IK 失败的唯一原因：CSV 没有 producer capture pose、candidate local z、cell-level observation time 或完整目标序列。

更关键的是，问题不能只修 z。publisher 在 map capture 时用 base x/y/yaw 把 local window cell 投影到世界；consumer planner 却用当前 HighState-derived base x/y/yaw。若两时刻有水平平移或 yaw 变化，同一 cell 数值会对应错误的世界位置，candidate 查询也可能命中错误 cell；z-only re-reference 不改变 origin、cell identity 或 world x/y。即使 z 修正后，yaw/xy 误配仍可能造成错误 foothold、法向/边缘 margin 和后续 IK 问题。

## 最小完整接口方案

首选在 map 生产者旁发布一个与 HeightMap 原子配对的内部 observation envelope 或 companion topic，键为唯一 sequence + 同一 monotonic/simulation tick：

`TerrainMapObservation { HeightMap map; uint64 sequence; int64 capture_tick; Pose capture_base_world {x,y,z,yaw}; optional roll,pitch; uint64 producer_schema; CellAgeSummary/observation bounds }`。

外部 IDL 不可改时，使用单独的 `TerrainMapCapturePose` topic，严格按 `(sequence,capture_tick)` join；不得按 arrival wall time 或“最近 HighState”猜配。当前 simulator 已在 `TerrainLidarLoop` 同时拥有 `sensor_data->time`、`base_pos`、`yaw`，最小可靠做法是在同一 publish 逻辑把这四项和 HeightMap 放进内部 envelope；这比从 controller 的 latest HighState 回溯更可靠。若必须保留公共 HeightMap 原样，可把 companion pose 写入 controller 内部 queue，并对无匹配项 fail closed。

Capture pose 至少需 xyz+yaw。因为当前 map 窗口是 heading-aligned XY、z 是相对 base 的高度，世界重建为：

`p_W = B_W(t_m) + Rz(yaw_m) * [x_m,y_m,0] + [0,0,h_rel]`。

如果以后把地图解释为完整 body frame，必须再传 roll/pitch 或明确传感器 frame；不能沿用 `base_link` 名称而改变语义。对当前 planner，最小一致实现有两种：

- 在 map→world 变换中使用 capture pose：候选查询先用当前/候选 world XY 经 `Rz(yaw_m)^T*(p_W-B_W(t_m))` 回到 map local，读取 capture-relative h，再以 capture pose 生成世界表面点；foot target 在 plan commit 后保持不变。
- 若必须将 map 重采样到 current/world grid，按 capture pose 做 xy/yaw 变换并保守处理边界/alias：任何超界、无邻域、冲突 cell 或不确定 age 都标 unknown，不能以插值填成“安全地形”。

`RereferenceHeightMapZ` 可作为诊断或过渡 helper：已知 `dz=B_z(now)-B_z(t_m)` 时将相对高度减去 dz；它只能补 z，不能代表完整注册，也不能把未认证 map 送入 terrain actuation。

## HighState 历史路线的可用边界

HighState position 语义目前可复用，但 fa84 simulator 的 stamp 缺失，且 controller 只保留 latest。要走这条路，先在 producer 给 HighState 写入与 map 同一时钟域的 stamp；controller 保存覆盖“最大允许 map age + 调度余量”的 ring buffer（建议初始 250-300 ms，最终阈值由实测定），按 map stamp 找 bracket，平移线性插值、yaw 用 unwrap/最短弧插值，姿态若需要则 quaternion SLERP。缺 stamp、时钟域不一致、无 bracket、相邻样本 gap 超界或只能 extrapolate 时拒绝注册。HighState history 是 fallback；在仿真端能直接取得同一 `sensor_data` snapshot 时，producer-side companion pose 仍是首选。

必须复用当前 IMU→base 的偏移语义（`trot_experiment_control.cpp:221-228`），并明确 offset 在 world 中按该时刻 quaternion 旋转。不能用某一时刻的 IMU position 与另一时刻的姿态/offset 拼成 base pose；也不能让 map registration 使用一套 pose、WBC 当前观测再使用另一套 frame 解释。

## age 与 fail-closed 合同

`state_stamp-map_stamp` 必须保留有符号值；当前 `max(0,...)` 会把 future map 伪装为 age 0。future 超过小 tolerance、invalid/zero stamp、clock domain mismatch、frame 不在白名单、pose 非 finite、resolution/dimensions/data 不一致、无严格 pair 或 map age 超过既有约 0.20 s 上限时，terrain actuation 应拒绝该 map/plan，并显式记录 `unregistered`/`future_stamp`/`capture_pose_missing` 原因。

由于 publisher cell cache 可旧至 1.5 s，而 map stamp 是当前 tick，必须传 per-cell age/last-observed tick，或至少传 conservative `oldest_observation_tick`/“age unknown”标记。现行 `BuildTerrainModel` 给所有 cell 同一个 model age，无法支撑安全 cell-level clearance。缺少该元数据时只能做 nominal/diagnostic，不能宣称 terrain safety fail-closed；kernel fallback 也不能改称 terrain 认证。

## 最小测试矩阵（纯 synthetic，禁止依赖 sim）

1. 平地垂直瞬变：构造 map stamp=15.476、state=15.532、capture/current base z 分别 0.385326493/0.356020178；断言旧路径约 -7.3 mm，完整 xyz+yaw envelope 的 world surface 约 22 mm，z-only helper 只修高度。
2. 水平平移、纯 yaw、xy+yaw 联合：在 capture map 中放唯一已知世界 cell，验证 registration 后 world cell 不变；旧 current-pose `RotateBaseToWorld` 应产生可测偏差，z-only helper 不可改变该偏差。
3. 时间 history：exact sample、合法 bracket interpolation、无 bracket、gap 超限、future map、HighState stamp=0、clock-domain mismatch；后五者必须不生成 terrain actuation plan。
4. cache freshness：两 cell 使用不同 producer observation tick；验证 envelope 不能将较旧 cell 伪装成 map-wide fresh，缺 per-cell ages 时应 unknown/reject。
5. 语义保持：capture pose registration 后 plan target 在 swing 内不可随 latest map/current pose 重写；WBC 当前测得足端杠杆臂继续来自当前 state，不能换成尚未达到的 target。未来 MPC knot 需单独携带 committed event target/normal，不能让“当前观测”冒充“未来接触预期”。

## 建议的最小闭环与可证伪假设

先做一条不宣称验收的 telemetry canary，记录 map sequence/stamp、capture xyz/yaw、state stamp/current xyz/yaw、registration Δxyz/Δyaw、查询 cell 坐标、cell age、target world xyz 和 plan commit id。假设 H1：补齐 capture z 后，垂直 transient 的 target z 与 nominal/真实地面差值按 Δz 预测收敛；H2：在固定 z、只引入 xy/yaw transient 时，z-only 仍出现 cell/world 偏差，而完整 registration 消除偏差；H3：无 pose pair、future stamp、过期 cell 均被拒绝，不产生“看似安全”的 fallback terrain plan。先用 synthetic 矩阵验证变换与拒绝，再在平地做 pose-perturbation canary；只有 target、IK、接触与原始 CSV 同时显示改善，才可把 frame registration 与 IK 失败建立因果联系。

本稿结论：fa84 代码确实存在 map capture-relative 数据与 current-pose planner 的时间/坐标断裂，run CSV 给出强诊断线索；最小有效修复接口是带严格时间键的 capture xyz+yaw（及 cell freshness）配对，HighState history 仅在 stamp 修复后可用。只改 z 能验证一个窄诊断，却不能满足动态小跑的完整 map registration 或 terrain safety 合同。
