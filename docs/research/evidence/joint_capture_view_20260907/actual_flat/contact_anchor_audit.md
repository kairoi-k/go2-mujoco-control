# joint_capture_flat_20260907_0001：四个 initial_contact_anchor_unavailable 案例

原始 run：`.../example/cpp/experiments/_runs/joint_capture_flat_20260907_0001`；`run_metadata.txt`/`run_manifest.json` 均为 `git_head=0ff9dc8...`、`git_dirty=false`，argv 为 `--period 0.14 --duty 0.44 --gait-pattern running-trot`。使用 `data.csv.state_tick_s` 与 `contact_ground_truth.csv.time_s` 作为同一秒单位的离线 join；`cmd_time_s` 是另一列，未用于 join。data 的 state 秒来源是 `trot_experiment_diagnostics.cpp:653-654`（LowState tick×0.001）。

腿位/位掩码顺序均为 FR,FL,RR,RL。源 `gait/locomotion_kernel.h:91-101,120-142` 给 running-trot 默认 offset=.46，`GaitLegScheduledStance` 为 phase<duty；四个精确 phase 均推出计划支撑 `0110`（FL/RR）。这是 gait schedule 推导值，不是 terrain execution telemetry；该四案 `terrain_execution_planned_contact_mask=0` 因 plan 不可用。`trot_experiment_wbc.cpp:201-215,1330-1344` 也表明 wbc_* measured/scheduled/planned 字段只在 `terrain_plan_active` 时写入，四案全为 0，不能当真实测量 mask。

|t(s)|data行/hash|GT行/hash|terrain plan|gait计划|direct测量|GT touch|精确GT脚力(N)|±10ms GT最大脚力(N)|判定|
|---|---|---|---|---|---|---|---|---|---|
|20.548|9290 / 0533965a424d5e05|10275 / bb1fba572255a0f9|failure=4, applied=0|0110|0010|0010|FR0 FL0 RR132.829 RL0|FR3.329 FL150.484 RR171.789 RL4.278|FL在精确采样点确实未接触，不是低于5N；+2ms 已有150N，属于 touchdown 边界时刻。|
|22.086|10059 / 2ac35eaa13a5c7d1|11044 / dca5628ea85f52df|failure=4, applied=0|0110|0010|0010|FR0 FL0 RR8.717 RL0|FR8.294 FL0 RR126.224 RL0|RR是真接触且高于5N；计划FL在精确点及±10ms没有GT touch，不能解释为measurement threshold或join偏移。|
|23.626|10830 / 88ff2458f3283b54|11814 / f8feda24949c8241|failure=4, applied=0|0110|0000|0000|四腿0|FR11.023 FL0 RR0 RL0|计划FL/RR在精确点与±10ms均无接触，是真实无支撑/空中段；不是5N阈值造成。|
|25.164|11599 / aea7989ebd88fd53|12583 / d21eb20cff492894|failure=4, applied=0|0110|0000|0000|四腿0|FR7.578 FL97.001 RR115.143 RL11.876|精确点是四腿无接触；±10ms两组对角支撑均出现，符合低 duty/offset 交接的实际空中间隔与触地延迟，不显示时钟错配。|

GT touch 定义为每 leg `*_touch_N>0`；`phase2_terrain_foot_contact_mask=0` 是“terrain”接触子掩码，平地 run 中为 0，不代表没有地面接触。production diagnostics 的在线 `contact_*` 由 `foot_force >= kContactForceThreshold`（`trot_experiment_diagnostics.cpp:677-688`、`trot_types.h:96`，阈值5N）；因此 22.086 的 RR=8.717 已应在线测得。控制快照同样用直接5N比较（`trot_experiment_control.cpp:191-205`）。

四条 `controller.log` 记录分别为行130/173/225/268，均 `failure=observation_unavailable detail=initial_contact_anchor_unavailable`，对应 id 359/389/419/449；物理行hash（去换行）依次为 `90e18386995eb02e`、`64d8000f2553a4d0`、`8e6ee9d5ceb595fd`、`c64fc5703415a20e`。日志中的 anchor_gap_m 分别 .000348、.002658、0、0；无 solver 调用。

结论限定为四案离线事实：20.548 是2ms触地边界，22.086 是计划FL真实缺失而RR仍接触，23.626 是计划双支撑真实缺失，25.164 是精确采样落在对角交接空中段。没有证据把任何一案归因于 data/GT 时钟错位；也没有 per-leg online planned mask，因为 terrain plan 全部 failure=4。无需本表外推整段运行。