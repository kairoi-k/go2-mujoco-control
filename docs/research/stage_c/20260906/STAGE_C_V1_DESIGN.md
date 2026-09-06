# Stage C v1：最小联合规划与执行契约

**设计状态：可实施的 C0 方案；基于固定源码的设计建议，不是已验证生产实现。**  
基线：`f3b452d56b2bedd5ea02249d4e5087b6ca151c47`。权威规则仍见仓库 CURRENT / AGENTS / PHASE2_ACCEPTANCE。来源索引见 SOURCES.md。

## 1. 目标与明确不做的事

C0 联合选择多个未来落脚事件的地形候选，并求出与这些候选一致的质心/机身状态及接触力序列，最终让 gait、SRBD-MPC、ID-WBC 消费一个被正式接纳的执行版本。C0 的交付必须包括完整事件与动力学预测，不能只是给旧逐腿结果增加几个一致性标签。

第一版固定 running-trot 拓扑，固定现行 Phase1 时序预览；**不优化 touchdown 时间和 duty，不引入局部 swing retiming**。这是降低第一版未知量的 C0 切片，不是永久放弃 Stage C 的联合时序目标。只有 C0 得到明确信息、且固定时序被证明是必要瓶颈时，才评审 C1 的同拓扑有界时序优化。[R02,R03]

不改 B0 默认控制数学，不驱动真机，不实现全身 NMPC、learning、scripted crawl、三脚预加载或停车换腿。旧 per-leg scorer 保留为候选排序与对照，不作为未知地形上绕过安全检查的 fallback。

## 2. 模块边界：少一个“各管一份状态”的机会

```text
已允许的状态/足力/地形观测 + Phase1 command/schedule
                    |
         TerrainPlanningInput（完整不可变输入）
                    |
   TerrainBelief/候选生成 —— 只读几何缓存与显式观测覆盖
                    |
      JointPlannerC0：事件组合 + 动力学子问题
                    |
      PlanCandidate + FeasibilityCertificate
                    |
 ExecutionManager：核对承诺/时间/新控制解 → 原子接纳
                    |
      AcceptedExecutionBundle（同一执行版本）
         /              |                 \
  gait 足端轨迹       SRBD-MPC           ID-WBC
```

生产与 shadow 使用同一个纯规划核。shadow 输出只能进入诊断/影子执行管理器，不能改变实际命令。shadow 输入的传感器采集不可被 `terrain_actuation` guard 错误屏蔽；模式开关只控制执行，不控制事实的可观测性。[R07–R10]

## 3. 输入 schema：先把物理量写清楚

下列类型名为拟新增接口，不声称当前仓库已经存在。

`TerrainPlanningInput` 至少包含：

| 字段组 | 必要内容与不变量 |
|---|---|
| 身份 | input hash、估计器版本、源状态 tick、完整配置/model/sensor hash；构建时刻与来源时刻分开。 |
| 身体状态 | world frame 中 base pose、model COM、COM velocity、姿态/角速度、质量与惯量；base velocity 与 COM velocity 不混名。每项附有效性/时间来源。 |
| 足端状态 | 每腿真实 body FK 与 world foot-site、接触 patch 坐标，力与滤波接触；统一 foot-site/patch 转换函数，不能遗漏半径或重复加偏移。 |
| 接触 | measured、planned、applied 三组来源；applied 是现行控制权威产生的当前集合，不能由 planner 为过门槛改写。 |
| 地图 | acquisition time/pose、heading frame 定义、cell resolution/extent、每cell观测有效性与年龄、可辨认的 surface/region IDs。结构valid与known coverage分别记录。 |
| 命令 | Phase1唯一速度权威的 command epoch、shaped/applied值、现有 nominal reference 生成参数和 gait schedule epoch。不让planner读取测试脚本未来“答案”。 |
| 执行承诺 | 已经接纳的事件ID、腿、落脚时间、目标、轨迹、有效窗口、source plan ID，及哪些支撑已经实测发生。 |
| 预算 | 预测覆盖区间、源状态最大可接纳年龄、求解迭代/候选预算、独立的wall-clock deadline。 |

时间优先使用有单位的 int64 tick/ns 包装，避免把 simulation seconds、wall-clock duration和gait phase混在同一个double里。处理原LowState tick环绕与重复；缺失的HighState时间同步不能靠复制LowState tick假装解决。

### 3.1 地图与FK的坐标变换

传感器当前发布的是heading对齐的局部XY与相对base高度，不应自动视为倾斜body-frame。[R11,R13]

若一格观测 `p_M=(x_M,y_M,h_M)` 采集于 `t_m`，按其真实传感器定义先变到世界：

`p_W = p_base_W(t_m) + Rz(yaw(t_m)) * (x_M,y_M,0) + (0,0,h_M)`。

body FK足端则使用完整姿态：`p_foot_W = p_base_W(t_s) + R_WB(t_s) * p_foot_B`。需要在当前body或map中查询时，再用相应逆变换；不能对不同frame共用yaw-only函数。

C0首先对已有观测重投影并保留来源，不增加无证据的未知区域填补。测得的支撑锚点只能证明有界的接触邻域，不能外推出下一块台阶。地形粗糙度、空间邻域方差与估计器协方差也不能当成同一个“sigma”。不确定性模型未标定时，不得宣称概率安全保证。

## 4. 事件与预测时域

### 4.1 事件身份

事件 ID 采用 `(schedule_epoch, leg, cycle/event_sequence)`；一个 plan 可以携带多个周期中同一条腿的不同 touchdown。source plan ID仅记录来源，重规划继承同一个承诺时不要求source ID变成最新proposal ID。[R09]

接触边界用显式事件时间表达，不由“每20ms是否触地”的格点差分充当真实落地时刻。格点用于动力学采样，事件用于切换；采样可能跨过一个事件，必须按区间处理，不能丢失或重复计数。

“固定时序”只表示C0不自由优化日程，绝不意味着整个变速过程period不变。日程必须由唯一全局调度权威给出，未承诺的未来事件可随现行Phase1调度更新；已经被accepted bundle承诺的前缀必须由同一权威执行。不能一边让gait自行改变period，一边把旧的TD绝对时间称为承诺。如果现有scheduler无法提供这项接口，先实现并验证统一的preview/commit边界，默认关闭时保持旧行为；不得在某条腿的consumer中偷偷冻结或拉长swing。明确记录这属于执行接口改造，不把它藏成索引修复。

### 4.2 覆盖、freshness与执行窗口分开

规定节点 `t_0...t_N` 表示状态，区间 `[t_k,t_{k+1})` 表示接触/力输入。若消费者有N_m个控制区间，则规划要覆盖**区间末端**，不只覆盖最后区间起点。

必要条件：

`T_prediction >= A_max + T_consumer_intervals + T_guard`。

`A_max` 来自允许的状态/地图年龄，以及成功构建链的源状态→队列→构建→发布→接纳测量，不能使用“长期没有新可行解导致旧计划已经很老”倒推出巨大允许延迟。`T_guard`由接纳/消费者刷新边界定义并测试，不凭感觉取值。

接纳截止应不晚于：状态有效期、地图支持证据有效期、预测末端减去消费者需求，以及承诺安全窗口中的最早者。增加节点数不能自动增加这些freshness上限。[R08；A04]

力学网格必须覆盖事件边界；可以使用事件对齐的非均匀网格，或先以固定网格实现并在事件边界增加节点。适配现有8步MPC时按绝对时间重采样连续状态/力；接触布尔量按事件区间查询，禁止插值或末节点复制。存储上限、solver节点上限、消费者步数分别命名，不把`48`一处改动传播成所有QP规模翻倍。

## 5. 联合规划问题

### 5.1 变量

对每个**未承诺**的未来落地事件 `e`，生成经过观测及硬几何初筛的有限候选集合 `C_e`，选择 `z_e ∈ C_e`，得到 world patch位置 `p_e`。已承诺事件的位置、时间和正在执行的轨迹固定。

连续变量至少包括各节点的 `c_k, v_k, orientation_k, angular_velocity/momentum_k` 和各接触区间的 `f_i,k`。第一版不让优化器自由移动x0，不引入动作脚本。各腿的落脚选择通过整段身体/受力可行性和代价耦合，不能逐腿选完后完全不回溯。

### 5.2 物理模型与近似边界

物理核关系是：

```text
c_dot = v
m * v_dot = sum_i f_i + m*g
L_dot = sum_i (p_i - c) cross f_i
R_dot = R * skew(omega_body)
```

采用SRBD时惯量与肢体角动量贡献作近似。不能把这个近似模型称为完整全身动力学；rollout的坐标/角速度约定必须与所用惯量一致。

当前代码已有12状态、接触力为变量的SRBD凝聚QP，可作为参考代数与回归基线。[R14] C0可在独立纯函数实现中使用状态显式或凝聚形式，但必须返回**整段**预测和力，而不只是第一步。

对离散落点固定后的连续问题，角动量项仍可能对 `c×f` 双线性。可围绕上一份有效rollout线性化：

`(p-c)×f ≈ (p_bar-c_bar)×f + ((p-p_bar)-(c-c_bar))×f_bar`。

C0的候选点离散固定时 `p-p_bar` 是已知量。以有限次SCP更新参考，再用未线性化的关系检查残差；不能只看到QP solver成功就发布。取一步还是两步、trust region与残差容限须先在单位测试/离线误差分析中定义，记录配置hash，不能观察B1结果后随意放宽。

初始实现应保留现有SRBD离散化作对照。若改成含半步加速度的位置积分，属于新的模型实现，必须单独验证；不能一边改积分器一边把收益归因于joint选点。[R14]

### 5.3 硬约束

- 输入初值与已发生接触是事实；未承诺落点才是决策。
- 力：非接触力为零、接触法向力与摩擦限制、有限解、既有执行层的力/扭矩安全边界。地形法向需统一frame。未来`J^T f`估计只能是代理检查，不能冒充完整动态扭矩保证。
- 足端：落点必须在已观察且足底patch通过检查的区域/中心，保留edge、roughness、slope、local patch step、IK margin、clearance等冻结门槛。
- 身体与腿：基于同一预测身体姿态检查腿长/IK、足端扫掠、膝/小腿和机身碰撞，不能只检查落点高度。对不确定或未覆盖空间显式拒绝。
- 承诺：新计划必须继承正在执行的事件，不得为了寻找可行解暗改落脚点、TD时间或当前支撑。
- 时域：完整覆盖消费者需求；缺失未来事件、观测或状态不能用重复最后脚位填充。
- 保留当前15mm几何支撑门，并另做动力学残差/受力检查。它们是两个条件，不互相替代。[R04,R06]

两脚固定端点a,b时，令单位切向e、法向n、线长l，所需几何margin为 `r=max(0.015,原定义的不确定性膨胀要求)`。可在QP加入等价的线性带约束：

`r <= e·(c_xy-a) <= l-r`，`|n·(c_xy-a)| <= 0.040-r`。

这只实现冻结几何规则；并不说明该规则足以保证动态稳定。若r>0.040，带为空应显式拒绝；若当前knot0已违反该约束，不能平移初始COM。三/四点支撑使用凸包边半空间；单脚与腾空严格按已审定的当前规则处理，另外检查实际分析器的transfer要求。[R06,R15]

### 5.4 目标函数与唯一速度权威

形式上求：

`min_(z,X,F) sum_k ||x_k-r_Phase1,k||_Q^2 + ||f_k||_R^2 + foothold_cost(z) + commitment_change_cost`。

硬约束不能通过目标权重换取违反。`Q/R`先使用既有SRBD的量纲明确参考值；新的离散项和搜索优先级在离线fixture前固定。已锁定承诺的变化是禁止项，不是低罚分项。

必须分开三个量：

1. `command_v_xy`：Phase1给出的唯一目标速度。
2. `solver_reference`：真正进入QP的参考，包含原solver既有的位置推进规则。
3. `predicted_state_trajectory`：优化器根据力与接触预测的身体运动，它不等于新的外部命令。[R14]

C0不得增加独立水平速度指令，也不能“把参考藏成预测”再用于执行。第一版保持Phase1水平reference生成语义；联合求解预测身体状态与落脚力学。地形导出的垂直/姿态参考只有在规划、MPC、WBC数据来源和限制都被明确测试后才能进入执行。若需要新的水平body-reference自由度才能解题，先提交与≤1mm条款的具体冲突，不得自行改metric。[R04,R15]

### 5.5 候选生成：第一版用点，不用伪凸区域

当前safe region的宽度几乎为零。[R12；A01] C0直接使用已经验证的中心点作为离散候选；为每个未来事件按该时刻的body可达范围生成，而不是只按当前body位置过滤所有未来落点。未来body尚待优化时，先用保守的body trust-region包络做候选预筛，再用最终轨迹严格复核，避免预筛误杀。

保留候选多样性：名义附近、不同观察表面、不同边缘余量。不能全部保留到同一格，也不能读取台阶编号或holdout坐标。候选被截断与真正没有候选分别报告。未来若增加连续区域，必须用有观测支持的patch并集做几何收缩并验证边界，绝不对half加一个人为小正数。

## 6. 求解算法：可测的有限联合搜索

建议两个共用相同可行性定义的求解模式：

**离线参考模式**：对小型真实/合成输入枚举候选组合，求解连续子问题，用于验证搜索、排名与拒绝原因。它不是线上实时基线。

**线上候选模式**：按未来接触事件组做有界beam/branch搜索，保留上一可行解及若干不同表面的候选序列；完整叶节点调用连续子问题，联合动态代价决定选择。不能只对greedy结果调用一次QP，然后把它命名为joint。

```text
validate input; preserve immutable commitments
build observed candidate sets per event
construct a bounded collection of multi-event combinations
for each retained complete combination, in deterministic order:
    solve bounded continuous SRBD/body-force subproblem
    verify original nonlinear residuals and exact safety checks
select a fully feasible solution using the frozen ranking rule
return candidate + certificate, or a typed failure (not a fabricated plan)
```

初次benchmark可从每事件最多4个候选、beam宽4、最多4个完整QP子问题的开发配置起步；这些是**计算预算试验参数，不是已经证明最优的常量**。可行性检查、有效事件数、候选深度、SCP步数和总deadline都计入成本。若超过预算，返回`BudgetExhausted/SearchIncomplete`，不能记为物理不可行。减少候选预算的效果必须与离线穷举比较，不能只展示速度。

求解器选择：先把C0内核接在独立solver接口上，用现有DenseQP做参考；若实测多候选预算不达标，再在新路径比较OSQP固定结构实现，保持原B0后端不动。OSQP支持固定维度/稀疏模式的生成C代码，并不免除精度、收敛与Atlas时限验证。[S01] 此时不新写一个复杂通用SQP库；完整acados/whole-body NMPC保留为后续选项。[S02]

## 7. 执行接纳：不能让“最新尝试”支配实际电机

`PlanCandidate` 不等于 `AcceptedExecutionBundle`。新的candidate可能被拒绝或未完成MPC适配，而旧accepted bundle仍合法；两者状态和统计分开。这样不抹掉任何required-plan rejection，只避免候选失败直接把真实足端轨迹换成另一套行为。

一个accepted bundle至少包含：

- execution version、input/state/model/command/schedule/map身份；
- 完整事件时间轴、逐事件足端目标/轨迹及已锁定承诺；
- 身体预测、真正用于控制的参考、力/加速度解和对应时间区间；
- 当前applied support与未来planned contacts及其来源；
- 几何/受力/覆盖/承诺证书和接纳、执行有效窗口。

**原子接纳条件**：candidate与当前承诺兼容；状态与地图仍可用；MPC适配/解已与该候选一致；当前MPC k0与WBC施加支持一致。只有全部满足才发布给gait/WBC。不能只让gait切新plan，而继续用另一个plan ID的缓存MPC力解。[R10]

ID不同不必然错误：旧source plan的同一event可被新proposal继承。目标/时刻/轨迹不变且证书仍有效才可继承；纯粹把ID改成相同不算修复。

接触事实可能与计划不同：触地提前/延后、滤波滞后应由唯一现行接触权威解释，不能每个消费者自建政策。第一版不改变该政策，不直接把实测15替换计划9。未来若需要统一接触融合改造，作为独立结构变更和回归范围说明。

### 7.1 足端路径必须是同一条

planner检查的swing曲线应与gait真正执行的曲线一致。C0固定TD时刻，按已测/已承诺的起始位置、速度、加速度构造边界连续的曲线；可用分段quintic和地形决定的抬脚高度，但不得由gait临时改变摆动时间或在新plan到来时跳目标。验证器检查同一曲线的速度/加速度、连续扫掠与足底/小腿碰撞。

离散采样检查不自动保证采样间安全。使用几何/曲率界、自适应细分或保守体积包络，输出检查分辨率与剩余误差界；不能靠把采样步长变粗减少“碰撞数”。

### 7.2 无计划和失效处理

shadow失败只影响shadow诊断。实际地形执行时，尚在已验证窗口内的已承诺动作可以按既定安全语义继续；不得延长失效计划、杜撰新落点或在未知台阶上静默切回blind Phase1。无法保证现有动作继续安全时使用已有全局安全机制并把本次B1记FAIL。安全停止是安全行为，不是验收成功。

## 8. 实时性：把计算能力当成待测量，不当成口号

冻结目标仍是planner p95≤2ms、hard≤5ms、misses=0。[R04] 发布频率不等于单次耗时，lockstep仿真步暂停也不等于实时达标。

分别记录：地图预计算wall time、输入排队时间、候选生成、连续QP、原始约束复核、构建总时间、发布时间、消费者source age、deadline miss。若把几何预处理缓存到map epoch，仍报告全路径成本和freshness，不能只统计QP子过程伪称满足原Build预算。

规划worker不得为提高自身得分重新默认绑到影响writer的CPU；Atlas实际亲和性、优先级与并发负载留证。预分配数组/矩阵、固定排序、固定可重现迭代上限；目标版本只在预算可测后进入live运行。需要超出冻结预算或大规模替换算法时明确停止评审，不把限制偷偷移到另外一个线程/字段。

## 9. 代码实施映射

以下均为拟新增文件/责任，不移动或覆盖旧证据。

| 模块 | 拟新增位置 | 复用/改造边界 |
|---|---|---|
| typed input / event / bundle | `example/cpp/terrain/stage_c/types.h` | 复用现有几何类型，显式区分frame与时间；不用所有bool默认true。 |
| belief/input adapter | `.../stage_c/input_adapter.*` | 对接R07、model COM与地图采样姿态；shadow不屏蔽测量。 |
| event candidate provider | `.../stage_c/candidates.*` | 复用TerrainModel/硬检查，以中心点开始；旧scorer只排序。 |
| continuous solver | `.../stage_c/centroidal_subproblem.*` | 独立于旧SRBD控制路径，返回全rollout/force/residual。 |
| joint planner | `.../stage_c/joint_planner.*` | bounded combinations + continuous solve；记录search budget与真实原因。 |
| independent verifier | `.../stage_c/verify_plan.*` | 原始模型/几何复核；不能只相信solver内的线性化可行性。 |
| execution manager | `.../stage_c/execution_manager.*` | 统一pending/accepted/commitment；适配gait/MPC/WBC而非三份新FSM。 |
| replay/test fixtures | `example/cpp/tests/`与新的分析工具 | 完整输入和初始化状态，原始命令流只读；CMake新测试target。 |
| durable evidence | `docs/research/evidence/stage_c_.../` | manifest、短窗口输入/结果、哈希与命令；禁止提交/修改旧`_runs`。 |

## 10. 冻结设计决策与仍需Atlas裁决的事项

本轮确定：先C0、固定现有timing；点候选联合选择；显式身体/力rollout；严格区分freshness/coverage；单一accepted bundle；保持未知与冻结门槛；默认shadow关闭；原始B0控制路径保留。

Atlas尚需量测而非凭空指定：H9无锚点与零地图实际影响、可行初始状态/transfer合同冲突、合理source age、候选数和SCP步数的时间预算、模型残差/几何插值误差界、command invariance。新agent可在这些事项上做既定fixture和小型benchmark，不需要每步询问；但修改冻结含义、拓扑或硬件执行权限必须重新授权。

这些未知不妨碍现在实现纯C0核和接口测试，也不授权在已知安全/合同矛盾下冲B1。后续门槛和具体停止条件见VALIDATION_MATRIX与CODEX_HANDOFF。
