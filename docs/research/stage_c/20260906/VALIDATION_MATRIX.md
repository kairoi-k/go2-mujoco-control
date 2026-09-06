# Stage C 验证、对照与停止条件

所有正式条件继续以固定合同与分析器为准。本文件新增的是开发/诊断验证，不替换 frozen B0/B1，不得凭新表格给旧失败改判。

## 1. 五层结果，禁止互相冒充

| 层次 | 能说明什么 | 不能说明什么 |
|---|---|---|
| 本包16项分析反例 | 算术、源码表达式与合成反例成立 | 生产函数正确、真实输入原因、机器人安全。 |
| 仓库unit/contract tests | 实际接口和核函数满足测试样本 | 实时、原始B0不受干扰、能跨台阶。 |
| 同输入deterministic replay | 算法对照、命令不变性和事件一致性 | 独立wall-clock运行的调度质量、闭环成功。 |
| Atlas shadow / live B0 | 部署、预算和无actuation运行回归 | joint计划在真实执行中可跟踪、B1接受。 |
| B1 canary / holdout | 前者开发诊断，后者完整合同验收 | 硬件、安全认证或sim-to-real成功。 |

H8/H9原始数据尚未在云端独立复算；先在Atlas核对已有原始目录、SHA、hash、运行参数。新补采输入不可标成旧H5丢失坐标。[R02,R03]

## 2. 第一批必须落入实际仓库的fixture

| ID | 输入/操作 | 旧实现需要暴露什么 | 新路径成功条件 |
|---|---|---|---|
| T01 | actuation=off、shadow=on，同一状态/力数据 | world support anchor初始化被屏蔽 | 新输入adapter获得真实且带来源的锚点；默认off命令与额外计算不变。 |
| T02 | float32 0.05m / double 0.05m，patch radius 0.025m | 区域宽度退化/消失 | 使用点候选或真实验证区域；不能加epsilon冒充可行区域。 |
| T03 | 地图全NaN但metadata有效；世界缓存边界内外 | map.valid与有可规划地形混淆 | 类型化原因、观测覆盖明确；不插入假地形。 |
| T04 | 有roll/pitch及非零map age的已知几何 | heading与body frame混用 | 独立坐标真值对照，正确SE3与采样时刻变换。 |
| T05 | 同腿至少两次TD，不同目标/周期 | first-TD重复用于后续周期 | 每次event ID/时间/目标唯一可追踪。 |
| T06 | 20/30ms网格、延迟消费、边界与区间末端 | clamp或缺未来控制区间 | 完整覆盖或明确拒绝，不补最后脚位。 |
| T07 | 当前COM/支撑固定且15mm规则已失败 | 联合求解可能“移动初态”过关 | 明确初态约束冲突；初态不能被solver修饰。 |
| T08 | 同输入多腿候选：greedy不行，联合组合有解 | 逐腿选择非联合可行 | 小规模穷举与C0找到相同可行集合/最优排序，或清楚说明有界搜索漏解。 |
| T09 | 只有一个组合动力学可行，其他几何可行 | 只看support几何或QP状态 | 原始摩擦、力矩/动力学残差与IK/碰撞二次验证。 |
| T10 | 已承诺事件来自旧plan，新proposal继承/修改 | 只比plan ID，或悄悄换目标 | 相同event可继承；目标/时间/轨迹篡改拒绝。 |
| T11 | 新plan发布，MPC尚未刷新 | gait/WBC新、MPC旧 | pending与accepted分离，原子提升一致bundle。 |
| T12 | 计划拒绝、旧plan未过期/已过期两种情况 | 候选失败和执行失效混同 | 不篡改reject统计，不延TTL、不在未知地形盲目fallback。 |
| T13 | 含腾空段的transfer horizon | `min contacts>=2`与该实例冲突 | 保留原分析器FAIL，输出冲突见证；绝不伪造接触/flag。 |
| T14 | Phase1水平恒速与内部xref递推 | supplied reference与真实xref混名 | 三层量独立记录，冻结reference语义不得偷换。 |
| T15 | 变速导致period/duty变化、事件跨重规划 | 独立重算未来schedule破坏承诺 | 唯一调度源、commit prefix与执行完全一致；没有局部摆动重定时。 |
| T16 | 轨迹中途障碍，节点不碰但节点间碰 | 稀疏采样漏撞 | 同一执行曲线的保守连续/自适应验证。 |
| T17 | 预算耗尽、数值失败、候选为空 | 把搜索失败当成物理不可行 | `BudgetExhausted`、`NumericalFailure`、`ObservationUnavailable`分别报告。 |

测试必须调用实际新旧函数，不只在测试内另写一份“正确实现”。所有目标fixture优先小而完整；不得只更改期望值让红灯变绿。

## 3. 最小可回放证据包

优先捕获：首次support rejection前后、第一次publication停止、最后valid plan到期、zero-known出现、target/event mismatch、新旧计划交接、上/下台阶的关键几何窗口。无需把整份巨大CSV提交Git。

每个输入包包含：状态与其真实时间、base姿态/COM/速度/足端、原始/滤波力和三种接触、地图cell值/known mask/采样位姿、Phase1命令与scheduler内部状态、已有承诺、全部有效配置与环境模式、source/binary/simulator/sensor/analyzer hash。NaN必须保留未知语义，不能转换成0高度。

逐规划事件输出：input hash、尝试ID、source time、build wall duration、map epoch、逐事件候选数及拒绝类别、search剪枝/终止原因、knot/接触mask/margin分解、原模型残差、publishable、publication/acceptance、执行版本及失效原因。至少保留一个足以独立重算的完整失败输入。

旧`_runs`只读。新的精选证据写到`docs/research/evidence/stage_c_<batch>/`，附命令、JSON结果、hash和README；大文件留Atlas，以hash和路径索引。[R03]

## 4. 消融与公平比较

在**相同输入包、相同初始化、相同候选集合及同一硬约束**上比较：

- L0：原输入adapter + 旧planner，复现原行为。
- L1：规范化输入adapter + 旧planner，隔离frame/锚点/地图差异。
- C0：规范化输入 + 联合规划，隔离真正算法增益。
- C0-ablation：同预算单个greedy组合；小型穷举oracle用来衡量beam漏解。

如果L1已经解决大部分问题，必须如实记录，不能把这部分收益归因于joint。传感器修改另有独立提交与数据包，不能与算法对照混在同一个曲线里。优化器不看holdout的具体障碍坐标、seed结果或场景几何标签。

## 5. 指标与分母

同时报告按planner尝试、按控制tick、按时间加权的三组结果。

必须区分：观测不可用、初始状态/冻结规则冲突、候选为空、搜索预算不足、求解器失败、整体可行性拒绝、期限超时、已发布未接纳、计划过期、承诺不一致。未调用/未检查不是PASS。

至少记录：全部active窗口有效覆盖率；预先定义的输入有效窗口条件覆盖率；最大连续no-plan时长；每次失败原因；事件覆盖/身份错误数；所有规划尝试p50/p95/max延迟与miss；命令语义不变性；真实执行时的tracking、碰撞、支撑与稳定退出。

输入有效窗口的定义必须先固定并单独显示被排除样本，不能事后删掉地图坏、姿态差、plan invalid或发生碰撞的样本，最后宣布覆盖率100%。优化器应可以正确拒绝真正无解输入；无解释的拒绝和有证据的不可行是不同结果。

## 6. Command invariance 的最小有效证明

独立两次lockstep仿真可能收到不同初始状态，不能证明shadow不改变命令。需要同输入重放：相同Low/HighState序列、地图事件、外部命令、时间增量、初始积分器/滤波器/scheduler/WBC cache、随机种子及环境配置，分别关闭/开启shadow。

比较每个tick实际LowCmd字段、关节目标、力矩、参考和停止状态。优先固定平台/编译设置下逐字段精确一致；不要比较含未定义padding的整块内存。确有浮点非确定性时，先定位来源、预定义理由充分的误差界，不能观察差异后随意放宽。

该回放证明的是语义不变。实时调度干扰仍需独立Atlas wall-clock B0；不能因为离线命令相等就省略live回归。

## 7. 分阶段放行与停止

### P0：输入/合同就绪

完成T01–T07、T13–T15所需最小fixture；明确数据缺失和合同冲突；建立同输入回放接口。没有完整旧H5坐标不必停止全部工作，可以使用新捕获样本继续，但必须标明新证据。

如果发现冻结合同在合法固定初始状态下无解，或transfer条件与实际日程冲突，停止该验收路径并提出最小修订议题。可以继续不改变合同的算法/接口离线实现，不允许“先让B1过了再解释”。

### P1：纯C0内核

目标：真实多事件联合选择，完整状态/力rollout，可解释地返回结果；小型穷举oracle一致性、预算耗尽与无解输入测试通过。旧planner保留为对照。不是只有schema/影子validator，也不是重新做一个大而全框架。

### P2：回放与影子集成

在flat及5cm开发输入中，明确说明C0相对于L1改善哪些拒绝、仍失败在哪里。承诺、事件和完整时域在准备接入执行的窗口没有已知不一致；同输入command invariance成立；Atlas预算得到实测。不要只报checked子集全绿。

有一个长期无解/超预算主因已经被证实，就在此停止扩展，给出证据与下一最小决策。不得为追求覆盖率替换未知地形或延长失效计划。

### P3：第一次B1开发闭环

新路径默认关闭仍通过全测试/构建，代表性B0配对完整PASS；C0的输入、执行和安全条件满足；没有未裁决的冻结合同矛盾，才开启一次仿真B1 development canary。使用同一exact clean candidate，场景seed遵守manifest，不读取holdout调参。

这次canary尽早进行，不等待几周shadow；其价值是验证真实落脚/身体/受力跟踪。首次有信息失败即保存输入和执行轨迹，按计划/执行/感知/控制模型分类，不回到总rejection数调参。

### P4：正式验收

只有稳定候选才运行同一exact SHA的fresh full frozen B0全部规定profile/repeats，之后全部frozen B1 holdout；两者全部PASS才更新accepted。开发固定3m/s pair只是代表性开发保护，不能代替full B0；单边terrain PASS不等于pair PASS。[R04,R05]

### 通用停止条件

连续三个不同且有信息的试验仍卡同一blocker；需要改冻结门槛/统计语义/传感器权限；需要跨出C0做拓扑切换或全身NMPC；无法取得可信输入/来源；已知破坏默认Phase1/B0；或需要硬件动作。保存证据、更新CURRENT与milestone后停止，不强求PASS。

单测代码笔误、编译修复不自动算一个“算法假设”。没有必要每改一行都跑full B0；便宜fixture先淘汰坏候选。
