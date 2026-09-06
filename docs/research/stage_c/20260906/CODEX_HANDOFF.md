# 给 Atlas 新 coding agent 的完整任务书

你接手 Go2 Stage C。阅读本包全部文件后自主工作，不在“列计划”处结束，不需要每个小步骤等待确认；真实缺失权限、冻结合同矛盾、明确停止条件除外。先做可检验的最小实现，不为通过B1倒推修改指标。本轮仅限仿真，禁止连接或驱动真机。

## 一、精确起点与权威

仓库：`kairoi-k/go2-mujoco-control`。
固定起点：`f3b452d56b2bedd5ea02249d4e5087b6ca151c47`。
本包审计时，该SHA在`investigate/phase2-b0-wallclock-telemetry-20260905`；`fix/phase2-b0-runtime-integrity`落后，不能凭名字选它。
建议新分支：`feat/stage-c-joint-planner`；独立worktree，不修改main、不reset现有worktree、不处理别的agent未提交修改。

先检查远端、对象、工作树和已有worktree。以下是路径示例，发现已存在分支/目录时只读核对，禁止覆盖：

```bash
repo=/home/che/dev/go2-workspace/current
base=f3b452d56b2bedd5ea02249d4e5087b6ca151c47
git -C "$repo" status --short
git -C "$repo" worktree list
git -C "$repo" fetch origin investigate/phase2-b0-wallclock-telemetry-20260905
git -C "$repo" cat-file -e "$base^{commit}"
# 确认新分支与目录不存在后再执行：
git -C "$repo" worktree add -b feat/stage-c-joint-planner \
  /home/che/dev/go2-workspace/stage-c-joint-planner "$base"
```

先读仓库CURRENT、AGENTS、PHASE2_ACCEPTANCE、PHASE2_HOLDOUT_MANIFEST、RESEARCH_HISTORY/INDEX，再读本包AUDIT_AND_RESEARCH、STAGE_C_V1_DESIGN、VALIDATION_MATRIX。以实际源码检查设计，不默认外部设计正确；发现反例直接留证修正设计。更新现有canonical入口，不创建第二套主线状态。

`_runs`是被忽略的本地证据，只读，不删除/重命名/覆盖/整体提交。精选证据进入新的`docs/research/evidence/stage_c_<batch>/`，保留原路径、命令、source/binary/input hashes。

## 二、目标

实现最小 **C0：多个未来落脚事件的有限候选联合选择 + 连续身体/质心/接触力预测 + 单一执行版本与承诺管理**。固定running-trot拓扑；第一版不优化接触时间、不改Phase1唯一水平速度权威，不做全身NMPC、learning或scripted crawl。

这不是继续围绕旧planner总rejection数调参，也不是只写一个新schema。纯规划核必须真的联合比较多个事件组合，用整体动力学/几何可行性和成本选择，并返回完整rollout、事件、力与可解释失败。

默认关闭新路径；先离线/replay/shadow。达到下述gate后尽早允许一次B1 development canary，不无限停留在shadow。不能凭“构建+CTest通过”宣称B1完成。

## 三、必须带着反证核对的事实

H9只在报告层归因了planner拒绝→旧计划到期；不证明全部根因是贪心算法。尤其核对：

1. shadow actuation-off时，force-backed anchor仍被actuation guard屏蔽；新旧planner的输入要相同。
2. 5cm grid与2.5cm patch radius使safe region half几乎为零；C0先用已验证中心点，禁止对half加epsilon冒充安全区域。
3. 模拟世界缓存约x[-2,20)、y[-2,2)，zero-known与map metadata valid可并存；查真实轨迹是否出界，不能猜。
4. 当前body/COM整段复制、每腿只取第一次TD，不是完整多事件预测；增加knots不能修正这个问题。
5. code4=0经过readiness/commitment预筛，不能用checked子集通过率证明全管线。
6. 冻结分析器要求transfer horizon最少2接触；包含腾空段的该类样本会失败。水平reference≤1mm还需区分supplied anchor、真正solver reference和预测状态。禁止改flag/字段来躲检查。

本包分析脚本16/16仅是独立反例；需要实际仓库函数测试。不要把本包脚本结果写成你的生产CTest或Atlas实验。

## 四、执行顺序与提交边界

### C0-01：就绪fixture、输入包、合同映射

把T01–T07、T13–T15最小见证落到实际仓库测试。核对传感器/状态frame与来源、锚点采集模式、区域退化、knot0初态和transfer/reference字段。

优先只读现有H8/H9 raw evidence，生成少量完整输入fixture。缺失逐腿坐标时允许最小补采，明确是新数据，绝不反推旧H5坐标。冻结新的开发输入集合及hash，作为新旧方法统一对照。只补对实现有必要的telemetry，不开启一轮泛化日志工程。

若发现输入adapter缺陷，在新Stage C默认关闭的adapter中做最小修正并留对照；如果确需模拟传感器滚动缓存修复，另一个提交、独立hash与回归，不偷偷改地图边界适配场景。

合同冲突输出“原条件→具体输入→最小反例→需要的决策”，不自行修冻结文件。这个问题不阻止无冲突的纯C0代码继续开发，但阻止对应actuation/验收路径。

### C0-02：纯联合规划核

按STAGE_C_V1_DESIGN实现：typed完整输入、显式事件表、离散候选组合、连续SRBD子问题、完整body/force rollout、独立几何/原模型验证器。

每个未来TD都有独立身份，已承诺前缀固定；使用唯一scheduler的preview/commit接口，不能planner按旧period算时间、gait再自行改变它。C0不优化时间，不等于整个变速过程强行保持period不变。

先做小规模穷举离线reference和确定性的有界搜索。明确`ObservationUnavailable`、`InitialConditionConflict`、`NoFeasibleCandidateInSet`、`SearchIncomplete/BudgetExhausted`、`NumericalFailure`等，不把beam漏解叫物理无解。

保留旧SRBD/ID-WBC默认实现；新solver接口返回全预测。QP线性化后的可行性还需原始残差/几何复核，不能以success标志替代证书。候选数、beam/SCP预算先定义再测，不调阈值换成功。

### C0-03：公平对照与实时预算

同一输入比较L0原输入旧法、L1规范化输入旧法、C0联合方法；对small fixture与穷举oracle比较。说明收益来自输入修复还是joint，不用不同场景、不同初态或不同候选集合拼对比。

在Atlas记录整个规划链各阶段wall time、排队和source age；满足冻结p95≤2ms/hard≤5ms不能只计QP，不能把计算移到计时器外。当前DenseQP若成为瓶颈，才在新路径做OSQP固定结构后端对照；不先重写整个优化生态。

### C0-04：单一执行版本与shadow回放

实现pending proposal与accepted bundle分离，统一event/commitment，验证新计划与MPC解一致后再原子接纳。ID是来源，不替代目标/时间/轨迹一致性检查。

shadow使用同一规划核但不能改实际命令。构建同输入、同初始化的command replay，对比LowCmd字段、关节目标、力矩与参考；不要用两次起始状态不同的lockstep当不变性证明。另做live B0保护实时行为。

发布、接纳、消费和过期各自留证；不延长过期计划，不复制末节点，不把当前脚位补作未知未来落点，不在未知台阶上切blind fallback。

### C0-05：早期闭环与正式验收边界

实际全测试和build通过，新路径off保持B0；输入/执行/覆盖条件已在目标窗口证明，无未裁决安全或合同矛盾，同输入command invariance成立，代表性完整paired B0在同exact clean SHA通过：可直接跑一次冻结开发场景的B1 canary，不必额外等每一步确认。

首个信息性失败保存完整输入与控制输出，定位是感知、候选、优化、执行承诺、模型还是跟踪。没有必要每次改代码都跑full B0；正式准备宣称B1 accepted时，才在同一个exact candidate SHA上依次跑fresh full frozen B0和全部B1 holdout。任一失败都不能宣称accepted。

若目标仅完成C0 core就触发下面停止条件，交付已验证部分和证据，不补造后续实验。

## 五、禁止事项

不改15mm、edge/IK/clearance、latency或验收分母；不把planned contact强行设为measured；不把初始COM挪到多边形里；不在转换flag/诊断字段中隐藏失败；不读场景XML、geom identity、障碍坐标、step index或ground-truth给controller/planner；不复活crawl/三脚entry/低姿态/停车换腿；不为单腿增加私有恢复/retiming；不整套替换MPC/WBC；不改main或旧证据。

## 六、停止与报告

同一blocker连续三个有信息的不同试验失败；需要改冻结含义/拓扑/whole-body架构；出现已知Phase1/B0保护失败；输入或来源不可验证；预算不达标且只能放宽门槛；需要真机动作——停止并报告。编译笔误修复不算独立算法试验。

阶段性提交clean代码，实验绑定exact SHA；每个有信息的阶段commit/push，更新CURRENT和RESEARCH_HISTORY，RESEARCH_INDEX只按既有职责链接结论。不要把每条运行日志复制进三份主文档。

最终报告只需概括：功能源/文档SHA、具体新增能力、实际测试及其边界、同输入对照、可用计划覆盖/最长空档、全路径耗时、command invariance、首个B1结果或未运行原因、剩余问题。附可复现命令与精选证据入口。避免把“计划/脚本已生成”“空检查集全绿”“旧版测试PASS”当成交付完成。
