# Phase 2 工作流治理（2026-08-29 整治版）

本文件取代旧纪律：redesign 文档 §4 的"一假设一改一 canary"、
WORKER_ORDERS 里的"不许 commit"红线、每轮强制 reviewer、每改必双
canary。目录/证据边界仍由工作区根的 `AGENTS.md` 和
`WORKSPACE_STANDARD.md` 管，本文件只管工作方法。验收合约文件、
analyzer 阈值、canary 定义依旧冻结，任何模式都不得改。

## 两种模式

### 探索模式（默认）

- 允许自由改代码、批量分析既有运行数据、跑 ctest。ctest 绿即可
  本地 commit 到 `phase2-b1-b3`（不推远端），每个可工作状态都要
  留下 checkpoint，杜绝上千行未提交堆积。
- 不强制 canary。canary 只在需要运行态证据回答具体问题时跑，
  且一次可以把多个待验证假设排进同一批串行运行。
- 分析类任务（不碰代码）不受仿真锁以外的任何限制，尽量批量做。

### 验收模式（声称某个验收门被修复时）

- 一次只验证一个假设；修复 diff 必须能单独指认。
- 抖动门（历史上 PASS/FAIL 交替的门）必须预注册样本量 n 并给
  Wilson 区间，单次 PASS/FAIL 不得下结论；稳定门用双采样。
- 必须过 reviewer 审查才能把结果写进交接/证据文档。
- 跑验收 canary 前工作树必须先 commit，证据记录精确 SHA。

## 证据与日志

- `ESCALATION_LOG.md` 沿用追加式格式，每轮一节，字段不变。
- `WORKER_ORDERS.md` 仍是主 agent 下单的唯一通道，订单里写明
  模式（探索/验收）和验收标准。
- `START_HERE.md` 不再手维护精确 HEAD，改为记录"最后验收过的
  commit + 当前阶段 + 当前 blocker"，HEAD 以 git 为准。

## 交接

交接者只需保证：工作树已 commit 或脏状态在 START_HERE 明写原因、
`git worktree list` 符合目录契约、日志最新一节与实际状态一致。
"编译通过"不等于阶段通过，这条不变。
