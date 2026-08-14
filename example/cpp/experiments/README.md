# 实验附件库

这里是可复核的实验证据库，不是进度页。

- 当前主张与范围：`docs/RESEARCH_INDEX.md`
- 历史取舍与 EXP 编号：`docs/experiment-ledger.md`
- 本目录检索总表：`CATALOG.md`

## 顶层放什么

只放**正式证据**目录（当前为 `go2_*`）。每个正式目录应能独立说明：目的、关键参数、主要结果、证据文件。有 `README.md` 的以 README 为准。

## `_runs/` 放什么

默认临时跑次（名称不是 `go2_*`）。确认有价值后再晋升顶层或迁入 `_archive/`。

## `_archive/` 放什么

探针、扫参、失败尝试、批量 gate、松散日志。可追溯，**不代表当前方案**。各子批次有自己的 README。

## 文件约定

| 文件 | 是否进 git / 主线引用 |
|---|---|
| `README.md`、摘要表、结果图、必要 CSV | 正式实验可纳入版本库 |
| `controller.log`、`simulator.log`、`data.csv` | 默认归档保留，不进 git（`*.log` 已忽略） |
| `_archive/**` 原始批量 | 默认本机；git 已忽略 `atlas_*` 等大体量路径 |

## 新实验怎么进库

1. 先在 `docs/experiment-ledger.md` 建 EXP（或注明 diagnostic）。
2. 目录名：`go2_<意图>_<日期>` 或阶段前缀清晰可检索。
3. 写目录内 `README.md`（复现命令、参数、指标、文件清单）。
4. 只有 accepted / 对外需要的才留在顶层；其余进 `_archive/<批次>/`。
5. 更新本目录 `CATALOG.md` 一行。

## 当前顶层正式目录

见 `CATALOG.md`「正式证据」一节。一键步态基线的大批量 trot/WBC 原始跑次在 `_archive/2026-08_*`，主张以 `docs/RESEARCH_INDEX.md` 为准。
