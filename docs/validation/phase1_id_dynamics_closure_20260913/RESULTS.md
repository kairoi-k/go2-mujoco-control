# Phase1 ID dynamics closure补充归因

## 结论

本轮不能对主失败 transition `varying 1.4→2.3` 作完整的 ID closure 归因。原因是 diagnostic capture window 以 MuJoCo `state_tick_s=30–34` 固定，而该 run 的 velocity-profile active time 为 `cmd_time_s-active_start=32` 时才进入 2.3 m/s；实际对应 `state_tick_s≈37.6`。因此推送的 `closure.csv` 明确分为：

- `closure_pre_transition`：396 个精确 closure rows，active time `24.4060–28.3960 s`；
- `target_1p4_to_2p3`：500 个主失败窗口 rows，active time `32.0000–32.9981 s`，但没有精确 `M/h/J/full qdd/f/tau`，closure 字段留空。

这不是近似 closure，也不把 pre-transition 证据冒充主失败证据。最终归因：**INCONCLUSIVE，暂不选择 A–E**。上一轮“ID-WBC/contact realization 最早丢失 authority”被削弱为候选，尚未被本轮精确闭环证明。

## 数据与运行

基准为 B semantic source `f87cf929ddbb124a85464ce2863a4584b4270e2c`：`step=applied*period`、`effective-speed convention=false`、`period=0.14`、`duty=0.44`。诊断 instrumentation 仅增加默认关闭的 closure 采样；`1fb22e0b3dd4d42c984d6c6d6b44b1a8d2aaf281` 增加快照，`262a1e45dee3092589cb5cbd10f33218a1a25603` 将采样限制到 `state_tick_s 30–34` 且每 10 ms 一次。B runtime 数学、参数、模型和验收阈值未改。

运行命令均为原 Phase1 varying 入口，仅增加 `TROT_DIAG_ID_CLOSURE=1`：

```text
TROT_CPU_AUTOPIN=1 TROT_DIAG_ID_CLOSURE=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_id_dynamics_closure_20260913 218
TROT_CPU_AUTOPIN=1 TROT_DIAG_ID_CLOSURE=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_id_dynamics_closure_20260913 230
```

| run | source | result | coverage |
|---|---|---|---|
| `varying_20260913_182736` | `1fb22e0` | controller 0，hard-posture safety 1，controlled stop 1 | 未到主 endpoint 后 1 s |
| `varying_20260913_183234` | `262a1e4` | controller/safety/completion 全 0，strict pass | 主 profile 完整；closure 只覆盖 endpoint 前 |
| `varying_20260913_183212` | `262a1e4` | simulator DDS participant 启动失败，无 `data.csv` | 不计入数据 |

## 坐标、方程和映射确认

源码确认 `Go2RigidBody` 要求 `nq=19,nv=18`。`qpos[0:3]` 是 world base position，`qpos[3:7]` 是 world-from-body quaternion；`qvel/qacc[0:3]` 是 world linear base components，`[3:6]` 是 base angular components，后 12 项为 MuJoCo joint DoF。`mj_fullM` 给出 `M`；将 `qacc=0` 后调用 `mj_inverse` 得到 `h=qfrc_inverse`；`mj_jacGeom` 给出 world-frame foot linear Jacobian，按 `FR,FL,RR,RL` 堆叠。

ID-WBC 源码实际求解的等式为：

```text
M qdd + h = Sᵀ tau + Jᵀ f
tau = M_joint qdd + h_joint - J_jointᵀ f
```

`wbc_out.force[3*leg:3*leg+3]` 是逐腿 world-frame contact force；`Sᵀtau` 的 floating-base 前 6 行为零。源码注释确认 world +x 为 forward ground force，负 x 才是 braking 方向。`wbc_out.tau` 使用 MuJoCo joint-DoF 顺序，送往 LowCmd 前由 `MotorDof` 映射到 motor array；这不是诊断错误。

## 主失败窗口（已有 raw 的非 closure 字段）

第二条完整 run 的 active `32–33 s` 对应 1.4→2.3 target endpoint 后 1 s：

| quantity | median | p05 | p95 |
|---|---:|---:|---:|
| measured velocity (m/s) | 2.5228 | 2.4789 | 2.5438 |
| applied / kernel nominal (m/s) | 2.2772 | 2.2562 | 2.3000 |
| target error (m/s) | +0.2228 | +0.1789 | +0.2438 |
| WBC desired ax (m/s²) | -2.4567 | -2.8757 | -1.7889 |
| SRBD ax (m/s²) | -1.5062 | -4.2649 | +0.0729 |
| existing ID qdd_x (m/s²) | -2.2535 | -3.2724 | -0.8666 |
| existing simple ΣFx (N) | +2.6912 | -18.7782 | +19.8603 |
| realized ax, 100 ms local regression (m/s²) | +0.0739 | -0.6228 | +0.4900 |

Measured already exceeded 2.3 m/s at active time `32.0000 s`; `applied<shaped` began at `32.1000 s`. WBC/SRBD desired acceleration were negative at the start of the window; existing qdd_x became continuously negative at about `32.0700 s`. The target window has no exact per-leg `Jᵀf` or closure components, so it cannot distinguish ID mapping from plant/contact execution.

## Exact closure actually captured

For the 396 pre-transition rows, solver and final closure were computed independently from the same-tick `M,h,J,qdd,f,tau`:

| quantity | median | p95 | max |
|---|---:|---:|---:|
| solver full `|lhs-rhs|` component (N/Nm) | 0 | 8.953e-7 | 1.2804e-5 |
| solver base `|lhs-rhs|` component | 2.94e-7 | 1.215e-6 | 1.2804e-5 |
| final full residual component | 0 | 8.953e-7 | 1.2804e-5 |
| final base residual component | 2.94e-7 | 1.215e-6 | 1.2804e-5 |

在该窗口，base-x 的 median 为：`qdd_x=-1.5707`、`M_x0*qdd_x=-23.8847`、`Σ(j≠0)M_xj*qdd_j=+17.8281`、`h_x=+14.4429`、`base lhs_x=+10.0608`、`base Jᵀf_x=+10.0608`。这些逐样本闭合值说明 `qdd_x<0` 与 generalized contact x force 为正可由惯性耦合与 bias 合法共存，不能把简单 ΣFx 的符号当作 base acceleration 的替代。

逐腿 `Jᵀf` 的 base-x 贡献在每个 tick 求和与总 `base Jᵀf_x` 的最大差为 `2.0e-9`；简单逐腿 `Fx` 与 `base Jᵀf_x` 最大差为 `0`。捕获窗口中 `qdd_x<0` 为 `389/396`，而 generalized contact x force 为负仅 `94/396`，`qdd_x<0` 且 generalized contact x force 为正为 `295/396`。WBC internal contact mask 与 logger 的 foot-force threshold mask 有 `139/396` 不同；前者是 scheduled/hysteretic QP contact，后者是观测阈值，差异本身不等于 mapping fault。

Solver 后快照与最终快照的 force/tau 变化均为 `0`；按源码 `MotorDof` 重排后，final tau 与记录的 LowCmd `tau_ff` 最大误差为 `1.74e-7`。这对 pre-transition capture 提供中等强度证据：该窗口没有观察到 post-QP force/tau overlay 或 qdd→tau 映射破坏。它不能替代主 failure window 的同 tick closure。

## 归因分级与限制

- A（ID optimization/dynamics closure failure）：**NOT SUPPORTED（仅对已捕获 pre-transition window；主失败段 INCONCLUSIVE）**。
- B（ID output mapping/post-processing failure）：**INCONCLUSIVE**；捕获窗口映射未见偏离，但主失败段未采样。
- C（model-to-plant/contact realization failure）：**INCONCLUSIVE**；主失败段只有已有 qdd、简单 ΣFx 和 realized ax，缺精确 `Jᵀf` closure。
- D（diagnostic semantic error）：**NOT SUPPORTED**；qdd frame、MuJoCo DoF order、force frame/sign 和 motor remap 已由源码核实。
- E（distributed coupling）：**INCONCLUSIVE**；惯性/bias/contact topology 在已捕获窗口确实参与，但不足以把它定为主失败段首因。

本轮因此不维持“ID-WBC/contact realization 已是第一处明确偏离”的强表述；维持的是“WBC/SRBD/qdd 已要求减速而 realized ax 未稳定跟随”的观察，削弱的是“contact realization 已被证实为首因”。

## 唯一下一步建议（未执行）

只做一次 diagnostic-only B varying：不改 controller 数学、参数、模型或 profile，把 capture window 改为按该 run 的 `active_start_cmd_time_s` 对齐到 active `[32,33) s`，同时保留 endpoint 前后各 100 ms，记录同 tick full `qdd/f/tau/M/h/J` 与 final LowCmd。该单次 closure 才能在 B/C/E 之间作有因果意义的判别；本轮不再执行。

## SHA256

| artifact | SHA256 |
|---|---|
| first `data.csv` | `aa1ffee9ec1480994ecb04a98092ffdfc51c93de85328d08842ff026e143634e` |
| first `run_metadata.txt` | `472fdf79d255036a6b6d8bb49441e02f500c92a81c699a82383a78f5a50b5f4b` |
| first `run_manifest.json` | `d124f52b874259cf7eb4656e55e95c90a4426b570d701b7e44b4badb8904dd7b` |
| second `data.csv` | `25e459029c9384f8dca86e0ec4aa33352615d663f55fc49816a741cf83932aaa` |
| second `run_metadata.txt` | `c17d97cccf0d28d989491d9b3a2f58aa5a6a478d687275f006fce1a460b14934` |
| second `run_manifest.json` | `020f42e5f9059b9b1bf1a6dd3960f1d96201af85e771af324c0179410f2a6b04` |
| varying profile | `9efcc3b2d89fb349a12990ace1cf6ceb45e0d731deb470bdf2af084d82449d74` |
| scene | `12286418247d0e240ae131b5ae5c60f3a7a481d4754aefe4517476e937aa05b8` |
| diagnostic controller binary | `3b6535c00089a2282ddb2f81471a5654e3f04da6419ddb50d59e6ffd6296142c` |
| simulator binary | `4b311183c0817aab82aee909d3f1a4d82fdec9ec8237eea5e8f3ba707475f137` |
