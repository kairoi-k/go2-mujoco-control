# Reactive acceptance representatives — 2026-08-20

这是给老师/人工验收的代表性长窗口，不覆盖原来的 49 条穷举矩阵。每个视频都保留约 3 s 行进前置、至少 4 s 事件窗口和恢复段；`obstacle_to_turn_handoff` 的两个事件各保持 4 s。

`impact_strong_recovery` 使用真实 MuJoCo 速度冲击（`push-vel-x=0.80 m/s`、0.20 s），不是矩阵中仅把参考速度归零的 scheduled impact，所以它会明显踉跄并自动恢复。`slip_reference` 是控制器参考保护测试：不改地面参数，速度参考降到 0.45 倍。`low_friction_physical` 是真实地面摩擦系数在 4 s 内降至 μ=0.02；它本来可能视觉不明显，判断以同步数据图为准，不能把它误说成“脚底打滑检测已触发”。

`turn_left_long` 只改变偏航参考；`obstacle_left_physical` 使用 `scene_reactive_obstacle.xml` 的真实红色碰撞箱，避障命令同时包含横移和偏航，验收条件是绕开且机器人—障碍物接触力/次数均为 0。`obstacle_to_turn_handoff` 专门展示长窗口衔接，不是两个短动作一闪而过。

原始视频在 `videos/`；带同步数据面板的视频在 `videos_data_panel/`，总合集为 `reactive_representatives_data_panel_collection.mp4`。面板左侧实线是测量反馈，虚线是控制器参考，红色游标是当前视频时刻；上图为 vx/vy，下图为 yaw rate。`representative_manifest.json` 和 `data_panel_manifest.json` 记录文件与实际事件时段；`metrics.csv` 及两张图用于区分“参考变化”和“物理环境变化”。原始运行证据仍在本地仓库对应的 `example/cpp/experiments/_runs/reactive_representative_*`。
