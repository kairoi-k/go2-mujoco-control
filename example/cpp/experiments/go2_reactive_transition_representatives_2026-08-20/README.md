# A-to-B reactive transition representative collection

这是面向老师展示的 A-to-B 过渡合集：每段先执行事件 A，再切换到事件 B，最后保留恢复段；不是六个独立事件的拼接。

合集包含 6 段：真实碰撞箱 OBSTACLE LEFT -> TURN LEFT；名义平地参考衔接 OBSTACLE LEFT -> OBSTACLE RIGHT、TURN LEFT -> TURN RIGHT、SLIP -> LOW FRICTION、LOW FRICTION -> TURN RIGHT；真实 MuJoCo 速度冲击 PHYSICAL IMPACT -> EMERGENCY STOP。最后一段使用 0.8 m/s、0.2 s 的速度冲击，控制器自动检测 impact 后再进入急停。

每段左侧是仿真，右侧是同步数据面板：实线为测量反馈，虚线为控制器参考，红色游标为当前视频时刻；上图为 vx/vy，下图为 yaw rate。真实碰撞箱只用于障碍物段，真实速度冲击只用于最后一段；其余段隔离场景变量，专门验证统一参考接口的连续衔接，不宣称完成自主感知规划。直接播放：reactive_ab_transition_representative_collection.mp4；单段视频在 OneDrive 的 videos_data_panel/；source_manifest.json 和 data_panel_manifest.json 记录来源与渲染信息。原始冲击视频为 videos/physical_impact_to_emergency_stop.mp4。
