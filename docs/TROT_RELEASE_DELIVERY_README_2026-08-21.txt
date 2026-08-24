Go2 trot / running release delivery
Date: 2026-08-21

这是一份可直接转发或验收的最小交付包。视频是 MuJoCo 原始 GUI 录制，
均为 24 秒、480 帧、camera-follow；数字验收和失败边界见两个文档。

文件
1. go2_natural_trot_straight_release.mp4
   自然小跑直线稳定基线，period 0.34 s，step 0.340 m。
2. go2_running_trot_straight_release.mp4
   低占空比高速跑态直线演示，period 0.26 s，step 0.320 m，约 1.15--1.18 m/s。
3. TROT_STRAIGHT_RUNNING_RELEASE_2026-08-21.md
   当前发布配置、验收阈值、复现命令和失败边界。
4. TROT_RELEASE_METRICS_2026-08-21.txt
   三次重复实验的核心数据和未发布速度探针记录。

代码仓库
/home/che/dev/go2-mujoco-control
分支 gait/natural-trot-1mps-2026-08-21

复现入口
bash example/cpp/scripts/run_natural_trot.sh 100 natural_release_v1 --headless
bash example/cpp/scripts/run_running_trot.sh 100 running_release_v1 --headless
随后按发布文档运行对应 Python 验收脚本。
