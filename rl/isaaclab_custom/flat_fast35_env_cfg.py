"""Go2 flat velocity curriculum: lin_vel_x ±3.5 m/s.

Drop-in for Isaac Lab's Go2 velocity task package. Gait terms stay at
the official velocity-task defaults (no reference-motion imitation).
"""
from isaaclab.utils.configclass import configclass
from isaaclab_tasks.manager_based.locomotion.velocity.config.go2.flat_fast30_env_cfg import UnitreeGo2FlatFast30EnvCfg


@configclass
class UnitreeGo2FlatFast35EnvCfg(UnitreeGo2FlatFast30EnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()
        self.commands.base_velocity.ranges.lin_vel_x = (-3.5, 3.5)


@configclass
class UnitreeGo2FlatFast35EnvCfg_PLAY(UnitreeGo2FlatFast35EnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()
        self.scene.num_envs = 16
        import isaaclab.sim as sim_utils
        from isaaclab.sensors import CameraCfg

        self.scene.tiled_cam = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Camera",
            update_period=0.1,
            height=240,
            width=320,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=24.0,
                focus_distance=400.0,
                horizontal_aperture=20.955,
                clipping_range=(0.1, 20.0),
            ),
            offset=CameraCfg.OffsetCfg(
                pos=(0.0, -1.5, 0.6),
                rot=(0.0, 0.0, 0.0, 1.0),
                convention="world",
            ),
        )
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None
