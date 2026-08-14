"""Go2 flat velocity curriculum: lin_vel_x ±2.0 m/s."""

from isaaclab.utils.configclass import configclass
from isaaclab_tasks.manager_based.locomotion.velocity.config.go2.flat_env_cfg import UnitreeGo2FlatEnvCfg


@configclass
class UnitreeGo2FlatFastEnvCfg(UnitreeGo2FlatEnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()
        self.commands.base_velocity.ranges.lin_vel_x = (-2.0, 2.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.5, 0.5)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)
        self.rewards.track_lin_vel_xy_exp.weight = 1.5
        self.viewer.eye = (0.8, 6.0, 1.2)
        self.viewer.lookat = (0.8, 0.0, 0.35)


@configclass
class UnitreeGo2FlatFastEnvCfg_PLAY(UnitreeGo2FlatFastEnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None
