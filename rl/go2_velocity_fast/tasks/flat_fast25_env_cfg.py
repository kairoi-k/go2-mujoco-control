"""Go2 flat velocity curriculum: lin_vel_x ±2.5 m/s."""

from isaaclab.utils.configclass import configclass

from .flat_fast_env_cfg import UnitreeGo2FlatFastEnvCfg


@configclass
class UnitreeGo2FlatFast25EnvCfg(UnitreeGo2FlatFastEnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()
        self.commands.base_velocity.ranges.lin_vel_x = (-2.5, 2.5)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.5, 0.5)
        self.viewer.eye = (2.5, 10.0, 2.0)
        self.viewer.lookat = (2.5, 0.0, 0.35)


@configclass
class UnitreeGo2FlatFast25EnvCfg_PLAY(UnitreeGo2FlatFast25EnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None
