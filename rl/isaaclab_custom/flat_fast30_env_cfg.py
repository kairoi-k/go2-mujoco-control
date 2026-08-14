"""Go2 flat velocity curriculum: lin_vel_x ±3.0 m/s.

Drop-in for Isaac Lab's Go2 velocity task package.
"""
from isaaclab.utils.configclass import configclass
from isaaclab_tasks.manager_based.locomotion.velocity.config.go2.flat_fast25_env_cfg import UnitreeGo2FlatFast25EnvCfg


@configclass
class UnitreeGo2FlatFast30EnvCfg(UnitreeGo2FlatFast25EnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()
        self.commands.base_velocity.ranges.lin_vel_x = (-3.0, 3.0)


@configclass
class UnitreeGo2FlatFast30EnvCfg_PLAY(UnitreeGo2FlatFast30EnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None
