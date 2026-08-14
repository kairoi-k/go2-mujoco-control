"""Register Isaac Lab gym ids for the Go2 fast-velocity curricula."""

import gymnasium as gym
from isaaclab_tasks.manager_based.locomotion.velocity.config.go2 import agents

_RSL_RL = f"{agents.__name__}.rsl_rl_ppo_cfg:UnitreeGo2FlatPPORunnerCfg"
_SKRL = f"{agents.__name__}:skrl_flat_ppo_cfg.yaml"

_TASKS = (
    ("Isaac-Velocity-Flat-Unitree-Go2-Fast-v0", "flat_fast_env_cfg:UnitreeGo2FlatFastEnvCfg"),
    ("Isaac-Velocity-Flat-Unitree-Go2-Fast-Play-v0", "flat_fast_env_cfg:UnitreeGo2FlatFastEnvCfg_PLAY"),
    ("Isaac-Velocity-Flat-Unitree-Go2-Fast25-v0", "flat_fast25_env_cfg:UnitreeGo2FlatFast25EnvCfg"),
    ("Isaac-Velocity-Flat-Unitree-Go2-Fast25-Play-v0", "flat_fast25_env_cfg:UnitreeGo2FlatFast25EnvCfg_PLAY"),
    ("Isaac-Velocity-Flat-Unitree-Go2-Fast30-v0", "flat_fast30_env_cfg:UnitreeGo2FlatFast30EnvCfg"),
    ("Isaac-Velocity-Flat-Unitree-Go2-Fast30-Play-v0", "flat_fast30_env_cfg:UnitreeGo2FlatFast30EnvCfg_PLAY"),
    ("Isaac-Velocity-Flat-Unitree-Go2-Fast35-v0", "flat_fast35_env_cfg:UnitreeGo2FlatFast35EnvCfg"),
    ("Isaac-Velocity-Flat-Unitree-Go2-Fast35-Play-v0", "flat_fast35_env_cfg:UnitreeGo2FlatFast35EnvCfg_PLAY"),
)

for _id, _cfg in _TASKS:
    gym.register(
        id=_id,
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.{_cfg}",
            "rsl_rl_cfg_entry_point": _RSL_RL,
            "skrl_cfg_entry_point": _SKRL,
        },
    )
