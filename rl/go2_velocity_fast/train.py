"""Train a Go2 fast-velocity policy. Registers gym ids, then runs Isaac Lab's RSL-RL train."""

from go2_velocity_fast._launch import launch_isaaclab_script


def main() -> None:
    launch_isaaclab_script("train.py")


if __name__ == "__main__":
    main()
