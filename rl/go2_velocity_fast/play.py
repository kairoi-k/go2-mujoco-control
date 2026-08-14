"""Play a Go2 fast-velocity policy. Registers gym ids, then runs Isaac Lab's RSL-RL play."""

from go2_velocity_fast._launch import launch_isaaclab_script


def main() -> None:
    launch_isaaclab_script("play.py")


if __name__ == "__main__":
    main()
