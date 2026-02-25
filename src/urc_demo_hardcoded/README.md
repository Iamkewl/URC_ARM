# urc_demo_hardcoded

Hard-coded pick-and-place scaffolding for demo-only workflows in ROS 2 Humble.

Default sequence now does:

1. Joint showcase (`joint1` through `joint6`, positive and negative motion)
2. Pick-and-place (`pregrasp -> grasp -> lift -> preplace -> place -> retreat`)

The default config is now mapped to the provided `complete_description` arm (6 joints).

## What is already set up

- A ROS 2 Python package with launch + config files.
- A sequencer node that sends pre-defined joint waypoints in `ros_gz_topic` mode (default for Fortress).
- Optional planning-scene object publish + attach/detach events for visual pick/place flow.

## What you still need from URDF/controllers

When tuning for your hardware model, usually only these values need updates in `config/hardcoded_pick_place.yaml`:

- `joint_names`
- `controller_mode`
- `joint_command_topics`
- `end_effector_link`
- `world_frame`
- waypoint joint values

## Build

From the workspace root:

```bash
colcon build --packages-select urc_demo_hardcoded
source install/setup.bash
```

## Run node only (after Fortress + bridges are up)

```bash
ros2 launch urc_demo_hardcoded hardcoded_pick_place.launch.py
```

## Run complete integrated demo (Fortress + spawn + bridges + hardcoded sequence)

```bash
ros2 launch urc_demo_hardcoded complete_gazebo_demo.launch.py
```

By default the Fortress stack runs in server-only mode for stability in Docker. To request GUI mode:

```bash
ros2 launch urc_demo_hardcoded complete_gazebo_demo.launch.py \
  gz_args:="-r /root/colcon_ws/install/complete_description/share/complete_description/worlds/pick_place_demo.sdf"
```

## Run arm-only movement demo (recommended for GUI demonstration)

This launch ignores pick/place and continuously moves the arm joints.

```bash
ros2 launch urc_demo_hardcoded arm_motion_gazebo_demo.launch.py \
  gz_args:="-r /root/colcon_ws/install/complete_description/share/complete_description/worlds/pick_place_demo.sdf"
```

The 5-DOF mapping used is:

- J1 -> `revolute_13`
- J2 -> `revolute_5`
- J3 -> `revolute_6`
- J5 -> `revolute_8`
- J6 -> `revolute_12`

Optional custom parameter file:

```bash
ros2 launch urc_demo_hardcoded hardcoded_pick_place.launch.py \
  params_file:=/absolute/path/to/your_pick_place.yaml
```
