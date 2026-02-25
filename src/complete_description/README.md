# complete_description

ROS 2 Humble-ready description package created from the provided URDF set.

## Included

- Converted package metadata/build files to ROS 2 (`ament_cmake`)
- Gazebo Fortress (`ros_gz_sim`) bringup
- Joint-position control via Gazebo Fortress system plugins and topic bridges
- Launch files:
  - `launch/gazebo_bringup.launch.py` (spawn robot in Fortress + ros_gz bridges)
  - `launch/view.launch.py` (RViz + joint state publisher GUI)
- Demo world with object: `worlds/pick_place_demo.sdf`

## Joint names used

- `revolute_13`
- `revolute_5`
- `revolute_6`
- `revolute_7`
- `revolute_8`
- `revolute_12`

(These are sanitized from names with spaces to improve ROS 2 compatibility.)

## Gazebo Fortress command topics

- `/revolute_13_cmd`
- `/revolute_5_cmd`
- `/revolute_6_cmd`
- `/revolute_7_cmd`
- `/revolute_8_cmd`
- `/revolute_12_cmd` (end-effector rotary gripper)
