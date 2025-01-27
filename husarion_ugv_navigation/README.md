# husarion_ugv_navigation

The package contains the nav2 package configurations and E-STOP handling during navigation for Husarion UGVs.

## Launch Files

This package contains:

- `bringup_launch.py`: Main launch responsible for activating nav2 stack.
- `localization_launch.py`: Responsible for running localization-related nodes (including `amcl`).
- `navigation_launch.py`: Responsible for running navigation-related nodes.
- `slam_launch.py`: Responsible for running slam-related nodes (including `slam_toolbox`).

## Executables

### is_estop

A BehaviorTree plugin responsible for chancing the e-stop status and cancel navigation action if e-stop is activated.

#### Subscribers

- `hardware/e_stop` [*std_msgs/Bool*]: E-stop state.

### map_autosaver_node

A ROS node responsible for autosave map.

### Clients

- `map_saver/save_map` [*nav2_msgs/srv/save_map*]: Service which save map.
