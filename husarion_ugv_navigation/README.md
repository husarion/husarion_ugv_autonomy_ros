# husarion_ugv_navigation

The package contains the nav2 package configurations and E-STOP handling during navigation for Husarion UGVs.

## Launch Files

This package contains:

- `bringup_launch.py`: Main launch responsible for activating nav2 stack.
- `localization_launch.py`: Responsible for running localization-related nodes (including `amcl`).
- `navigation_launch.py`: Responsible for running navigation-related nodes.
- `slam_launch.py`: Responsible for running slam-related nodes (including `slam_toolbox`).

## Velocity Chain

`controller_server` → `cmd_vel_nav` → `velocity_smoother` → `cmd_vel_smoothed` → `collision_monitor` → `cmd_vel`

`collision_monitor` (nav2_collision_monitor) simulates the commanded motion of the bare robot footprint against the observation source (the cropped point cloud, or the laser scan) and slows the robot down so that contact stays at least 1 s ahead, down to a full stop. If the source stops publishing for 1 s, it stops the robot.

- `collision_monitor_state` [*nav2_msgs/CollisionMonitorState*]: Current action and the polygon that triggered it.
- `collision_monitor_footprint` [*geometry_msgs/PolygonStamped*]: Footprint used by the monitor, for visualization.

## Executables

### is_estop

A BehaviorTree plugin responsible for chancing the e-stop status and cancel navigation action if e-stop is activated.

#### Subscribers

- `hardware/e_stop` [*std_msgs/Bool*]: E-stop state.

### map_autosaver_node

A ROS node responsible for autosave map.

### Clients

- `map_saver/save_map` [*nav2_msgs/srv/save_map*]: Service which save map.
