# husarion_ugv_navigation

The package contains the nav2 package configurations and E-STOP handling during navigation for Husarion UGVs.

## Launch Files

This package contains:

- `bringup_launch.py`: Main launch responsible for activating nav2 stack.
- `localization_launch.py`: Responsible for running localization-related nodes (including `amcl`).
- `navigation_launch.py`: Responsible for running navigation-related nodes.
- `slam_launch.py`: Responsible for running slam-related nodes (including `slam_toolbox`).

## Executables

### autonomy_preflight

Runs before Nav2 in `bringup_launch.py` (`preflight:=True`, the default) and in two phases:

1. **Wait for the robot, with no deadline.** A robot that boots together with its user computer,
   or whose driver restarts, shows up tens of seconds late. The preflight waits until the robot's
   `<ns>/odom -> <ns>/base_link` is on TF, and every 10 s logs what it waits for. If TF carries
   another robot's `base_link`, the log names its namespace, so a wrong `namespace:=` does not
   wait unnoticed.
2. **Check what Nav2 needs, `preflight_timeout` seconds each (30 s by default).** The
   `mount_link` height (with `lidar_profile` and no `mount_height`), the LiDAR's messages, the
   LiDAR's frame on the robot and a subscriber on Nav2's velocity output. For that check it holds
   a publisher on the topic that sends nothing: on Husarion UGV OS the robot's bridge subscribes to
   a topic only once it has a publisher. A failure prints a report with the fix and ends the launch
   with an error. Nav2 is not started.

```text
map-nav preflight: namespace lynx_d6b2, timeout 30 s

  [ok  ] mount TF       mount_link is 0.366 m above base_footprint
  [ok  ] driver TF      lynx_d6b2/odom -> lynx_d6b2/base_link is available
  [FAIL] lidar          nothing publishes /lynx_d6b2/rplidar/points
  [ok  ] velocity sink  the robot listens on /lynx_d6b2/autonomous/cmd_vel
```

Run it on its own against a live robot:

```bash
ros2 run husarion_ugv_navigation autonomy_preflight --namespace lynx_d6b2 --mount \
  --observation-topic /lynx_d6b2/rplidar/points --cmd-vel-topic /lynx_d6b2/autonomous/cmd_vel
```

`preflight:=False` skips the checks of phase 2. The launch still waits for the robot when it reads
`mount_height` from TF.

Two more modes serve the Cockpit's project form on the user computer. Each prints one JSON line on
stdout and logs to stderr:

- `--list-observation-topics [--timeout S]` discovers the ROS graph for S seconds (3 by default)
  and lists the LaserScan and PointCloud2 topics, sorted by name. It does not wait for the robot
  and always exits 0.

  ```json
  {"topics": [{"name": "/lynx/points", "type": "pointcloud"}, {"name": "/user/scan", "type": "laserscan"}]}
  ```

- `--observation-topic T --observation-topic-type auto|laserscan|pointcloud --checks observation
  --json [--timeout S]` checks only the observation topic: its type (`auto` reads it from the
  graph), its messages with the rate over the S-second window, and its frame on
  `<ns>/base_link`. It waits for the robot at most S seconds and then reports `robot not visible`.
  It exits 0 when every check passes and 1 otherwise. A failed check carries its `fix`.

  ```json
  {"ok": true, "type": "pointcloud", "frame_id": "lynx/os_lidar", "rate_hz": 10.0,
   "checks": [{"name": "type", "ok": true, "detail": "/lynx/points carries a pointcloud", "fix": null}, ...]}
  ```

### is_estop

A BehaviorTree plugin responsible for chancing the e-stop status and cancel navigation action if e-stop is activated.

#### Subscribers

- `hardware/e_stop` [*std_msgs/Bool*]: E-stop state.

### map_autosaver_node

A ROS node responsible for autosave map.

### Clients

- `map_saver/save_map` [*nav2_msgs/srv/save_map*]: Service which save map.
