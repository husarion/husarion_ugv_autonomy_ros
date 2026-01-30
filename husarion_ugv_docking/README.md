# husarion_ugv_docking

The package contains a `ChargingDock` plugin for the [opennav_docking](https://github.com/open-navigation/opennav_docking) project. Thanks to this package, Panther can dock to a charging station.

## Launch Files

- `docking.launch.py`: Launch a node that creates `docking_server` and runs a `ChargingDock` plugin. Also it launches `station.launch.py`.
- `station.launch.py`: Launch a node that creates a charging station description with generated apriltag.

## Configuration Files

- [`husarion_ugv_docking_server.yaml`](./config/docking_server.yaml): Defines parameters for a `docking_server` and a `ChargingDock` plugin. Defines poses where charging docks are spawned in the Gazebo.
- [`send_to_dock.yaml`](./config/send_to_dock.yaml): Defines parameters used by `send_to_dock_node` for docking action.

## ROS Nodes

- `DockPosePublisherNode`: A lifecycle node listens to `tf` and republishes position of `dock_pose` in the fixed frame when it is activated.
- `ChargingDock`:  A plugin for a Panther robot what is responsible for a charger service.
- `DockDatabaseUpdater`: Listens to new poses for docking stations and reloads the database of `docking_server`.
- `SendToDockNode`: A node which creates service server SetBool `send_robot_to_dock` and based on a request value sent (true/false) it either sends a robot to a dock or stops the docking action.

### DockPosePublisherNode

#### Publishes

- `docking/dock_pose` [*geometry_msgs/PoseStamped*]: An offset dock pose.

#### Subscribers

- `tf` [*tf2_msgs/TFMessage*]: Tf tree with a detected dock transform.

#### Parameters

- `fixed_frame` [*string*, default: **odom**]: A fixed frame id of a robot.
- `<dock_name>.type` [*string*, default: **charging_dock**]: It checks if this dock with name `dock_name` is a type of  `charging_dock`.
- `<dock_name>.frame` [*string*, default: **main_wibotic_transmitter_link** ]: Then look for transformation between `fixed_frame` and `<dock_name>.frame`  to publish `dock_pose`. A frame id of a wireless transmitter.

### ChargingDock

#### Publishes

- `docking/staging_pose` [*geometry_msgs/PoseStamped*]: An offset staging pose next to a charging station.

#### Subscribers

- `docking/dock_pose` [*geometry_msgs/PoseStamped*]: An offset dock pose.

#### Parameters

- `base_frame` [*string*, default: **base_link**]: A frame id of a wireless receiver.
- `fixed_frame` [*string*, default: **odom**]: A fixed frame id of a robot.
- `charging_dock.external_detection_timeout` [*double*, default: **0.2**]: A timeout in seconds for looking up a transformation from an april tag of a dock to a base frame id.
- `charging_dock.docking_distance_threshold` [*double*, default: **0.05**]: A threshold of a distance between a robot pose and a dock pose to declare if docking succeed.
- `charging_dock.docking_yaw_threshold` [*double*, default: **0.3**]: A threshold of a difference of yaw angles between a robot pose and a dock pose to declare if docking succeed.
- `charging_dock.staging_x_offset` [*double*, default: **-0.7**]: A staging pose is defined by offsetting a dock pose in axis X.
- `charging_dock.filter_coef` [*double*, default: **0.1**]: A key parameter that influences the trade-off between the filter's responsiveness and its smoothness, balancing how quickly it reacts to new pose data pose how much it smooths out fluctuations.
- `charging_dock.use_wibotic_info` [*bool*, default: **True**]: Whether WiBotic information is used.
- `charging_dock.wibotic_info_timeout` [*double*, default: **1.5**]: A timeout in seconds to receive a wibotic_info.
- `<dock_name>.apriltag_id` [*int*, default: **0**]: AprilTag ID of a dock.
- `<dock_name>.dock_frame` [*string*, default: **main_wibotic_transmitter_link**]: A frame id to compare with fixed frame if docked.
- `<dock_name>.pose` [*list*, default: **[0.0, 0.0, 0.0]**]: A dock position on the map. In the simulation, the station will be created with an orientation facing opposite to the dock position.

### DockDatabaseUpdater

#### Subscribers

- `<dock_name>/new_dock_pose` [*geometry_msgs/PoseStamped*]: A new pose for a dock with name `dock_name`.

#### Service clients

- `docking_server/reload_database` [*nav2_msgs/ReloadDockDatabase]: A service from `docking_server` to reload a database from yaml file.

#### Parameters

- `docks` [*list of strings*, default: **["main"]**]: List of dock names to be managed.
- `dock_database_filepath` [*string*, default: **"dock_database.yaml"**]: Path to the new YAML file containing dock database information.
- `<dock_name>.type` [*string*, default: **charging_dock**]: Type of the dock with the given name.
- `<dock_name>.pose` [*list of doubles*, default: **[0.0, 0.0, 0.0]**]: Pose of the dock with the given name.
- `<dock_name>.frame` [*string*, default: **main_wibotic_transmitter_link**]: Frame ID associated with the dock.

### SendToDockNode

#### Service server

- `send_robot_to_dock` [*std_srvs/SetBool*]: Receives requests for docking or stopping the docking action.
  - If *std_srvs/SetBool: true* then the node sends request of docking with specified parameters to the action client.
  - If *std_srvs/SetBool: false* then the node sends request of cancelling current docking action.

#### Action client

- `dock_robot` [*nav2_msgs/DockRobot*]: Docks robot or stops docking depending on the request sent by the service server `send_robot_to_dock`.

#### Parameters

- `dock_type` [*string*, default: "charging_dock"]: Type of the dock the robot navigates to.
- `navigate_to_staging_pose` [*bool*, default: true]: Whether the robot has to use navigation stack to dock or not.
- `dock_id` [*string*, default: "main"]: Name of the docking station.

<!-- Override description
ros2 launch panther_description overwrite_robot_description.launch.py controller_config_path:=$(pwd)/panther_ros/panther_description/config/components.yaml namespace:=panther
 -->
