# husarion-ugv-autonomy

A collection of packages containing autonomous functionalities for Husarion UGV vehicles.

![autonomy-result](https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/husarion_ugv/husarion_ugv_autonomy.gif)

## 🛠️ Setup Repository

### Create Workspace

```bash
mkdir ~/husarion_ws
cd ~/husarion_ws
git clone https://github.com/husarion/husarion_ugv_autonomy_ros.git src/husarion_ugv_autonomy_ros
```

### Build

```bash
vcs import src < src/husarion_ugv_autonomy_ros/husarion_ugv_autonomy/docking_deps.repos

sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to husarion_ugv_autonomy --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

## 🚀 Demo

### 📋 Requirements

1. **Just** To simplify the execution of this project, we are utilizing [just](https://github.com/casey/just). Install it with:

    ```bash
    curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | sudo bash -s -- --to /usr/bin
    ```

2. **Robot Configuration**

> [!IMPORTANT]
> The provided example is configured for the Husarion UGV platforms and supports any LIDAR that publishes `PointCloud2` or `LaserScan` and any camera that publishes `Image` and `CameraInfo` data types by setting the appropriate environment variable. Before running the navigation demo, ensure the following:
>
> - This demo should be run on **User Computer** with IP address: **`10.15.20.3/24`**.
> - A LIDAR publishes messages of type: **`PointCloud2`** or **`LaserScan`**.
> - A camera publishes messages of type: **`Image`** and **`CameraInfo`**.
> - A static transformation between a LIDAR, a Camera and a robot frame is provided. The value of the **`frame_id`** field inside the published messages must connect to the robot's `base_link`.

### 🧭 Navigation

#### Step 1: Environment configuration

Download this repository:

```bash
git clone https://github.com/husarion/husarion_ugv_autonomy_ros
```

Setup environment:

```bash
cd husarion_ugv_autonomy_ros
export OBSERVATION_TOPIC={point_cloud_topic} # absolute topic name to match your LIDAR pointcloud2 topic (e.g. /scan)
export OBSERVATION_TOPIC_TYPE={msg_type} # Specify: `laserscan`, `pointcloud`
export CAMERA_IMAGE_TOPIC={camera_image_topic} # absolute topic name to match your camera image topic (e. g. /camera/color/image_raw)
export CAMERA_INFO_TOPIC={camera_info_topic} # absolute topic name to match your camera info topic (e. g. /camera/camera_info)
export SLAM=True # if you have a map you can run navigation without SLAM
```

> [!NOTE]
> Additional arguments are detailed in the [Launch Arguments](#launch-arguments) section.

#### Step 2: Run navigation

Run navigation on the **physical robot**:

```bash
just start-hardware
```

Run navigation in **Gazebo simulation**:

```bash
just start-simulation
```

#### Step 3: Control the robot from a Web Browser

1. Install and run husarion-webui

    ```bash
    just start-visualization
    ```

2. Open the your browser on your laptop and navigate to:

    - http://{ip_address}:8080/ui (devices in the same LAN)
    - http://{hostname}:8080/ui (devices in the same Husarnet Network)

### ⚓ Docking

#### Step 1: Locate docks

Once you have mapped an area, locate your charging docks on map and select their poses in [the configuration file](docker/config/docking_server.yaml). You can use RViz or Foxglove.

In the example below for dock named `main` the position is `pose: [1.0, 1.20, 1.57]`.

```yaml
[...]
    main:
        [...]
        pose: [1.0, 1.20, 1.57] # [x, y, yaw] of the dock on the map. Used also for spawning dock in the simulation.
[...]
```

#### Step 2: Run Docking

```bash
just start-docking
```

#### Step 3: Dock the robot

```bash
just dock main
```

or press LB + RB + Y on the gamepad.

#### Step 4: Undock the robot

```bash
just undock
```

or press LB + RB + X on the gamepad.

## Documentation

### Launch Arguments

| Argument                 | Description <br/> ***Type:*** `Default`                                                               |
| ------------------------ | ----------------------------------------------------------------------------------------------------- |
| `autostart`              | Automatically startup the nav2 stack. <br/> ***bool:*** `True`                                        |
| `log_level`              | Logging level. <br/> ***string*** `info` (choices: `debug`, `info`, `warning`, `error`, `custom`)     |
| `map`                    | Path to map yaml file to load. <br/> ***string:*** `/maps/map.yaml`                                   |
| `namespace`              | Add namespace to all launched nodes. <br/> ***string:*** `env(ROBOT_NAMESPACE)`                       |
| `observation_topic`      | Topic name for LaserScan or PointCloud2 observation messages type. <br/> `''`                         |
| `observation_topic_type` | Observation topic type. <br/> ***string:*** `pointcloud` (choices: `laserscan`, `pointcloud`)         |
| `params_file`            | Path to the parameters file to use for all nav2 related nodes. <br/> ***string:*** [`nav2_params.yaml](./husarion_ugv_navigation/config/nav2_params.yaml) |
| `pc2ls_params_file`      | Path to the parameters file to use for pointcloud_to_laserscan node. <br/> ***string:*** [`pc2ls_params.yaml](./husarion_ugv_navigation/config/pc2ls_params.yaml) |
| `slam`                   | Whether run a SLAM. <br/> ***bool:*** `False`                                                         |
| `use_composition`        | Whether to use composed bringup. <br/> ***bool:*** `True`                                             |
| `use_respawn`            | Whether to respawn if a node crashes. Applied when composition is disabled. <br/> ***bool:*** `False` |
| `use_sim_time`           | Use simulation (Gazebo) clock if true. <br/> ***bool:*** `False`                                      |
