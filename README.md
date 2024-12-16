# husarion-ugv-autonomy

A GitHub template for Husarion UGVs: creating a map using Slam Toolbox and navigation with localization using Nav2.

![autonomy-result](https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/husarion_ugv/husarion_ugv_autonomy.gif)

## 📋 Requirement

### Justfile

To simplify the execution of this project, we are utilizing [just](https://github.com/casey/just). Install it with:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | sudo bash -s -- --to /usr/bin
```

### Robot Configuration

The provided example is configured for the Panther robot and supports any LIDAR that publishes `PointCloud2` or `LaserScan` data type by setting the appropriate environment variable.

> [!IMPORTANT]
> Before running the navigation demo, ensure the following:
>
> - This demo should be run on **User Computer** with IP address: **`10.15.20.3/24`**.
> - a LIDAR publishes messages of type: **`PointCloud2`** or **`LaserScan`**.
> - A static transformation between a LIDAR and a robot frame is provided. The value of the **`frame_id`** field inside the published message must connect to the robot's `base_link`.

## 🚀 Quick Start

### 🔧 Step 1: Environment configuration

Download this repository:

```bash
git clone https://github.com/husarion/panther-navigation
```

Setup environment:

```bash
cd panther-navigation

# The following configuration is only needed for the physical robot
export OBSERVATION_TOPIC={point_cloud_topic} # absolute topic name to match your LIDAR pointcloud2 topic (e.g. /scan)
export OBSERVATION_TOPIC_TYPE={msg_type} # Specify: `laserscan`, `pointcloud`
```

> [!NOTE]
> Additional arguments are detailed in the [Launch Arguments](#launch-arguments) section.

### 🧭 Step 2: Run navigation

🤖 Run Navigation on the Physical Robot:

```bash
just start-hardware
```

🖥️ Run Navigation in Simulation:

```bash
just start-simulation
```

### 🕹️ Step 3: Control the robot from a Web Browser

1. Install and run husarion-webui

    ```bash
    just start-visualization
    ```

2. Open the your browser on your laptop and navigate to:

    - http://{ip_address}:8080/ui (devices in the same LAN)
    - http://{hostname}:8080/ui (devices in the same Husarnet Network)

## Launch Arguments

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
