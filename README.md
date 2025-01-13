# husarion-ugv-autonomy

A GitHub template for Husarion UGVs: creating a map using Slam Toolbox and navigation with localization using Nav2.

![autonomy-result](https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/husarion_ugv/husarion_ugv_autonomy.gif)

## 📋 Requirement

### Justfile

To simplify the execution of this project, we are utilizing [just](https://github.com/casey/just). Install it with:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | sudo bash -s -- --to /usr/bin
```

## 🤖 Physical robot

The provided example is configured for the Panther robot and supports any LIDAR that publishes `PointCloud2` or `LaserScan` and any camera that publishes `Image` and `CameraInfo` data types by setting the appropriate environment variable.

> [!IMPORTANT]
> Before running the navigation demo, ensure the following:
>
> - This demo should be run on **User Computer** with IP address: **`10.15.20.3/24`**.
> - A LIDAR publishes messages of type: **`PointCloud2`** or **`LaserScan`**.
> - A camera publishes messages of type: **`Image`** and **`CameraInfo`**.
> - A static transformation between a LIDAR, a Camera and a robot frame is provided. The value of the **`frame_id`** field inside the published messages must connect to the robot's `base_link`.

### 🔧 Step 1: Environment configuration

Download this repository:

```bash
git clone https://github.com/husarion/panther-navigation
```

Setup environment:

```bash
cd panther-navigation
export OBSERVATION_TOPIC={point_cloud_topic} # absolute topic name to match your LIDAR pointcloud2 topic (e.g. /scan)
export OBSERVATION_TOPIC_TYPE={msg_type} # Specify: `laserscan`, `pointcloud`
export CAMERA_IMAGE_TOPIC={camera_image_topic} # absolute topic name to match your camera image topic (e. g. /camera/color/image_raw)
export CAMERA_INFO_TOPIC={camera_info_topic} # absolute topic name to match your camera info topic (e. g. /camera/camera_info)
export SLAM=True # if you have a map you can run navigation without SLAM
```

### 🧭 Step 2: Run navigation

Run navigation.

```bash
just start-hardware
```

### 🕹️ Step 3: Control the robot from a Web Browser

1. Install and run husarion-webui

    ```bash
    just start-visualization
    ```

2. Open the your browser on your laptop and navigate to:

    - http://{ip_address}:8080/ui (devices in the same LAN)
    - http://{hostname}:8080/ui (devices in the same Husarnet Network)

## 🖥️ Simulation

Example demo with Navigation2, using The Husarion Panther robot equipped with the Velodyne Puck.

### 🔧 Step 1: Environment configuration

Download this repository:

```bash
git clone https://github.com/husarion/panther-navigation
```

Setup environment:

```bash
export SLAM=True # if you have map you can run navigation without SLAM
```

### 🧭 Step 2: Run navigation

Run navigation.

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


## 🏗️ Docking

### ⚙️ Step 1: Locate docks
Once you have mapped an area, locate your charging docks on map and select their poses in [the configuration file](docker/config/docking_server.yaml). You can use RViz or Foxglove.

In the example below for dock named `main` the position is `pose: [1.0, 1.20, 1.57]`.

```yaml
[...]
    main:
        [...]
        pose: [1.0, 1.20, 1.57] # [x, y, yaw] of the dock on the map. Used also for spawning dock in the simulation.
[...]
```

### 🚀 Step 2: Run Docking

Run Docking nodes:

```bash
just start-docking
```

### ⚓ Step 2: Dock the robot

Run Docking sequence:

```bash
just dock
```

### 🛩️ Step 3: Undock the robot

Run Undocking sequence:

```bash
just undock
```
