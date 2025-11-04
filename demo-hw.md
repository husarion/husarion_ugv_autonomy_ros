# 🚀 Hardware Demo

This guide walks you through the most important steps needed to run the autonomy configuration on a physical robot.

## 📋 Requirements

1. **Husarion UGV Platform & ROS Driver**

    This demo is prepared for the **Lynx** and **Panther** robots. This version has been tested with [**Husarion UGV Jazzy 2.3.1**](https://github.com/husarion/husarion_ugv_ros/tree/2.3.1) ROS drivers.

2. **Robot Configuration**

    - Run the demo from the **User Computer** with IP address: **`10.15.20.3/24`**.
    - A LIDAR publishing either a `PointCloud2` or a `LaserScan` topic.
    - A camera that publish RGB `Image` and corresponding `CameraInfo` topic. (Not required if docking is not used.)
    - A static transform between the LIDAR, Camera, and robot frame. Ensure the **`frame_id`** in the published messages is connected to the robot’s `base_link`. For more details, see the [documentation on configuring transforms for sensors](https://github.com/husarion/husarion_ugv_ros/blob/ros2/husarion_ugv_description/CONFIGURATION.md#urdf---robot-model-configuration).
.

3. **Just**

    To simplify running commands, we use [just](https://github.com/casey/just). Install it with:

    ```bash
    sudo snap install just
    ```

## 🧭 Navigation

### Step 1: Configure the environment

Setup environment:

```bash
export OBSERVATION_TOPIC={point_cloud_topic} # absolute LIDAR topic (e.g. /scan)
export OBSERVATION_TOPIC_TYPE={msg_type} # laserscan | pointcloud
export CAMERA_IMAGE_TOPIC={camera_image_topic} # absolute camera rgb image topic (e.g. /camera/color/image_raw)
export CAMERA_INFO_TOPIC={camera_info_topic} # absolute camera info topic (e.g. /camera/camera_info)
export SLAM=True # set False if you already have a map
export ROBOT_MODEL=panther # set to 'lynx' if using Husarion UGV Lynx
export ROBOT_NAMESPACE=panther # set to 'lynx' if using Husarion UGV Lynx
```

### Step 2: Start navigation

Run navigation on the **physical robot**:

```bash
just start-hardware navigation
```

### Step 3: Control the robot via Web Browser

1. Start the web interface:

    ```bash
    just start-visualization
    ```

2. Open your browser and navigate to:

    - http://{ip_address}:8080/ui (devices in the same LAN)
    - http://{hostname}:8080/ui (devices in the same Husarnet Network)

## ⚓ Docking

### Step 1: Ensure navigation is running

### Step 2: Define dock locations

After mapping the area, specify charging dock poses in [docker/config/docking_server.yaml](docker/config/docking_server.yaml). You can use **RViz** or **Foxglove** to get the poses.

In the example below for dock named `main` the position is `pose: [1.0, 1.20, 1.57]`.

```yaml
[...]
    main:
        [...]
        pose: [1.0, 1.20, 1.57] # [x, y, yaw] of the dock on the map. Used also for spawning dock in the simulation.
[...]
```

### Step 3: Setup OS

```bash
just setup-os
```

### Step 4: Start Docking

```bash
just start-hardware docking
```

### Step 5: Dock the robot

```bash
just dock main
```

or press **LB + RB + Y** on the gamepad.

### Step 6: Undock the robot

```bash
just undock
```

or press **LB + RB + X** on the gamepad.

## ✅ Further Information

Now that you’ve gone through the demo, feel free to experiment and explore the robot’s autonomous features.
You can adjust the configuration and parameters to match your setup:

- [compose.hardware.yaml](./docker/compose.hardware.yaml)
- [apriltag.yaml - tag detection](./husarion_ugv_docking/config/apriltag.yaml)
- [docking_server.yaml - docking parameters](./husarion_ugv_docking/config/docking_server.yaml)
- [nav2_params.yaml - navigation parameters](./husarion_ugv_navigation/config/nav2_params.yaml)
