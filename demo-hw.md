# 🚀 Hardware Demo

This guide walks you through the most important steps needed to run the autonomy configuration on a physical robot.

## 📋 Requirements

1. **Husarion UGV Platform & ROS Driver**

    This demo is prepared for the **Lynx** and **Panther** robots. This version has been tested with [husarion-ugv:jazzy-update-components-description](https://hub.docker.com/layers/husarion/husarion-ugv/jazzy-update-components-description/images/sha256-25c9afeab20864504edcfe0eed11c5d10d32015cf9644a00222c8e7ced9a472d) ROS driver, which should be specified on the **Built-in Computer** (IP address: **`10.15.20.2/24`**) in the `/home/husarion_ws/compose.yaml` file.

2. **Robot Configuration**

    - Run the demo from the **User Computer** with IP address: **`10.15.20.3/24`**.
    - Set up a LIDAR to publish either a PointCloud2 or a LaserScan topic.
    - Set up a camera to publish RGB `Image` and corresponding `CameraInfo` topic (not required if docking is not used).
    - Define a static transform between the LIDAR, camera, and robot frames, and ensure the published messages use a **`frame_id`** connected to the robot’s `base_link`. For more details, see the [documentation on configuring transforms for sensors](https://github.com/husarion/husarion_ugv_ros/blob/ros2/husarion_ugv_description/CONFIGURATION.md#urdf---robot-model-configuration).
    - Attach an AprilTag to your docking station, if used.
    - Detailed instructions on hardware and software setup, and apriltag requirements can be accessed in this article: [Autonomous Navigation and Docking for Panther & Lynx UGVs](https://husarion.com/blog/husarion-ugv-autonomy/).

3. **WiBotic**
    - If you plan to dock the robot using the `wibotic_receiver`, make sure this component is added to the robot URDF on the **Built-in Computer** (IP address: **`10.15.20.2/24`**). Add the appropriate configuration snippet shown below for your robot to the following file: `/home/husarion/config/husarion_ugv_description/config/components.yaml`.

    For Lynx:

    ```yaml
    components:
        - type: WCH01
            parent_link: cover_link
            xyz: 0.33 0.0 -0.15
            rpy: 0.0 0.0 0.0
    ```

    For Panther:

    ```yaml
    components:
        - type: WCH01
            parent_link: lights_channel_1_link
            xyz: -0.02 0.0 -0.0185
            rpy: 0.0 0.0 0.0
    ```

    - After adding the component, restart the driver on the Built-in Computer to apply the changes:

    ```bash
    docker compose down
    docker compose up --force-recreate
    ```

    - Make sure that the `wibotic_receiver` sensor is properly configured.

    - If the WiBotic system is not used, disable it later (during `Step 1` in `Navigation` part) by setting `export USE_WIBOTIC_INFO=False` in the `.env` file.

4. **Just**

    To simplify running commands, we use [just](https://github.com/casey/just). Install it with:

    ```bash
    sudo snap install just --classic
    ```

## 🧭 Navigation

### Step 1: Configure the environment

Remember to run the demo from the **User Computer** with IP address: **`10.15.20.3/24`**.

Configure the environment variables. Copy the `.env.template` into `.env` file:

```bash
cp src/husarion_ugv_autonomy_ros/.env.template src/husarion_ugv_autonomy_ros/docker/.env
```

**Check and adjust** the content of the `.env` file. Make sure that the file is located in the `docker` directory so that it works correctly with the `docker compose`. This file will automatically be sourced by `just`. Navigate to the directory from where the `just` commands might be called:

```bash
cd src/husarion_ugv_autonomy_ros
```

### Step 2: Setup OS

```bash
just setup-os
```

### Step 3: Start navigation

Run the navigation:

```bash
just start-hardware navigation
```

### Step 4: Control the robot via Web Browser

1. Open your browser and navigate to:

    - http://{ip_address}:8080/ui (devices in the same LAN) (default: [http://10.15.20.3:8080/ui](http://10.15.20.3:8080/ui))
    - http://{hostname}:8080/ui (devices in the same Husarnet Network)

2. If the visualization did not start itself, open a new terminal on the **User Computer** with IP address: **`10.15.20.3/24`**, go to `~/husarion_ws/src/husarion_ugv_autonomy_ros` and start the web interface:

    ```bash
    just start-visualization
    ```

## ⚓ Docking

### Step 1: Ensure the navigation is running

Verify that the navigation stack is running by opening the visualization tool and confirming that the map is visible and that the robot’s position updates in real time.

### Step 2: Define dock locations

Map the area, and after that, specify charging dock poses in [husarion_ugv_docking/config/docking_server.yaml](https://github.com/husarion/husarion_ugv_autonomy_ros/blob/ros2/husarion_ugv_docking/config/docking_server.yaml). You can use **RViz** or **Foxglove** to get the poses.

In the example below for dock named `main` the position is `pose: [1.0, 1.20, 1.57]`.

```yaml
[...]
    main:
        [...]
        pose: [1.0, 1.20, 1.57] # [x, y, yaw] of the dock on the map. Used also for spawning dock in the simulation.
[...]
```

If your robot is docked you can reset the odometry and make sure that the pose of the dock is set to [0.0, 0.0, 0.0] as well as the pose of the robot. To do that, reset the odometry by opening a new terminal on the **User Computer** with IP address: **`10.15.20.3/24`**, going to `~/husarion_ws/src/husarion_ugv_autonomy_ros`, and running:

```bash
source docker/.env
ros2 service call /${ROBOT_NAMESPACE:-panther}/reset std_srvs/srv/Empty
```

After calling this service, the `odom` frame is aligned with the current `base_link` frame. Consequently, the robot’s current pose is set to: `pose: [0.0, 0.0, 0.0]`.

-----------------

### Step 3: Start Docking

Open a new terminal on the **User Computer** with IP address: **`10.15.20.3/24`**, go to `~/husarion_ws/src/husarion_ugv_autonomy_ros`, and start docking:

```bash
just start-hardware docking
```

### Step 4: Dock the robot

Open a new terminal on the **User Computer** with IP address: **`10.15.20.3/24`**, go to `~/husarion_ws/src/husarion_ugv_autonomy_ros`, and dock the robot to the `main` station:

```bash
just dock main
```

or press **LB + RB + Y** on the gamepad (if AprilTag is visible).

The `just dock <dock_name>` command consists of two stages: navigating to the staging pose and then approaching the docking station from that pose. If the robot is already close enough to the docking station, you can skip the navigation stage and perform only the final docking maneuver by running:

```bash
just dock-direct main
```

Pressing **LB + RB + Y** on the gamepad triggers docking **without** navigating to the staging pose. This option works only when the AprilTag is visible in the camera.

### Step 5: Undock the robot

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
