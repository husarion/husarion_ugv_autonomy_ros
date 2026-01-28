# 🚀 Hardware Demo

This guide walks you through the most important steps needed to run the autonomy configuration on a physical robot.

## 📋 Requirements

1. **Husarion UGV Platform & ROS Driver**

    This demo is prepared for the **Lynx** and **Panther** robots. This version has been tested with [husarion-ugv:jazzy-update-components-description](https://hub.docker.com/layers/husarion/husarion-ugv/jazzy-update-components-description/images/sha256-25c9afeab20864504edcfe0eed11c5d10d32015cf9644a00222c8e7ced9a472d) ROS driver.

2. **Robot Configuration**

    - Run the demo from the **User Computer** with IP address: **`10.15.20.3/24`**.
    - Set up/Configure and prepare a LIDAR to publish either a PointCloud2 or a LaserScan topic.
    - Set up/Configure and prepare a camera to publish RGB `Image` and corresponding `CameraInfo` topic. (Not required if docking is not used.)
    - Define a static transform between the LIDAR, camera, and robot frames, and ensure the published messages use a **`frame_id`** connected to the robot’s `base_link`. For more details, see the [documentation on configuring transforms for sensors](https://github.com/husarion/husarion_ugv_ros/blob/ros2/husarion_ugv_description/CONFIGURATION.md#urdf---robot-model-configuration).

3. **Wibotic**
    - If you plan to dock the robot using the `wibotic_receiver`, make sure this component is added to the robot URDF on the **Built-in Computer** (IP address: **`10.15.20.2/24`**). If necessary, update the file `config/husarion_ugv_description/config/components.yaml` as shown below, and ensure the `xyz` and `rpy` values are set correctly for your setup:

    ```yaml
    components:
        - type: WCH01
            parent_link: cover_link
            xyz: 0.33 0.0 -0.15
            rpy: 0.0 0.0 0.0
    ```

    - After adding the component, restart the driver on the Built-in Computer to apply the changes:

    ```bash
    docker compose down
    docker compose up --force-recreate
    ```

    - If the `wibotic` system is not used, disable it by setting `use_wibotic_info:=False` in `docker/compose.hardware.yaml`.

4. **Just**

    To simplify running commands, we use [just](https://github.com/casey/just). Install it with:

    ```bash
    sudo snap install just --classic
    ```

## 🧭 Navigation

### Step 1: Configure the environment

Configure the environment variables. **Check and adjust** the content of the `.env` file. Make sure that the file is located in the `docker` directory so that it works correctly with the `docker compose`.

```bash
cp src/husarion_ugv_autonomy_ros/.env.template src/husarion_ugv_autonomy_ros/docker/.env
```

After modifying the file, and **in each newly opened terminal**, source the `.env` file:

```bash
cd src/husarion_ugv_autonomy_ros
source docker/.env
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

-----------------
^ komentarz o mapach i o punkcie 0,0,0

-----------------

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
