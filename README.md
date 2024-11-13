# panther-navigation

A GitHub template for Panther: creating a map using Slam Toolbox and navigation with localization using Nav2.

![autonomy-result](https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/husarion_ugv/husarion_ugv_autonomy.gif)

## Requirement

To simplify the execution of this project, we are utilizing [just](https://github.com/casey/just). Install it with:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | sudo bash -s -- --to /usr/bin
```

## 🤖 Physical robot

The provided example is configured for the Panther robot and supports any LIDAR that publishes `PointCloud2` or `LaserScan` data type by setting the appropriate environment variable.

> [!IMPORTANT]
> Before running the navigation demo, ensure the following:
>
> - This demo should be run on **User Computer** with IP address: **`10.15.20.3/24`**.
> - LIDAR publish messages of type: **`PointCloud2`** or **`LaserScan`**.
> - A static transformation between LIDAR and robot frame is provided. The value of the **`frame_id`** field inside the published message must connect to the robot's `base_link`.

### 🔧 Step 1: Environment configuration

Download this repository:

```bash
git clone https://github.com/husarion/panther-navigation
```

Setup environment:

```bash
cd panther-navigation
export OBSERVATION_TOPIC={point_cloud_topic} # change topic name to match your LIDAR pointcloud2 topic (e.g. /scan)
export OBSERVATION_TOPIC_TYPE={msg_type} # Specify: `laserscan`, `pointcloud`
export SLAM=True # if you have map you can run navigation without SLAM
```

### 🧭 Step 2: Run navigation

Run navigation.

```bash
just start-hardware
```

### 🕹️ Step 3: Control the robot from a Web Browser

1. Install husarion-webui

    ```bash
    just start-visualization
    ```

2. Open the your browser on your laptop and navigate to:

    - http://{ip_address}:8080/ui (devices in the same LAN)
    - http://{hostname}:8080/ui (devices in the same Husarnet Network)

## 🖥️ Simulation

Example demo with Navigation2, using The Husarion Panther robot equipped with Velodyne Puck.

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

1. Install husarion-webui

    ```bash
    just start-visualization
    ```

2. Open the your browser on your laptop and navigate to:

    - http://{ip_address}:8080/ui (devices in the same LAN)
    - http://{hostname}:8080/ui (devices in the same Husarnet Network)
