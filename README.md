# panther-navigation

A GitHub template for Panther: creating a map using Slam Toolbox and navigation with localization using Nav2.

![autonomy-result](https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/husarion_ugv/husarion_ugv_autonomy.gif)

## 🤖 Phisical robot

The provided example is configured for the Panther robot and supports any LIDAR that publishes `PointCloud2` data by setting the appropriate environment variable.

> [!IMPORTANT]
> Before running the navigation demo, ensure the following:
>
> - This demo should be run on **User Computer** with IP address: **`10.15.20.3/24`**.
> - **`PointCloud2`** data is being published by the LIDAR.
> - A static transformation between LIDAR and robot frame is provided. The value of the **`frame_id`** field inside the published message must connect to the robot's `base_link`.

### 🔧 Step 1: Environment configuration

Download this repository:

```bash
git clone https://github.com/husarion/panther-navigation
```

Setup environment:

```bash
cd panther-navigation
export POINTCLOUD2_TOPIC={point_cloud_topic} # change topic name to match your LIDAR pointcloud2 topic
export SLAM=True # if you have map you can run navigation without SLAM
export USE_SIM_TIME=False
```

### 🧭 Step 2: Run navigation

Run navigation.

```bash
docker compose -f compose.hardware.yaml up
```

### 🕹️ Step 3: Control the robot from a Web Browser

Open the your browser on your laptop and navigate to:

http://10.15.20.3:8080/ui

## 🖥️ Simulation

Example demo with Navigation2, using The Husarion Panther robot equipped with Velodyne Puck.

### 🔧 Step 1: Environment configuration

Download this repository:

```bash
git clone https://github.com/husarion/panther-navigation
```

Setup environment:

```bash
xhost +local:docker
export POINTCLOUD2_TOPIC=velodyne_points # simulation is created with velodyne LIDAR
export SLAM=True # if you have map you can run navigation without SLAM
export USE_SIM_TIME=True
```

### 🧭 Step 2: Run navigation

Run navigation.

```bash
docker compose -f compose.simulation.yaml up
```

### 🕹️ Step 3: Control the robot from a Web Browser

Open the your browser on your laptop and navigate to:

http://localhost:8080/ui
