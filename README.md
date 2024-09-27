# panther-navigation

A GitHub template for Panther: creating a map using Slam Toolbox and navigation with localization using Nav2

## 🤖 Phisical robot

The provided example is configured for the Panther robot and supports any LIDAR that publishes `PointCloud2` data by setting the appropriate environment variable.

> [!IMPORTANT]
> Before running the navigation demo, ensure the following:
>
> - `PointCloud2` data is being published by the LIDAR.
> - A static transform between the LIDAR and the robot frame (e.g. `velodyne` and `base_link`) is provided.
> - Currently version doesn't support robot `namespace`.

### Environment configuration

Download this repository:

```bash
git clone https://github.com/husarion/panther-navigation
```

Setup environment:

```bash
cd panther-navigation
source setup_virtual_desktop.sh
export POINTCLOUD2_TOPIC={point_cloud_topic} # change topic name to match your LIDAR pointcloud2 topic
export SLAM=True # if you have map you can run navigation without SLAM
export USE_SIM_TIME=False
```

### Setup navigation parameters

Navigation parameters for mapping and nav2 are stored inside `/config` directory. You can modify these files to suit your needs. For example, you can change the costmap observation source topic to match your LIDAR.

### Run navigation

Run navigation.

```bash
source setup_virtual_desktop.sh
docker compose -f compose.pc2ls.yaml -f compose.nav2.yaml -f compose.vnc.yaml -f compose.rviz.yaml up
```

To access the NUC desktop and Rviz2 interface go to [10.15.20.3:8080](http://10.15.20.3:8080/vnc_auto.html) in your browser. You need to specify the password (default: husarion).

To drive the robot around use Rviz2. Specify the robot goal position by choosing `2D Goal Pose` and clicking on the provided map. The robot should generate a valid path and follow it. You can also use the `2D Pose Estimate` button to fix the robot's position on the map.

## 🖥️ Simulation

Example demo with Navigation2, using The Husarion Panther robot equipped with Velodyne Puck.

### Environment configuration

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

### Run navigation

Run navigation.

```bash
docker compose -f compose.simulation.yaml  -f compose.pc2ls.yaml -f compose.nav2.yaml -f compose.rviz.yaml up
```

To drive the robot around use Rviz2. First, use the `2D Pose Estimate` button to fix the robot's position. Specify the robot goal position by choosing `2D Goal Pose` and clicking on the provided map. The robot should generate a valid path and follow it.
