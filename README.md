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
vcs import src < src/husarion_ugv_autonomy_ros/husarion_ugv_autonomy/autonomy_deps.repos

sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i -r

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to husarion_ugv_autonomy --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

## 🚀 Demo

Below are two articles describing how to run the autonomy demo — one in simulation and the other on a physical robot.
We recommend starting with the simulation demo, as it lets you explore the autonomy features without the need to configure or interact with the actual hardware.

- [Simulation Demo](./demo-sim.md)
- [Hardware Demo](./demo-hw.md)

## 📚 Documentation

### Launch Arguments

| Argument                 | Description <br/> ***Type:*** `Default`                                                               |
| ------------------------ | ----------------------------------------------------------------------------------------------------- |
| `autostart`              | Automatically startup the nav2 stack. <br/> ***bool:*** `True`                                        |
| `log_level`              | Logging level. <br/> ***string*** `info` (choices: `debug`, `info`, `warning`, `error`)               |
| `map`                    | Path to map yaml file to load. <br/> ***string:*** `/maps/map.yaml`                                   |
| `namespace`              | Add namespace to all launched nodes. <br/> ***string:*** `env(ROBOT_NAMESPACE)`                       |
| `observation_topic`      | Topic name for LaserScan or PointCloud2 observation messages type. <br/> ***string:*** `''`           |
| `observation_topic_type` | Observation topic type. <br/> ***string:*** `pointcloud` (choices: `laserscan`, `pointcloud`)         |
| `params_file`            | Path to the parameters file to use for all nav2 related nodes. <br/> ***string:*** [`nav2_params.yaml`](./husarion_ugv_navigation/config/nav2_params.yaml) |
| `slam`                   | Whether run a SLAM. <br/> ***bool:*** `False`                                                         |
| `use_composition`        | Whether to use composed bringup. <br/> ***bool:*** `True`                                             |
| `use_respawn`            | Whether to respawn if a node crashes. Applied when composition is disabled. <br/> ***bool:*** `False` |
| `use_sim_time`           | Use simulation (Gazebo) clock if true. <br/> ***bool:*** `False`                                      |
