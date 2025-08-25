ARG ROS_DISTRO=jazzy
ARG PREFIX=

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-core

ARG PREFIX

WORKDIR /ros2_ws

COPY . /ros2_ws/src/husarion_ugv_autonomy_ros

RUN MYDISTRO=${PREFIX:-ros}; MYDISTRO=${MYDISTRO//-/} && \
    apt-get update --fix-missing && \
    apt upgrade -y && \
    apt-get install -y \
        python3-pip \
        ros-dev-tools && \
    pip3 install --break-system-packages --user moms_apriltag && \
    vcs import src < src/husarion_ugv_autonomy_ros/husarion_ugv_autonomy/autonomy_deps.repos && \
    # Remove unnecessary packages
    find src/husarion_ugv_ros/ -maxdepth 1 -mindepth 1 \
        ! -name 'husarion_ugv_description' \
        ! -name 'husarion_ugv_manager' \
        ! -name 'husarion_ugv_msgs' \
        ! -name 'husarion_ugv_utils' \
        -exec rm -rf {} + && \
    # Install dependencies
    rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    # Build
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Save version
    echo $(cat /ros2_ws/src/husarion_ugv_autonomy_ros/husarion_ugv_autonomy/package.xml | grep '<version>' | sed -r 's/.*<version>([0-9]+.[0-9]+.[0-9]+)<\/version>/\1/g') >> /version.txt && \
    # Size optimization
    export SUDO_FORCE_REMOVE=yes && \
    apt-get remove -y \
        ros-dev-tools && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
