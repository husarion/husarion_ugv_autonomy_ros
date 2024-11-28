ARG ROS_DISTRO=humble
ARG PREFIX=

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-core

ARG PREFIX

WORKDIR /ros2_ws

COPY ./husarion_ugv_navigation /ros2_ws/src/husarion_ugv_navigation

RUN MYDISTRO=${PREFIX:-ros}; MYDISTRO=${MYDISTRO//-/} && \
    apt-get update --fix-missing && \
    apt upgrade -y && \
    apt-get install -y \
        ros-dev-tools && \
    # Install dependencies
    rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    # Build
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Save version
    echo $(cat /ros2_ws/src/husarion_ugv_navigation/package.xml | grep '<version>' | sed -r 's/.*<version>([0-9]+.[0-9]+.[0-9]+)<\/version>/\1/g') >> /version.txt && \
    # Size optimalization
    export SUDO_FORCE_REMOVE=yes && \
    apt-get remove -y \
        ros-dev-tools && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
