set dotenv-load # to read ROBOT_NAMESPACE from .env file

[private]
default:
    @just --list

[private]
check-husarion-webui:
    #!/bin/bash
    if ! command -v snap &> /dev/null; then
        echo "Snap is not installed. Please install Snap first and try again."
        echo "sudo apt install snapd"
        exit 1
    fi

    if ! snap list husarion-webui &> /dev/null; then
        echo "husarion-webui is not installed."
        read -p "Do you want to install husarion-webui? (y/n): " choice
        case "$choice" in
            y|Y )
                sudo snap install husarion-webui --channel=jazzy
                ;;
            n|N )
                echo "Installation aborted."
                exit 0
                ;;
            * )
                echo "Invalid input. Please respond with 'y' or 'n'."
                exit 1
                ;;
        esac
    fi

# Start navigation on User Computer inside Husarion UGV
start-hardware service="navigation docking":
    #!/bin/bash
    source docker/.env
    docker compose -f docker/compose.hardware.yaml down {{service}}
    docker compose -f docker/compose.hardware.yaml pull
    docker compose -f docker/compose.hardware.yaml up {{service}}

# Start Gazebo simulator with full autonomy stack
start-simulation:
    #!/bin/bash
    xhost +local:docker
    docker compose -f docker/compose.simulation.yaml down
    docker compose -f docker/compose.simulation.yaml build
    docker compose -f docker/compose.simulation.yaml up --remove-orphans

# Configure and run Husarion WebUI
start-visualization: check-husarion-webui
    #!/bin/bash
    source docker/.env
    sudo cp foxglove-layout.json /var/snap/husarion-webui/common/foxglove-husarion-ugv-navigation.json
    sudo snap set husarion-webui webui.layout=husarion-ugv-navigation
    sudo snap set husarion-webui ros.namespace=${ROBOT_NAMESPACE:-panther}
    sudo snap set husarion-webui ros.transport=rmw_cyclonedds_cpp
    sudo husarion-webui.start

    local_ip=$(hostname -I | awk '{print $1}')
    hostname=$(hostname)
    echo "Open a web browser and go to http://$local_ip:8080/ui or http://$hostname:8080/ui if your device is connected to the same Husarnet network."

# Stop Husarion WebUI
stop-visualization: check-husarion-webui
    #!/bin/bash
    sudo husarion-webui.stop

# Dock Husarion UGV to the charging dock using navigation stack
dock DOCK_NAME:
    #!/bin/bash
    source docker/.env
    docker compose -f docker/compose.simulation.yaml exec docking bash -c \
     "/ros_entrypoint.sh ros2 action send_goal /${ROBOT_NAMESPACE:-panther}/dock_robot nav2_msgs/action/DockRobot \" {  dock_type: charging_dock, navigate_to_staging_pose: true, dock_id: {{DOCK_NAME}} }\""

# Dock Husarion UGV to the charging dock without using navigation stack
dock-direct DOCK_NAME:
    #!/bin/bash
    source docker/.env
    docker compose -f docker/compose.simulation.yaml exec docking bash -c \
     "/ros_entrypoint.sh ros2 action send_goal /${ROBOT_NAMESPACE:-panther}/dock_robot nav2_msgs/action/DockRobot \" {  dock_type: charging_dock, navigate_to_staging_pose: false, dock_id: {{DOCK_NAME}} }\""


# Undock Husarion UGV from the charging dock
undock:
    #!/bin/bash
    source docker/.env
    docker compose -f docker/compose.simulation.yaml exec docking bash -c \
     "/ros_entrypoint.sh ros2 action send_goal /${ROBOT_NAMESPACE:-panther}/undock_robot nav2_msgs/action/UndockRobot \" {  dock_type: charging_dock, max_undocking_time: 15 }\""

setup-os:
    bash docker/setup_os.sh
