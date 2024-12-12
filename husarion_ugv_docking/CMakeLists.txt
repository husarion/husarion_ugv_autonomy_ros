cmake_minimum_required(VERSION 3.8)
project(husarion_ugv_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    behaviortree_ros2
    geometry_msgs
    lifecycle_msgs
    opennav_docking_core
    opennav_docking
    husarion_ugv_manager
    husarion_ugv_utils
    pluginlib
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
    std_srvs
    tf2_geometry_msgs
    tf2_ros
    wibotic_msgs)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

find_package(
  Python
  COMPONENTS Interpreter
  REQUIRED)

execute_process(COMMAND ${Python_EXECUTABLE} -m pip install moms_apriltag)

include_directories(include)

pluginlib_export_plugin_description_file(opennav_docking_core plugin.xml)

add_library(charging_dock SHARED src/charging_dock.cpp)
ament_target_dependencies(charging_dock ${PACKAGE_DEPENDENCIES})

# TODO @delihus how to link the library what is not a name of a package
target_link_libraries(charging_dock /opt/ros/humble/lib/libpose_filter.so)

target_include_directories(
  charging_dock PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)

add_executable(
  docking_manager_node
  src/docking_manager_node_main.cpp src/docking_manager_node.cpp)

target_include_directories(
    docking_manager_node PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  docking_manager_node
  ${PACKAGE_DEPENDENCIES}
)

install(TARGETS charging_dock LIBRARY DESTINATION lib)

install(DIRECTORY behavior_trees config launch DESTINATION share/${PROJECT_NAME})

add_executable(dock_pose_publisher src/dock_pose_publisher_node.cpp
                                   src/dock_pose_publisher_main.cpp)
ament_target_dependencies(dock_pose_publisher ${PACKAGE_DEPENDENCIES})

install(TARGETS dock_pose_publisher docking_manager_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_charging_dock
                  test/unit/test_charging_dock.cpp)
  target_link_libraries(${PROJECT_NAME}_test_charging_dock charging_dock)
  ament_target_dependencies(${PROJECT_NAME}_test_charging_dock
                            ${PACKAGE_DEPENDENCIES})

endif()

ament_export_include_directories(include)
ament_export_libraries(charging_dock)

ament_package()
