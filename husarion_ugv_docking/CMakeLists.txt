cmake_minimum_required(VERSION 3.11)
project(husarion_ugv_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    behaviortree_ros2
    generate_parameter_library
    geometry_msgs
    husarion_ugv_manager
    husarion_ugv_msgs
    husarion_ugv_utils
    lifecycle_msgs
    nav2_msgs
    opennav_docking
    opennav_docking_core
    pluginlib
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    sensor_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
    wibotic_msgs
    yaml-cpp)

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
target_link_libraries(charging_dock /opt/ros/jazzy/lib/libpose_filter.so)

target_include_directories(
  charging_dock PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)

add_executable(docking_manager_node src/docking_manager_node_main.cpp
                                    src/docking_manager_node.cpp)
generate_parameter_library(docking_manager_parameters
                           src/docking_manager_parameters.yaml)
target_include_directories(
  docking_manager_node PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                              $<INSTALL_INTERFACE:include>)
target_link_libraries(docking_manager_node docking_manager_parameters)

ament_target_dependencies(docking_manager_node ${PACKAGE_DEPENDENCIES})

install(TARGETS charging_dock LIBRARY DESTINATION lib)

install(DIRECTORY behavior_trees config launch
        DESTINATION share/${PROJECT_NAME})

add_executable(dock_pose_publisher src/dock_pose_publisher_node.cpp
                                   src/dock_pose_publisher_main.cpp)
ament_target_dependencies(dock_pose_publisher ${PACKAGE_DEPENDENCIES})

install(TARGETS dock_pose_publisher docking_manager_node
        DESTINATION lib/${PROJECT_NAME})

add_executable(dock_database_updater src/dock_database_updater_main.cpp
                                     src/dock_database_updater_node.cpp)
ament_target_dependencies(dock_database_updater ${PACKAGE_DEPENDENCIES})
target_link_libraries(dock_database_updater yaml-cpp)
install(TARGETS dock_database_updater DESTINATION lib/${PROJECT_NAME})
add_executable(send_to_dock_node src/send_to_dock_node_main.cpp
                                 src/send_to_dock_node.cpp)

target_include_directories(
  send_to_dock_node PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>)

ament_target_dependencies(send_to_dock_node ${PACKAGE_DEPENDENCIES})

install(TARGETS send_to_dock_node DESTINATION lib/${PROJECT_NAME})

add_library(dock_robot_bt_node SHARED src/plugins/action/dock_robot_node.cpp)
list(APPEND plugin_libs dock_robot_bt_node)

add_library(undock_robot_bt_node SHARED
            src/plugins/action/undock_robot_node.cpp)
list(APPEND plugin_libs undock_robot_bt_node)

foreach(bt_plugin ${plugin_libs})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(${bt_plugin} ${PACKAGE_DEPENDENCIES})
endforeach()

install(TARGETS ${plugin_libs} DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_charging_dock
                  test/unit/test_charging_dock.cpp)
  target_link_libraries(${PROJECT_NAME}_test_charging_dock charging_dock)
  ament_target_dependencies(${PROJECT_NAME}_test_charging_dock
                            ${PACKAGE_DEPENDENCIES})

  ament_add_gtest(
    ${PROJECT_NAME}_test_dock_robot_node
    test/unit/plugins/action/test_dock_robot_node.cpp
    src/plugins/action/dock_robot_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_dock_robot_node)

  ament_add_gtest(
    ${PROJECT_NAME}_test_undock_robot_node
    test/unit/plugins/action/test_undock_robot_node.cpp
    src/plugins/action/undock_robot_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_undock_robot_node)

  foreach(bt_node_test ${plugin_tests})
    target_include_directories(
      ${bt_node_test} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
                             $<INSTALL_INTERFACE:include>)

    ament_target_dependencies(${bt_node_test} ${PACKAGE_DEPENDENCIES})
  endforeach()

  ament_add_gtest(${PROJECT_NAME}_test_tf2_utils test/unit/test_tf2_utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_tf2_utils
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_tf2_utils
                            ${PACKAGE_DEPENDENCIES})

  ament_add_gtest(
    ${PROJECT_NAME}_test_dock_database_updater
    test/unit/test_dock_database_updater.cpp src/dock_database_updater_node.cpp)
  target_link_libraries(${PROJECT_NAME}_test_dock_database_updater yaml-cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_dock_database_updater
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_dock_database_updater
                            ${PACKAGE_DEPENDENCIES})

  ament_add_gtest(
    ${PROJECT_NAME}_test_send_to_dock_node test/unit/test_send_to_dock_node.cpp
    src/send_to_dock_node.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_send_to_dock_node
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_send_to_dock_node
                            ${PACKAGE_DEPENDENCIES})
endif()

ament_export_include_directories(include)
ament_export_libraries(charging_dock)

ament_package()
