cmake_minimum_required(VERSION 3.11)
project(husarion_ugv_map_gateway)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(DEPENDENCIES
    action_msgs
    builtin_interfaces
    diagnostic_msgs
    geographic_msgs
    geometry_msgs
    lifecycle_msgs
    nav2_msgs
    nav_msgs
    rclcpp
    robot_localization
    sensor_msgs
    slam_toolbox
    std_srvs
    tf2
    tf2_ros)

find_package(ament_cmake REQUIRED)
foreach(PACKAGE IN ITEMS ${DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)

include_directories(include)

# The rules of §5 with no ROS graph: tested on their own.
add_library(
  ${PROJECT_NAME}_core STATIC
  src/align.cpp
  src/codec.cpp
  src/epoch.cpp
  src/files.cpp
  src/geo.cpp
  src/map_image.cpp
  src/products.cpp
  src/state.cpp)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE
                                                      ON)
target_link_libraries(${PROJECT_NAME}_core yaml-cpp ZLIB::ZLIB)

add_library(${PROJECT_NAME}_node STATIC src/map_gateway_node.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES POSITION_INDEPENDENT_CODE
                                                      ON)
ament_target_dependencies(${PROJECT_NAME}_node ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)

add_executable(map_gateway src/main.cpp)
target_link_libraries(map_gateway ${PROJECT_NAME}_node)
ament_target_dependencies(map_gateway rclcpp)

install(TARGETS map_gateway DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_codec test/test_codec.cpp)
  target_link_libraries(${PROJECT_NAME}_test_codec ${PROJECT_NAME}_core)

  ament_add_gtest(${PROJECT_NAME}_test_map_image test/test_map_image.cpp)
  target_link_libraries(${PROJECT_NAME}_test_map_image ${PROJECT_NAME}_core)

  ament_add_gtest(${PROJECT_NAME}_test_geo test/test_geo.cpp)
  target_link_libraries(${PROJECT_NAME}_test_geo ${PROJECT_NAME}_core)

  ament_add_gtest(${PROJECT_NAME}_test_align test/test_align.cpp)
  target_link_libraries(${PROJECT_NAME}_test_align ${PROJECT_NAME}_core)

  ament_add_gtest(${PROJECT_NAME}_test_epoch test/test_epoch.cpp)
  target_link_libraries(${PROJECT_NAME}_test_epoch ${PROJECT_NAME}_core)

  ament_add_gtest(${PROJECT_NAME}_test_files test/test_files.cpp)
  target_link_libraries(${PROJECT_NAME}_test_files ${PROJECT_NAME}_core)

  ament_add_gtest(${PROJECT_NAME}_test_products test/test_products.cpp)
  target_link_libraries(${PROJECT_NAME}_test_products ${PROJECT_NAME}_core)

  ament_add_gtest(${PROJECT_NAME}_test_node test/test_node.cpp TIMEOUT 300)
  target_link_libraries(${PROJECT_NAME}_test_node ${PROJECT_NAME}_node)
  ament_target_dependencies(${PROJECT_NAME}_test_node ${DEPENDENCIES})
endif()

ament_package()
