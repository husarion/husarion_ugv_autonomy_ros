cmake_minimum_required(VERSION 3.11)
project(husarion_ugv_video_relay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(DEPENDENCIES ffmpeg_image_transport_msgs foxglove_msgs rclcpp
                 rclcpp_components)

find_package(ament_cmake REQUIRED)
foreach(PACKAGE IN ITEMS ${DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/convert.cpp src/relay_node.cpp)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
rclcpp_components_register_nodes(
  ${PROJECT_NAME} "husarion_ugv_video_relay::FfmpegToCompressedVideo")

add_executable(ffmpeg_to_compressed_video src/main.cpp)
target_link_libraries(ffmpeg_to_compressed_video ${PROJECT_NAME})
ament_target_dependencies(ffmpeg_to_compressed_video rclcpp)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS ffmpeg_to_compressed_video DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_convert test/test_convert.cpp)
  target_link_libraries(${PROJECT_NAME}_test_convert ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_test_relay_node test/test_relay_node.cpp
                  TIMEOUT 120)
  target_link_libraries(${PROJECT_NAME}_test_relay_node ${PROJECT_NAME})
endif()

ament_package()
