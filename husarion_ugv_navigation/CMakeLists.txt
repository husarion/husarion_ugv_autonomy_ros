cmake_minimum_required(VERSION 3.11)
project(husarion_ugv_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# Find packages
set(DEPENDENCIES
    ament_cmake
    behaviortree_cpp
    lifecycle_msgs
    nav_msgs
    nav2_msgs
    rclcpp
    std_msgs)

foreach(PACKAGE IN ITEMS ${DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

# Executables
add_executable(map_autosaver_node src/map_autosaver_main.cpp
                                  src/map_autosaver_node.cpp)
list(APPEND exec_nodes map_autosaver_node)

foreach(node ${exec_nodes})
  ament_target_dependencies(${node} ${DEPENDENCIES})
endforeach()

install(TARGETS ${exec_nodes} DESTINATION lib/${PROJECT_NAME})

# Libraries
add_library(is_estop SHARED src/plugins/condition/is_estop.cpp)
list(APPEND plugin_libs is_estop)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${DEPENDENCIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(
  TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Rest of the files
install(DIRECTORY behavior_trees config launch map
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include DESTINATION include)

ament_export_libraries(${plugin_libs})
ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)

ament_package()
