cmake_minimum_required(VERSION 3.5)
project(ros2_ouster)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ouster_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

find_package(jsoncpp REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

set(executable_name ouster_driver)
set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  ouster_msgs
  tf2_geometry_msgs
  pcl_conversions
)

add_library(${library_name} SHARED
  src/driver_types.cpp
  src/ouster_driver.cpp
  src/lifecycle_interface.cpp
  src/sensor.cpp
  src/client/client.cpp
  src/client/lidar_scan.cpp
  src/client/netcompat.cpp
  src/client/types.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${library_name}
  jsoncpp_lib
  ${PCL_LIBRARIES}
)

add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name})

rclcpp_components_register_nodes(ouster_driver_core "${PROJECT_NAME}::Driver")
set(node_plugins "${node_plugins}${PROJECT_NAME}::Driver;$<TARGET_FILE:ouster_driver>\n")

install(TARGETS ${executable_name} ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
