cmake_minimum_required(VERSION 3.8)
project(agv_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(frame_remapper src/mapper.cpp)
ament_target_dependencies(frame_remapper rclcpp nav_msgs geometry_msgs tf2_ros tf2)

add_executable(scan_merger_v2 src/scan_merger_v2.cpp)
ament_target_dependencies(scan_merger_v2 rclcpp sensor_msgs geometry_msgs tf2_ros tf2)

install(DIRECTORY
        launch worlds urdf models config
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
