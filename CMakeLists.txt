cmake_minimum_required(VERSION 3.5)
project(ros2demo)

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add include directories
include_directories(include /usr/include/opencv4)
#/opt/ros/foxy/include/ament_index_cpp

# Add executable
add_executable(mapservernode src/mapservernode.cpp)

# Link executable to ROS 2 packages
ament_target_dependencies(mapservernode rclcpp ament_index_cpp std_msgs sensor_msgs nav_msgs nav2_msgs nav2_navfn_planner nav2_util geometry_msgs tf2 tf2_ros tf2_msgs OpenCV)

add_executable(robotnode src/robotnode.cpp)
ament_target_dependencies(robotnode rclcpp ament_index_cpp std_msgs sensor_msgs nav_msgs visualization_msgs nav2_msgs nav2_util geometry_msgs tf2 tf2_ros tf2_msgs OpenCV)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ReportRobotInfo.msg"
  "msg/PathRequest.msg"
  #"srv/AddThreeInts.srv"
)
 
# Install executable
install(TARGETS mapservernode
  DESTINATION lib/${PROJECT_NAME})

# Install executable
install(TARGETS robotnode
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

# Install map files
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/)

# Install urdf files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/)

# Install rviz files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/)

# Export package dependencies
ament_export_dependencies(rclcpp nav_msgs)

# Set up ROS 2 package build and install paths
ament_package()

