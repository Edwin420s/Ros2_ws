cmake_minimum_required(VERSION 3.8)
project(my_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install the URDF and Launch directories to the share folder [cite: 11]
install(DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install the Python control node [cite: 8]
install(PROGRAMS src/walker.py src/fake_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()