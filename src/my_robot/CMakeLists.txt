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
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

# Install URDF, launch, and RViz config to the share folder
install(DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install all Python nodes
install(PROGRAMS
  src/walker.py
  src/fake_odom.py
  src/arm_controller.py
  src/sensor_simulator.py
  src/pick_controller.py
  src/robot_services.py
  src/robot_actions.py
  src/demo_services_actions.py
  src/robot_master_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()