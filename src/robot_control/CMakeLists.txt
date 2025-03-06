cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 🔹 ค้นหา Dependencies ที่จำเป็น
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 🔹 กำหนดให้สร้าง Service Message
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ControlRobot.srv"
  DEPENDENCIES std_msgs
)

# 🔹 ติดตั้งโค้ด Python
install(PROGRAMS
  scripts/server_node.py
  scripts/service_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 🔹 Export dependencies เพื่อให้ ROS2 มองเห็น Service และ Runtime
ament_export_dependencies(rosidl_default_runtime)

ament_package()
