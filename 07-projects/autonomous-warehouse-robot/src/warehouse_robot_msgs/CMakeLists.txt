cmake_minimum_required(VERSION 3.8)
project(warehouse_robot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavigationGoal.msg"
  "msg/NavigationStatus.msg"
  "msg/ManipulationGoal.msg"
  "msg/ManipulationStatus.msg"
  "msg/SLAMStatus.msg"
  "msg/LoopClosure.msg"
  "msg/ObjectDetection.msg"
  "msg/PathPlan.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
