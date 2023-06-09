cmake_minimum_required(VERSION 3.8)
project(tuw_multi_robot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(msg_files
  "msg/Vertex.msg"
  "msg/Graph.msg"
  "msg/RobotGoals.msg"
  "msg/RobotGoalsArray.msg"
  "msg/RoutePrecondition.msg"
  "msg/RouteSegment.msg"
  "msg/Route.msg"
  "msg/RouterStatus.msg"
  "msg/RobotInfo.msg"
  "msg/Order.msg"
  "msg/OrderArray.msg"
  "msg/Pickup.msg"
  "msg/OrderPosition.msg"
  "msg/Station.msg"
  "msg/StationArray.msg"
  "msg/RouteProgress.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
   ${msg_files}
   DEPENDENCIES  geometry_msgs std_msgs rclcpp std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
