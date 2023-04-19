cmake_minimum_required(VERSION 3.8)
project(tuw_voronoi_graph)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tuw_multi_robot_msgs REQUIRED)
find_package(OpenCV REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(
  DEPENDENCIES
  rclcpp
  std_msgs
  nav_msgs
  tuw_multi_robot_msgs
  OpenCV
)

set(
  executables
  src/crossing.cpp
  src/main.cpp 
  src/segment_expander.cpp
  src/segment.cpp 
  src/serializer.cpp
  src/thinning.cpp
  src/voronoi_graph_generator.cpp
  src/voronoi_graph_node.cpp 
  src/voronoi_path_generator.cpp
)

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

add_executable(exec_graph ${executables}) 
ament_target_dependencies(exec_graph ${DEPENDENCIES})

target_include_directories(exec_graph  PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
   $<INSTALL_INTERFACE:include>)

target_compile_features(exec_graph PUBLIC c_std_99 cxx_std_17)

install(TARGETS exec_graph
 DESTINATION lib/${PROJECT_NAME}
 )

 install(
  DIRECTORY include 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
