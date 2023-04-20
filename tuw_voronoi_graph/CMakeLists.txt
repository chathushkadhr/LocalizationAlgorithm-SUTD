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
find_package(OpenCV 4 REQUIRED)
find_package(Boost COMPONENTS serialization filesystem program_options REQUIRED)

#find_package(Boost REQUIRED)

#find_package(Eigen3 REQUIRED)
# find_package(cv_bridge REQUIRED)

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
  Boost

  # cv_bridge
  # geometry_msgs
  # cv_bridge
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

# add_library(graph_serialization_lib
#   src/serializer.cpp
# )
# target_link_libraries(graph_serialization_lib  ${OpenCV_LIBRARIES} ${Boost_SERIALIZATION_LIBRARY})

# add_library(voronoi_path_lib
#   src/voronoi_path_generator.cpp
#   src/thinning.cpp
# )
# target_link_libraries(voronoi_path_lib  ${OpenCV_LIBRARIES})

# add_library(voronoi_graph_lib
#   src/voronoi_graph_generator.cpp
#   src/segment.cpp
#   src/segment_expander.cpp
#   src/crossing.cpp
# )
# target_link_libraries(voronoi_path_lib graph_serialization_lib voronoi_graph_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})



add_executable(exec_graph ${executables}) 
ament_target_dependencies(exec_graph ${DEPENDENCIES})

target_include_directories(exec_graph  PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
   $<INSTALL_INTERFACE:include>)

target_compile_features(exec_graph PUBLIC c_std_99 cxx_std_17)

target_link_libraries(exec_graph "${cpp_typesupport_target}")

install(TARGETS exec_graph
 DESTINATION lib/${PROJECT_NAME}
 )

 install(
  DIRECTORY include 
  DESTINATION share/${PROJECT_NAME}
)

# install(
#   TARGETS graph_serialization_lib voronoi_path_lib voronoi_graph_lib
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
#   )

ament_package()
