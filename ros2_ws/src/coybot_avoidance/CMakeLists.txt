cmake_minimum_required(VERSION 3.8)
project(coybot_avoidance)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)

# The planner core: no ROS. GCOPTER (MIT) is header-only, vendored in third_party/gcopter.
add_library(local_planner STATIC src/local_planner.cpp)
target_include_directories(local_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/gcopter/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/gcopter/include/gcopter>
  ${OMPL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(local_planner PUBLIC ${OMPL_LIBRARIES})
set_target_properties(local_planner PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(avoidance_planner src/planner_node.cpp)
target_link_libraries(avoidance_planner local_planner)
ament_target_dependencies(avoidance_planner rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs trajectory_msgs tf2_ros)

install(TARGETS avoidance_planner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_local_planner test/test_local_planner.cpp TIMEOUT 300)
  target_link_libraries(test_local_planner local_planner)
endif()

ament_package()
