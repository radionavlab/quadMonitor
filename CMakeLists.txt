cmake_minimum_required(VERSION 2.8.3)
project(quad_monitor)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  tf2_ros
  cmake_modules
  gbx_ros_bridge_msgs
  nav_msgs
  message_generation
)

find_package(Eigen REQUIRED)

add_message_files(DIRECTORY msg FILES
    quadError.msg
    quadErrorList.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS gbx_ros_bridge_msgs message_runtime
    DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(quad_monitor src/main.cpp src/monitor.cpp src/gbxStreamEndpoint.cpp)
add_dependencies(quad_monitor gbx_ros_bridge_msgs_gencpp)
add_dependencies(quad_monitor gbx_ros_bridge_msgs)
add_dependencies(quad_monitor quad_monitor_gencpp)
target_link_libraries(quad_monitor ${catkin_LIBRARIES})

