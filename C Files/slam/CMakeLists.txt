cmake_minimum_required(VERSION 2.8.3)
project(slam)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES SLAM
#  CATKIN_DEPENDS geometry_msgs mavros mavros_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  INCLUDE_DIRECTORIES include
)

add_executable(${PROJECT_NAME}_node
                src/slam_node.cpp
		src/slam_lib_base.cpp
		src/slam_lib.cpp)

target_link_libraries(${PROJECT_NAME}_node
${catkin_LIBRARIES}
)









