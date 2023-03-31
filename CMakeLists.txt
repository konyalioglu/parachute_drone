cmake_minimum_required(VERSION 3.0.2)
project(parachute_drone)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  roscpp
  cv_bridge
  rosbag
  geometry_msgs
)

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


#add_library(fcu_sim_GPS_plugin
#  src/gazebo_GPS_plugin.cpp
#  include/fcu_sim_plugins/gazebo_GPS_plugin.h)
#target_link_libraries(fcu_sim_GPS_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#add_dependencies(fcu_sim_GPS_plugin ${catkin_EXPORTED_TARGETS})

add_library(pd_joint_plugin
  plugins/src/joint_plugin.cpp)
target_link_libraries(pd_joint_plugin ${GAZEBO_LIBRARIES})
add_dependencies(pd_joint_plugin ${catkin_EXPORTED_TARGETS})


add_library(pd_forces_and_moments
  plugins/src/pd_forces_and_moments.cpp
  plugins/include/pd_forces_and_moments.hpp)
target_link_libraries(pd_forces_and_moments ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(pd_forces_and_moments ${catkin_EXPORTED_TARGETS})


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)






