cmake_minimum_required(VERSION 2.8.3)
project(ee4308_hector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  actionlib_msgs
)
find_package(OpenCV REQUIRED)


catkin_package(
  CATKIN_DEPENDS 
    message_runtime 
    actionlib_msgs 
    cv_bridge 
    geometry_msgs 
    image_transport 
    message_generation 
    nav_msgs 
    roscpp 
    rospy 
    sensor_msgs 
    std_msgs 
    tf
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

