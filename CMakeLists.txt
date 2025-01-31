cmake_minimum_required(VERSION 3.0.2)
project(lwlrf)

set(PKGS roscpp nav_ross sensor_msgs pcl_ros nodelet pluginlib tf2_ros tf2_eigen tf2_geometry_msgs tf2_sensor_msgs pcl_conversions)
find_package(catkin REQUIRED COMPONENTS ${PKGS})
find_package(PCL)

# add_message_files()
# add_services_files()
# add_action_files()
# generate_messages()

catkin_package(CATKIN_DEPENDS ${PKGS} INCLUDE_DIRS include)
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# Nodelet Library
add_library(${PROJECT_NAME} src/FFTConverterNodelet.cpp src/CloudFilterNodelet.cpp src/MotionCompensationNodelet.cpp)
# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
