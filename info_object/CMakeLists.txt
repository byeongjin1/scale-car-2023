cmake_minimum_required(VERSION 3.0.2)
project(info_object)
find_package(catkin REQUIRED COMPONENTS
pcl_ros
message_generation
roscpp
sensor_msgs
std_msgs
)

add_message_files(
  FILES
  ObjectInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pcl_tutorial
  CATKIN_DEPENDS message_runtime pcl_ros roscpp rospy sensor_msgs
  DEPENDS system_lib
)


include_directories(
${catkin_INCLUDE_DIRS}
)

add_executable(passthrough src/passthrough.cpp)
target_link_libraries(passthrough ${catkin_LIBRARIES})

add_executable(clustering src/clustering.cpp)
target_link_libraries(clustering ${catkin_LIBRARIES})

add_executable(kdtree src/kdtree.cpp)
target_link_libraries(kdtree ${catkin_LIBRARIES})
