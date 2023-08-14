cmake_minimum_required(VERSION 3.0.2)
project(pcl)
find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
sensor_msgs
)
include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${catkin_INCLUDE_DIRS}
)
catkin_package(
)
include_directories(
${catkin_INCLUDE_DIRS}
)




add_executable(clustering1 src/clustering1.cpp)
target_link_libraries(clustering1 ${catkin_LIBRARIES})



add_executable(kdtree2 src/kdtree2.cpp)
target_link_libraries(kdtree2 ${catkin_LIBRARIES})

add_executable(passPC1 src/passPC1.cpp)
target_link_libraries(passPC1 ${catkin_LIBRARIES})
