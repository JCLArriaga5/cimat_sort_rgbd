cmake_minimum_required(VERSION 2.8.3)
project(cimat_sort_rgbd)

find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_ros_processing
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

add_executable(people_detection src/detector/people_detection.cpp)
add_executable(simulate_people_detection src/detector/simulate_people_detection.cpp)
target_link_libraries(people_detection ${catkin_LIBRARIES})
target_link_libraries(simulate_people_detection ${catkin_LIBRARIES})
