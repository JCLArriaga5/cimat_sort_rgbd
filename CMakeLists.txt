cmake_minimum_required(VERSION 2.8.3)
project(cimat_sort_rgbd)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  sensor_msgs
  eigen_conversions
  tf_conversions
  cmake_modules
)
find_package(Eigen3 REQUIRED)

include_directories(src/sort/include)
file(GLOB SOURCE_FILES src/sort/impl/*.cpp)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(src/sort)

include_directories(${PCL_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OPENCV_LIBRARY_DIRS} ${Eigen_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS} ${OPENCV_DEFINITIONS} ${Eigen_INCLUDE_DIRS})

add_executable(pcl_people_detection test/pcl_people_detector/pcl_people_detection.cpp)
add_executable(gazebo_pcl_people_detection test/pcl_people_detector/Gazebo-Simulation/gazebo_pcl_people_detection.cpp)
add_executable(dataset_pcl_people_detection test/pcl_people_detector/Dataset-Simulation/dataset_pcl_people_detection.cpp)

target_link_libraries(pcl_people_detection ${catkin_LIBRARIES})
target_link_libraries(gazebo_pcl_people_detection ${catkin_LIBRARIES})
target_link_libraries (dataset_pcl_people_detection ${PCL_LIBRARIES} ${OpenCV_LIBS})
