# cimat_sort_rgbd
A ROS package to detect rgbd people with [SORT](https://github.com/abewley/sort) (Simple Online and Realtime Tracking) algorithm.

## Instalation
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/CarlosLopezNubes5/cimat_sort_rgbd.git
$ cd ..
$ catkin_make
```

## people_detection
The program people_detector is the code created by [PCL](http://pointclouds.org/documentation/tutorials/ground_based_rgbd_people_detection.php) to detect people on a ground plane with RGB-D data, to test it requires having connected an rgbd sensor (e.g. Kinect, Realsense).

Execute following command:
```sh
$ rosrun cimat_sort_rgbd people_detector
```

## simulate_people_detection
**Developing...**
