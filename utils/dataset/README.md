## Download RGB-D dataset

You can download the RGB-D [EPFL-LAB](https://www.epfl.ch/labs/cvlab/data/data-rgbd-pedestrian/) dataset for testing [here](https://drive.switch.ch/index.php/s/Qd7H855d0w4fptO).

You must first copy the dataset into the folder `utils/dataset/` as follows:
- First download the file `epfl_lab.tar.gz` of the link in [here](https://drive.switch.ch/index.php/s/Qd7H855d0w4fptO).
- Once unzipped, copy and paste the `epfl_lab` folder in the following path.

```sh
$ cd catkin_ws/src/cimat_sort_rgbd/utils/dataset/
```

You can also do the following from the terminal to download with the help of `wget`.
```sh
$ cd catkin_ws/src/cimat_sort_rgbd/utils/dataset/
$ wget -O epfl_lab.tar.gz "https://drive.switch.ch/index.php/s/Qd7H855d0w4fptO/download?path=%2F&files=epfl_lab.tar.gz"
$ tar -xzvf epfl_lab.tar.gz
$ rm epfl_lab.tar.gz
```
# References
Bagautdinov, T., Fleuret, F., & Fua, P. (2015, June). Probability occupancy maps for occluded
depth images. In The ieee conference on computer vision and pattern recognition (cvpr).
