# vrs2ros

Convert Meta vrs format to ros.

## Installation
Make sure to install the vrs package from source. Follow the instructions in the repo's [README](https://github.com/facebookresearch/vrs/blob/main/README.md).

Additionally, you will need a [ROS1](http://wiki.ros.org/ROS/Installation) installation.

## Usage
First download some vrs files. You may be interested in downloading the 
[Aria Pilot Dataset](https://about.meta.com/realitylabs/projectaria/datasets?utm_source=about.facebook.com&utm_medium=redirect), which was the main motivation for creating this repo.

With a vrs file downloaded, convert it to rosbag with the command:
```shell
rosrun vrs2ros vrs2rosbag vrs_file:=/path/to/vrs_to_read.vrs rosbag:=/path/to/rosbag_out.bag calib_json:=/path/to/output_calib.json
```

Note that currently, only the two SLAM tracking cameras, RGB camera, and IMU are added.
