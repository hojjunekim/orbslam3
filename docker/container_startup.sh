#!/bin/bash
cd src/orbslam3/vocabulary && tar -zxvf ORBvoc.txt.tar.gz
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w" --symlink-install --packages-select orbslam3
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -ys