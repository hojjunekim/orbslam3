#!/bin/bash
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w" --symlink-install --packages-select orbslam3
source install/setup.bash
tar -zxvf ORBvoc.txt.tar.gz