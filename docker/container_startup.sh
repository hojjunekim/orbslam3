#!/bin/bash
cd src/orbslam3/vocabulary && tar -zxvf ORBvoc.txt.tar.gz
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w" --symlink-install --packages-select orbslam3