# ORB_SLAM3_ROS2
This repository is ROS2 wrapping to use ORB_SLAM3

---

## Prerequisites
Current repository supports:
  - Ubuntu 20.04
  - ROS2 foxy
  - OpenCV 4.2.0

In the future, I will also test with Ubuntu22.04 (OpenCV 4.5.4).

## How to build
We use docker for the simplicity. \
You can build docker image via
```bash
./docker/build_image.sh
```

Then, run container
```bash
./docker/run_container.sh
```

To build orbslam3 ros2, 
```bash
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w" --symlink-install --packages-select orbslam3
```

## How to use
1. Source the workspace  
```
$ source /home/ros2_ws/install/setup.bash
```

2. Run orbslam mode, which you want.  
For now, we only care about stereo and stereo-inertial.

First, start realsense node. 
```bash
ros2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_infra2:=true enable_accel:=true enable_gyro:=true unite_imu_method:=2 infra_width:=640 infra_height:=480 align_depth:=true pointcloud.enable:=true
```

Stereo
```bash
ros2 launch orbslam3 stereo.launch.yaml
```

Stereo-Inertial
```bash
ros2 launch orbslam3 stereo_inertial.launch.yaml
```