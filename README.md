# ORB_SLAM3_ROS2
This repository is ROS2 wrapping to use ORB_SLAM3

---

## Prerequisites
- I have tested on below version.
  - Ubuntu 20.04
  - ROS2 foxy
  - OpenCV 4.2.0

- Build ORB_SLAM3
  - Go to this [repo](https://github.com/zang09/ORB-SLAM3-STEREO-FIXED) and follow build instruction.

- Install related ROS2 package
```
$ sudo apt install ros-$ROS_DISTRO-vision-opencv && sudo apt install ros-$ROS_DISTRO-message-filters
```

## How to build
We use docker for the simplicity. \
You can build docker image via
```bash
# option1
./docker/build_image.sh
# option2
docker pull kmhr201218/orbslam3-ros2:latest
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


Stereo
```bash
ros2 launch orbslam3 stereo.launch.yaml
```

Stereo-Inertial
```bash
ros2 launch orbslam3 stereo_inertial.launch.yaml
```