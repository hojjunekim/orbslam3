FROM osrf/ros:foxy-desktop
ENV ROS_DISTRO foxy 

# install dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        software-properties-common \
        git \
        build-essential \
        cmake \
        byobu \
        vim \
        libeigen3-dev \
        python3-colcon-common-extensions \
        libopencv-dev \
        ros-$ROS_DISTRO-vision-opencv \
        ros-$ROS_DISTRO-message-filters && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

WORKDIR /root

# build pangolin
WORKDIR /home/thirdparty
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    mkdir build && cd build && \
    cmake .. && \
    make -j && \
    make install

# build ORB SLAM3
# RUN git clone https://github.com/jnskkmhr/ORB_SLAM3.git
RUN git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3
RUN cd ORB_SLAM3 && \
chmod +x build.sh && \
./build.sh
# Issue: otherwise, you will have Sophus include errors
RUN cd ORB_SLAM3/Thirdparty/Sophus/build && make install

# Install realsense SDK and ROS2 wrapper
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-realsense2-*

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
WORKDIR /home/ros2_ws

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]