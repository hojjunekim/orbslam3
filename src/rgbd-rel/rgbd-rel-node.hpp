#ifndef __RGBD_SLAM_NODE_HPP__
#define __RGBD_SLAM_NODE_HPP__

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImageMsg = sensor_msgs::msg::Image;
using PcdMsg = sensor_msgs::msg::PointCloud2;
using PoseMsg = geometry_msgs::msg::PoseStamped;
using OdomMsg = nav_msgs::msg::Odometry;

class RgbdSlamNode : public rclcpp::Node
{
public:
    RgbdSlamNode(ORB_SLAM3::System* pSLAM);

    ~RgbdSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);
    void ReadParam();

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > rgb_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > depth_sub;

    rclcpp::Publisher<OdomMsg>::SharedPtr pubOdom_;
    rclcpp::Publisher<OdomMsg>::SharedPtr pubOdomRel_;
    rclcpp::Publisher<PcdMsg>::SharedPtr pubPcd_;
    rclcpp::Publisher<ImageMsg>::SharedPtr pubTrackImage_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_; 
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; 
    
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;

    bool init = false;
    Sophus::SE3f Two = Sophus::SE3f();

    int numBA_prev = 0;
    int numMerge_prev = 0;
    int numLoop_prev = 0;
    int numReset_prev = 0;

    std::string m_world_frame; 
    std::string m_odom_frame;
    std::string m_odom_ref_frame;
    std::string m_camera_frame;
    std::string m_color_topic;
    std::string m_depth_topic;
    std::string m_pose_use;
    bool m_local_mapping;
    int m_queue_size;
    bool m_debug;
    int m_relative = false;
};

#endif
