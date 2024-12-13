#ifndef __RGBD_INERTIAL_NODE_HPP__
#define __RGBD_INERTIAL_NODE_HPP__


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

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;
using PcdMsg = sensor_msgs::msg::PointCloud2;
using PoseMsg = geometry_msgs::msg::PoseStamped;
using OdomMsg = nav_msgs::msg::Odometry;

class RgbdInertialNode : public rclcpp::Node
{
public:
    RgbdInertialNode(ORB_SLAM3::System* pSLAM);

    ~RgbdInertialNode();

private:
    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabImageColor(const ImageMsg::SharedPtr msgColor);
    void GrabImageDepth(const ImageMsg::SharedPtr msgDepth);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void SyncWithImu();

    ORB_SLAM3::System* m_SLAM;
    std::thread *syncThread_;

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;

    rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgColor_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgDepth_;

    rclcpp::Publisher<PoseMsg>::SharedPtr pubPose_;
    rclcpp::Publisher<OdomMsg>::SharedPtr pubOdom_;
    rclcpp::Publisher<PcdMsg>::SharedPtr pubPcd_;
    rclcpp::Publisher<ImageMsg>::SharedPtr pubTrackImage_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_; 
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; 

    // IMU
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // Image
    queue<ImageMsg::SharedPtr> imgColorBuf_, imgDepthBuf_;
    std::mutex bufMutexColor_, bufMutexDepth_;

};

#endif
