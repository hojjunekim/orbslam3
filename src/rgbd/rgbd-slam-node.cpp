#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <ros_utils.hpp>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/color/image_raw");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/depth/image_raw");

    pubPose_ = this->create_publisher<PoseMsg>("camera_pose", 1);
    pubTrackImage_ = this->create_publisher<ImageMsg>("tracking_image", 1);
    pubPcd_ = this->create_publisher<PcdMsg>("point_cloud", 1);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

    // declare rosparameters
    this->declare_parameter("world_frame", "map");
    this->declare_parameter("odom_ref_frame", "camera_color_frame");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("camera_frame", "camera_color_frame");

    world_frame = this->get_parameter("world_frame").as_string();
    odom_ref_frame = this->get_parameter("odom_ref_frame").as_string();
    odom_frame = this->get_parameter("odom_frame").as_string();
    camera_frame = this->get_parameter("camera_frame").as_string();
}

RgbdSlamNode::~RgbdSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    if (!init)
    {
        try
        {
            geometry_msgs::msg::TransformStamped world_to_odom = tf_buffer_->lookupTransform(world_frame, odom_ref_frame, tf2::TimePointZero);
            Two = transform_to_SE3(world_to_odom);
            init = true;
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_INFO(
            this->get_logger(), "Could not get transform %s to %s: %s",
            world_frame.c_str(), camera_frame.c_str(), ex.what());
            return;
        }

    }

    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f Tco;
    Tco = m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));
    Sophus::SE3f Toc = Tco.inverse(); // camera optical frame pose in opencv coordinate

        // define coordinate transforms ///
    // OpenCV to ROS FLU coordinate transforms
    Eigen::Matrix<float, 3, 3> cv_to_ros_rot; 
    Eigen::Matrix<float, 3, 1> cv_to_ros_trans; 
    cv_to_ros_rot << 0, 0, 1,
                    -1, 0, 0,
                    0, -1, 0;
    cv_to_ros_trans << 0, 0, 0;
    Sophus::SE3f cv_to_ros(cv_to_ros_rot, cv_to_ros_trans);
    std::cout << cv_to_ros.matrix() << std::endl; 

    // Coordinate Transform: OpenCV coordinate to ROS FLU coordinate
    Toc = cv_to_ros * Toc; // camera optical frame pose in ROS FLU map coorinate
    Toc = Toc * cv_to_ros.inverse(); // camera frame pose in ROS FLU map coorinate

    // Option2: publish map to camera tf from SLAM
    publish_world_to_odom_tf(tf_broadcaster_, msgD->header.stamp, Two, world_frame, odom_frame);
    publish_camera_tf(tf_broadcaster_, msgD->header.stamp, Toc, odom_frame, camera_frame);
    publish_camera_pose(pubPose_, msgD->header.stamp, Toc, odom_frame);
    publish_tracking_img(pubTrackImage_, msgD->header.stamp, m_SLAM->GetCurrentFrame(), world_frame);
}
