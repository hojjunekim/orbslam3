#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>
#include <ros_utils.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    img_sub = this->create_subscription<ImageMsg>("camera/image_raw", 10, std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));

    pubPose_ = this->create_publisher<PoseMsg>("camera_pose", 1);
    pubTrackImage_ = this->create_publisher<ImageMsg>("tracking_image", 1);
    pubPcd_ = this->create_publisher<PcdMsg>("point_cloud", 1);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // declare rosparameters
    this->declare_parameter("world_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("camera_frame", "camera_link");
    this->declare_parameter("camera_optical_frame", "camera_optical_link");
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f Tcw;
    Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
    Sophus::SE3f Twc = Tcw.inverse(); // camera optical frame pose in opencv coordinate
    
    // publish topics
    std::string world_frame = this->get_parameter("world_frame").as_string();
    std::string odom_frame = this->get_parameter("odom_frame").as_string();
    std::string camera_frame = this->get_parameter("camera_frame").as_string();
    std::string camera_optical_frame = this->get_parameter("camera_optical_frame").as_string();

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
    Twc = cv_to_ros * Twc; // camera optical frame pose in ROS FLU map coorinate
    Twc = Twc * cv_to_ros.inverse(); // camera frame pose in ROS FLU map coorinate

    // Option1: publish map to odom tf from SLAM and odom to camera from VIO 
    // TF processing ////
    try {
        geometry_msgs::msg::TransformStamped camera_to_odom = tf_buffer_->lookupTransform(camera_frame, odom_frame, tf2::TimePointZero);
        Sophus::SE3f Tco= transform_to_SE3(camera_to_odom);
        Sophus::SE3f Two = Twc * Tco.inverse();
        publish_world_to_odom_tf(tf_broadcaster_, this->get_clock()->now(), Two, world_frame, odom_frame);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
        this->get_logger(), "Could not get transform %s to %s: %s",
        camera_frame.c_str(), odom_frame.c_str(), ex.what());
        return;
    }

    // Option2: publish map to camera tf from SLAM
    // publish_camera_tf(tf_broadcaster_, this->get_clock()->now(), Twc, world_frame, camera_frame);
    publish_camera_pose(pubPose_, this->get_clock()->now(), Twc, world_frame);
    publish_tracking_img(pubTrackImage_, this->get_clock()->now(), m_SLAM->GetCurrentFrame(), world_frame);
}
