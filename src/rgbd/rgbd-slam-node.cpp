#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <ros_utils.hpp>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    ReadParam();

    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), m_color_topic);
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), m_depth_topic);

    pubTrackImage_ = this->create_publisher<ImageMsg>("tracking_image", 1);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    // tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

    // declare rosparameters

}

RgbdSlamNode::~RgbdSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::ReadParam()
{
    this->declare_parameter("world_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("camera_frame", "camera_color_frame");
    this->declare_parameter("local_mapping", true);
    this->declare_parameter("color_topic", "camera/color/image_raw");
    this->declare_parameter("depth_topic", "camera/depth/image_raw");

    m_world_frame = this->get_parameter("world_frame").as_string();
    m_odom_frame = this->get_parameter("odom_frame").as_string();
    m_camera_frame = this->get_parameter("camera_frame").as_string();
    m_local_mapping = this->get_parameter("local_mapping").as_bool();
    m_color_topic = this->get_parameter("color_topic").as_string();
    m_depth_topic = this->get_parameter("depth_topic").as_string();
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
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

    rclcpp::Time stamp = msgD->header.stamp;



    Sophus::SE3f Tcw;
    Tcw = m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(stamp));

    Eigen::Matrix<float, 3, 3> cv2ros_r; 
    Eigen::Matrix<float, 3, 1> cv2_ros_t; 
    cv2ros_r << 0, 0, 1,
                -1, 0, 0,
                0, -1, 0;
    cv2_ros_t << 0, 0, 0;
    Sophus::SE3f cv2ros(cv2ros_r, cv2_ros_t);

    // OpenCV to ROS FLU coordinate transforms
    Sophus::SE3f Twc = cv2ros * Tcw.inverse() * cv2ros.inverse(); // camera frame pose in ROS FLU map coorinate

    if(m_local_mapping)
    {
        int numBA = m_SLAM->LocalMappingNumBA();
        int numMerge = m_SLAM->LoopClosingNumMergeLocal();
        int numLoop = m_SLAM->LoopClosingNumLoop();
        int numReset = m_SLAM->TrackingNumReset();

        // if(numBA > numBA_prev)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Local BA Detected: %d", numBA);
        //     Sophus::SE3f TKFdeltaBA = m_SLAM->LocalMappingDeltaTKFBA();
        //     Two = Two * TKFdeltaBA;
        //     numBA_prev = numBA;
        // }
        if(numMerge > numMerge_prev)
        {
            RCLCPP_INFO(this->get_logger(), "Merge Detected: %d", numMerge);
            Sophus::SE3f TKFdeltaMerge = m_SLAM->LoopClosingDeltaTKFMerge();
            Two = Two * TKFdeltaMerge;
            numMerge_prev = numMerge;
        }
        // if(numLoop > numLoop_prev)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Loop Detected: %d", numLoop);
        //     Sophus::SE3f TKFdeltaLoop = m_SLAM->LoopClosingDeltaTKFLoop();
        //     Two = Two * TKFdeltaLoop;
        //     numLoop_prev = numLoop;
        // }
        if(numReset > numReset_prev)
        {
            RCLCPP_INFO(this->get_logger(), "Lost Detected: %d", numReset);
            Sophus::SE3f TKFdeltaReset = m_SLAM->TrackingDeltaTKFReset();
            Eigen::Vector3f tKFw = TKFdeltaReset.translation();
            Eigen::Quaternionf qKFw = TKFdeltaReset.unit_quaternion();
            std::cout << "tKFw: " << tKFw.transpose() << " qKFw: " << qKFw.coeffs().transpose() << std::endl;
            Two = Two * TKFdeltaReset;
            numReset_prev = numReset;
        }
        else if(numReset == -1)
        {
            RCLCPP_INFO(this->get_logger(), "Reset Detected: %d", numReset);
            Two = Sophus::SE3f();
            numReset_prev = 0;
        }
    }

    Sophus::SE3f Two_refined = cv2ros * Two * cv2ros.inverse();
    publish_world_to_odom_tf(tf_broadcaster_, stamp, Two_refined, m_world_frame, m_odom_frame);
    publish_camera_tf(tf_broadcaster_, stamp, Twc, m_world_frame, m_camera_frame);
    publish_tracking_img(pubTrackImage_, stamp, m_SLAM->GetCurrentFrame(), m_world_frame);

    bool map_changed = m_SLAM->MapChanged();
    int tracking_state = m_SLAM->GetTrackingState();
    std::vector<cv::KeyPoint> keypoints = m_SLAM->GetTrackedKeyPointsUn();
    bool isLost = m_SLAM->isLost();

    // if(tracking_state != 2)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Tracking state: %d", tracking_state); // 0: start new map(after 3), 2: good, 3: lost, 4: ?
    // }
    // if(map_changed)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Map changed!");
    // }
    // if(isLost)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Tracking lost!");
    // }
}
