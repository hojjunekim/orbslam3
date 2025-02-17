#include "rgbd-rel-node.hpp"

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
    pubOdom_ = this->create_publisher<OdomMsg>("visual_odom", 1);
    pubOdomRel_ = this->create_publisher<OdomMsg>("visual_odom_rel", 1);
    
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    // tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(m_queue_size), *rgb_sub, *depth_sub);
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
    this->declare_parameter("queue_size", 5);
    this->declare_parameter("debug", false);
    this->declare_parameter("pose_use", "Kinematics");

    m_world_frame = this->get_parameter("world_frame").as_string();
    m_odom_frame = this->get_parameter("odom_frame").as_string();
    m_camera_frame = this->get_parameter("camera_frame").as_string();
    m_local_mapping = this->get_parameter("local_mapping").as_bool();
    m_color_topic = this->get_parameter("color_topic").as_string();
    m_depth_topic = this->get_parameter("depth_topic").as_string();
    m_queue_size = this->get_parameter("queue_size").as_int();
    m_debug = this->get_parameter("debug").as_bool();
    m_pose_use = this->get_parameter("pose_use").as_string();

    RCLCPP_INFO(this->get_logger(), "world_frame: %s", m_world_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "odom_frame: %s", m_odom_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "camera_frame: %s", m_camera_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "local_mapping: %d", m_local_mapping);
    RCLCPP_INFO(this->get_logger(), "color_topic: %s", m_color_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "depth_topic: %s", m_depth_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "queue_size: %d", m_queue_size);
    RCLCPP_INFO(this->get_logger(), "debug: %d", m_debug);
    RCLCPP_INFO(this->get_logger(), "pose_use: %s", m_pose_use.c_str());
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    rclcpp::Time stamp = msgD->header.stamp;
    rclcpp::Time color_stamp = msgRGB->header.stamp;

    if (m_debug) RCLCPP_INFO(this->get_logger(), "c/d stamp : %f %f", color_stamp.seconds(), stamp.seconds());

    auto start_time = this->now();

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

    // int isLost = m_SLAM->isLost();
    int numBA = m_SLAM->LocalMappingNumBA();
    int numMerge = m_SLAM->LoopClosingNumMergeLocal();
    int numLoop = m_SLAM->LoopClosingNumLoop();
    int numReset = m_SLAM->TrackingNumReset();
    int tracking_state = m_SLAM->GetTrackingState();
    bool mergeDetected = m_SLAM->MergeDetected();

    // if(numBA > numBA_prev && m_local_mapping)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Local BA Detected: %d", numBA);
    //     Sophus::SE3f TKFdeltaBA = m_SLAM->LocalMappingDeltaTKFBA();
    //     Two = Two * TKFdeltaBA;
    //     numBA_prev = numBA;
    // }
    if(mergeDetected) 
    {
        m_relative = false;
        RCLCPP_INFO(this->get_logger(), "Merge Detected #: %d", numMerge);
    }
    if(numMerge > numMerge_prev)
    {
        m_relative = false;
        RCLCPP_INFO(this->get_logger(), "Merge Finished #: %d", numMerge);
        if(m_local_mapping)
        {
            Sophus::SE3f TKFdeltaMerge = m_SLAM->LoopClosingDeltaTKFMerge();
            Two = Two * TKFdeltaMerge;
        }
        numMerge_prev = numMerge;
    }
    if(numLoop > numLoop_prev)
    {
        RCLCPP_INFO(this->get_logger(), "Loop Detected: %d", numLoop);
        if(m_local_mapping)
        {
            Sophus::SE3f TKFdeltaLoop = m_SLAM->LoopClosingDeltaTKFLoop();
            Two = Two * TKFdeltaLoop;
            numLoop_prev = numLoop;
        }
    }
    if(tracking_state == 0)
    {
        m_relative = true;
        RCLCPP_INFO(this->get_logger(), "Rel ON. Track Lost #: %d", numReset);
        if(m_local_mapping)
        {
            Sophus::SE3f TKFdeltaReset = m_SLAM->TrackingDeltaTKFReset();
            Eigen::Vector3f tKFw = TKFdeltaReset.translation();
            Eigen::Quaternionf qKFw = TKFdeltaReset.unit_quaternion();
            Two = Two * TKFdeltaReset;
            numReset_prev = numReset;
        }
    }
    if(numReset == -1 && m_local_mapping)
    {
        m_relative = false;
        RCLCPP_INFO(this->get_logger(), "Reset Detected: %d", numReset);
        Two = Sophus::SE3f();
        numReset_prev = 0;
    }

    RCLCPP_INFO(this->get_logger(), "Tracking state: %d", tracking_state); // 0: start new map(after 3), 1: not initialized 2: good, 3: recently lost, 4: lost without initial map

    Sophus::SE3f Two_refined = cv2ros * Two * cv2ros.inverse();
    publish_world_to_odom_tf(tf_broadcaster_, stamp, Two_refined, m_world_frame, m_odom_frame);
    publish_camera_tf(tf_broadcaster_, stamp, Twc, m_world_frame, "Visual_Odom");
    publish_tracking_img(pubTrackImage_, stamp, m_SLAM->GetCurrentFrame(), m_world_frame);

    if(m_pose_use == "VO") publish_camera_tf(tf_broadcaster_, stamp, Twc, m_world_frame, m_camera_frame);

    if (tracking_state != 2 || mergeDetected) 
    {
        return;
    }
    else if (m_relative)
    {
        publish_camera_odometry(pubOdomRel_, stamp, Twc, m_world_frame, m_camera_frame);
    }
    else
    {
        publish_camera_odometry(pubOdom_, stamp, Twc, m_world_frame, m_camera_frame);
    }
    
   

    // bool map_changed = m_SLAM->MapChanged();
    // int tracking_state = m_SLAM->GetTrackingState();
    // std::vector<cv::KeyPoint> keypoints = m_SLAM->GetTrackedKeyPointsUn();
    // int isLost = m_SLAM->isLost();

    if (m_debug)
    {
        auto end_time = this->now();
        auto duration = end_time - start_time;
        RCLCPP_INFO(this->get_logger(), "Duration: %f", duration.seconds());
    }

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

    // Th1::Track() ->
    //   TrackWithMotionModel()
    //     UpdateLastFrame()
    //       Tl = Tlr * TlKFr
    //     Tc = Tvel * Tl
    //  TrackLocalMap()
    //    Optimizer::PoseOptimization(&mCurrentFrame) from single frame info
    //      Tc.optimize()
    //   Tvel = Tc * Tl.inverse
    //   Tl = Tc
    // if mState==OK Tcr = Tc * TcKFr.inverse
    // Th2::LocalMapping::run()
    //   if new frame -> Optimizer::LocalBundleAdjustment
    //     for all TKFs.optimize()
    // Th3::LoopClosing::run()
    //   if loopDetected -> CorrectLoop() -> update all TKF Tc
    //   if mergeDetected -> MergeLocal() -> update all TKF Tc
    // Th4::System::
    //   get Tc after poseopt and localBA, spontaneous loopclose
}
