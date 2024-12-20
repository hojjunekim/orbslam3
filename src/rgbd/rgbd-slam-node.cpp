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
    this->declare_parameter("use_toro", false);

    world_frame = this->get_parameter("world_frame").as_string();
    odom_ref_frame = this->get_parameter("odom_ref_frame").as_string();
    odom_frame = this->get_parameter("odom_frame").as_string();
    camera_frame = this->get_parameter("camera_frame").as_string();
    use_toro = this->get_parameter("use_toro").as_bool();
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
    if(!use_toro && !init)
    {
        RCLCPP_INFO(this->get_logger(), "No TORO optimization, initialize with identity transform");
        Two = Sophus::SE3f();
        init = true;
    }
    else if (!init)
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

    Sophus::SE3f Tco = m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgD->header.stamp));
    Sophus::SE3f Toc = Tco.inverse(); // camera optical frame pose in opencv coordinate
    
    int numBA = m_SLAM->LocalMappingNumBA();
    int numMerge = m_SLAM->LoopClosingNumMergeLocal();

    if(numBA > numBA_prev)
    {
        RCLCPP_INFO(this->get_logger(), "Local BA Detected: %d", numBA);
        Sophus::SE3f TKFdeltaBA = m_SLAM->LocalMappingDeltaTKFBA();
        Two = Two * TKFdeltaBA;
        numBA_prev = numBA;
    }
    if(numMerge > numMerge_prev)
    {
        RCLCPP_INFO(this->get_logger(), "Loop Merge Detected: %d", numMerge);
        Sophus::SE3f TKFdeltaMerge = m_SLAM->LoopClosingDeltaTKFMerge();
        Two = Two * TKFdeltaMerge;
        numMerge_prev = numMerge;
    }

    bool map_changed = m_SLAM->MapChanged();
    int tracking_state = m_SLAM->GetTrackingState();
    std::vector<cv::KeyPoint> keypoints = m_SLAM->GetTrackedKeyPointsUn();
    bool isLost = m_SLAM->isLost();
    bool isFinished = m_SLAM->isFinished();

    if(tracking_state != 2)
    {
        RCLCPP_INFO(this->get_logger(), "Tracking state: %d", tracking_state); // 0: start new map(after 3), 2: good, 3: lost, 4: ?
    }
    if(map_changed)
    {
        RCLCPP_INFO(this->get_logger(), "Map changed!");
    }
    if(isLost)
    {
        RCLCPP_INFO(this->get_logger(), "Tracking lost!");
    }
    if(isFinished)
    {
        RCLCPP_INFO(this->get_logger(), "Tracking finished!");
    }

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
    // [rgbd-1] [INFO] [1734546731.431254794] [orbslam3]: Tracking state: 3
    // [rgbd-1] Fail to track local map!
    // [rgbd-1] Creation of new map with id: 1
    // [rgbd-1] Stored map with ID: 0
    // [rgbd-1] Creation of new map with last KF id: 59
    // [rgbd-1] [INFO] [1734546731.553931130] [orbslam3]: Tracking state: 0
    // [rgbd-1] First KF:59; Map init KF:59
    // [rgbd-1] New Map created with 1027 points
    // [rgbd-1] *Merge detected
    // [rgbd-1] Local Mapping STOP
    // [rgbd-1] Change to map with id: 0
    // [rgbd-1] Local Mapping RELEASE
    // [rgbd-1] Local Mapping RELEASE
    // [rgbd-1] Merge finished!

    // OpenCV to ROS FLU coordinate transforms
    Eigen::Matrix<float, 3, 3> cv_to_ros_rot; 
    Eigen::Matrix<float, 3, 1> cv_to_ros_trans; 
    cv_to_ros_rot << 0, 0, 1,
                    -1, 0, 0,
                    0, -1, 0;
    cv_to_ros_trans << 0, 0, 0;
    Sophus::SE3f cv_to_ros(cv_to_ros_rot, cv_to_ros_trans);

    // Coordinate Transform: OpenCV coordinate to ROS FLU coordinate
    Toc = cv_to_ros * Toc; // camera optical frame pose in ROS FLU map coorinate
    Toc = Toc * cv_to_ros.inverse(); // camera frame pose in ROS FLU map coorinate

    // Option2: publish map to camera tf from SLAM
    publish_world_to_odom_tf(tf_broadcaster_, msgD->header.stamp, Two, world_frame, odom_frame);
    publish_camera_tf(tf_broadcaster_, msgD->header.stamp, Toc, odom_frame, camera_frame);
    publish_camera_pose(pubPose_, msgD->header.stamp, Toc, odom_frame);
    publish_tracking_img(pubTrackImage_, msgD->header.stamp, m_SLAM->GetCurrentFrame(), world_frame);
}
