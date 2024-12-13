#include "rgbd-inertial-node.hpp"

#include <opencv2/core/core.hpp>
#include <ros_utils.hpp>

using std::placeholders::_1;

RgbdInertialNode::RgbdInertialNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    rclcpp::QoS imu_qos(10);
    imu_qos.best_effort();
    imu_qos.durability_volatile();
    ////////////////////////////////////////
    subImu_ = this->create_subscription<ImuMsg>("camera/gyro_accel/sample", imu_qos, std::bind(&RgbdInertialNode::GrabImu, this, std::placeholders::_1));
    subImgColor_ = this->create_subscription<ImageMsg>("camera/color/image_raw", 10, std::bind(&RgbdInertialNode::GrabImageColor, this, std::placeholders::_1));
    subImgDepth_ = this->create_subscription<ImageMsg>("camera/depth/image_raw", 10, std::bind(&RgbdInertialNode::GrabImageDepth, this, std::placeholders::_1));

    // rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/color/image_raw");
    // depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/depth/image_raw");
    // syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    // syncApproximate->registerCallback(&RgbdInertialNode::GrabRGBD, this);

    pubPose_ = this->create_publisher<PoseMsg>("body_pose", 1);
    pubOdom_ = this->create_publisher<OdomMsg>("imu_odometry", 1);
    pubTrackImage_ = this->create_publisher<ImageMsg>("tracking_image", 1);
    pubPcd_ = this->create_publisher<PcdMsg>("point_cloud", 1);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // declare rosparameters
    this->declare_parameter("world_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("body_frame", "body_link");
    this->declare_parameter("body_optical_frame", "body_optical_link");
    this->declare_parameter("camera_optical_frame", "camera_optical_link");
    
    syncThread_ = new std::thread(&RgbdInertialNode::SyncWithImu, this);
}

RgbdInertialNode::~RgbdInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void RgbdInertialNode::GrabImageColor(const ImageMsg::SharedPtr msgColor)
{
    bufMutexColor_.lock();

    if (!imgColorBuf_.empty())
        imgColorBuf_.pop();
    imgColorBuf_.push(msgColor);

    bufMutexColor_.unlock();
}

void RgbdInertialNode::GrabImageDepth(const ImageMsg::SharedPtr msgDepth)
{
    bufMutexDepth_.lock();

    if (!imgDepthBuf_.empty())
        imgDepthBuf_.pop();
    imgDepthBuf_.push(msgDepth);

    bufMutexDepth_.unlock();
}

cv::Mat RgbdInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg); //, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    return cv_ptr->image.clone();

    // if (cv_ptr->image.type() == 0)
    // {
    //     return cv_ptr->image.clone();
    // }
    // else
    // {
    //     std::cerr << "Error image type" << std::endl;
    //     return cv_ptr->image.clone();
    // }
}

// void RgbdInertialNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
// {
//     // Copy the ros rgb image message to cv::Mat.
//     try
//     {
//         cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//         return;
//     }

//     // Copy the ros depth image message to cv::Mat.
//     try
//     {
//         cv_ptrD = cv_bridge::toCvShare(msgD);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//         return;
//     }

//     m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));
// }

void RgbdInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while (1)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "SLAM running...");
        cv::Mat imColor, imDepth;
        double tImColor = 0, tImDepth = 0;
        if (!imgColorBuf_.empty() && !imgDepthBuf_.empty() && !imuBuf_.empty())
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Grab Image");
            tImColor = Utility::StampToSec(imgColorBuf_.front()->header.stamp);
            tImDepth = Utility::StampToSec(imgDepthBuf_.front()->header.stamp);

            bufMutexDepth_.lock();
            while ((tImColor - tImDepth) > maxTimeDiff && imgDepthBuf_.size() > 1)
            {
                imgDepthBuf_.pop();
                tImDepth = Utility::StampToSec(imgDepthBuf_.front()->header.stamp);
            }
            bufMutexDepth_.unlock();

            bufMutexColor_.lock();
            while ((tImDepth - tImColor) > maxTimeDiff && imgColorBuf_.size() > 1)
            {
                imgColorBuf_.pop();
                tImColor = Utility::StampToSec(imgColorBuf_.front()->header.stamp);
            }
            bufMutexColor_.unlock();

            if ((tImColor - tImDepth) > maxTimeDiff || (tImDepth - tImColor) > maxTimeDiff)
            {
                RCLCPP_INFO_ONCE(this->get_logger(), "big time difference");
                continue;
            }
            if (tImColor > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;

            bufMutexColor_.lock();
            imColor = GetImage(imgColorBuf_.front());
            imgColorBuf_.pop();
            bufMutexColor_.unlock();

            bufMutexDepth_.lock();
            imDepth = GetImage(imgDepthBuf_.front());
            imgDepthBuf_.pop();
            bufMutexDepth_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            // Eigen::Vector3f Wbb; // body angular velocity in body frame
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                RCLCPP_INFO_ONCE(this->get_logger(), "Grab Imu");
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImColor)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    // Wbb = Eigen::Vector3f(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            // if (bClahe_)
            // {
            //     clahe_->apply(imColor, imColor);
            //     clahe_->apply(imDepth, imDepth);
            // }

            // if (doRectify_)
            // {
            //     cv::remap(imColor, imColor, M1l_, M2l_, cv::INTER_LINEAR);
            //     cv::remap(imDepth, imDepth, M1r_, M2r_, cv::INTER_LINEAR);
            // }
            
            // Transform of camera in  world frame
            Sophus::SE3f Tcw = m_SLAM->TrackRGBD(imColor, imDepth, tImColor, vImuMeas);
            Sophus::SE3f Twc = Tcw.inverse(); // Twc is imu optical frame pose in ROS FLU map coordinate

            // publish topics
            std::string world_frame = this->get_parameter("world_frame").as_string();
            std::string odom_frame = this->get_parameter("odom_frame").as_string();
            std::string body_frame = this->get_parameter("body_frame").as_string();
            std::string body_optical_frame = this->get_parameter("body_optical_frame").as_string();
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

            // coordiante transform
            Twc = Twc * cv_to_ros.inverse(); // imu frame pose in ROS FLU map coorinate

            // Option1: publish map to odom tf from SLAM and odom to camera from VIO 
            //// TF processing ////
            try {
                geometry_msgs::msg::TransformStamped camera_to_odom = tf_buffer_->lookupTransform(body_frame, odom_frame, tf2::TimePointZero);
                Sophus::SE3f Tco= transform_to_SE3(camera_to_odom);
                Sophus::SE3f Two = Twc * Tco.inverse();
                publish_world_to_odom_tf(tf_broadcaster_, this->get_clock()->now(), Two, world_frame, odom_frame);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_INFO(
                this->get_logger(), "Could not get transform %s to %s: %s",
                body_frame.c_str(), odom_frame.c_str(), ex.what());
                return;
            }

            // Option2: publish map to camera tf from SLAM
            // publish_camera_tf(tf_broadcaster_, this->get_clock()->now(), Twc, world_frame, body_frame);
            publish_camera_pose(pubPose_, this->get_clock()->now(), Twc, world_frame);
            publish_tracking_img(pubTrackImage_, this->get_clock()->now(), m_SLAM->GetCurrentFrame(), world_frame);

            // std::chrono::milliseconds tSleep(1);
            // std::this_thread::sleep_for(tSleep);
        }
    }
}
