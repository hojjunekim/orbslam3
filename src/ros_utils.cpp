#include <ros_utils.hpp>

using ImageMsg = sensor_msgs::msg::Image;
using PcdMsg = sensor_msgs::msg::PointCloud2;
using PoseMsg = geometry_msgs::msg::PoseStamped;
using OdomMsg = nav_msgs::msg::Odometry;

void publish_camera_pose(
  const rclcpp::Publisher<PoseMsg>::SharedPtr& publisher, 
  const rclcpp::Time& stamp,
  Sophus::SE3f& Twc,  
  std::string world_frame_id)
{
  // Twc: pose matrix from world coordinate to camera coordinate
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = world_frame_id;
  pose_msg.header.stamp = stamp;

  pose_msg.pose.position.x = Twc.translation().x();
  pose_msg.pose.position.y = Twc.translation().y();
  pose_msg.pose.position.z = Twc.translation().z();

  pose_msg.pose.orientation.w = Twc.unit_quaternion().coeffs().w();
  pose_msg.pose.orientation.x = Twc.unit_quaternion().coeffs().x();
  pose_msg.pose.orientation.y = Twc.unit_quaternion().coeffs().y();
  pose_msg.pose.orientation.z = Twc.unit_quaternion().coeffs().z();

  publisher->publish(pose_msg);
}

void publish_body_odometry(
  const rclcpp::Publisher<OdomMsg>::SharedPtr& publisher,
  const rclcpp::Time& stamp, 
  Sophus::SE3f Twb, 
  Eigen::Vector3f Vwb, 
  Eigen::Vector3f Wwb,
  std::string world_frame_id,
  std::string odom_frame_id)
{
  // Twb: pose matrix from world coordinate to body coordinate
  // Vwb: body velocity in world coordinate
  // Wwb: body angular velocity in world coordinate
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = world_frame_id;
  odom_msg.child_frame_id = odom_frame_id;
  odom_msg.header.stamp = stamp;

  odom_msg.pose.pose.position.x = Twb.translation().x();
  odom_msg.pose.pose.position.y = Twb.translation().y();
  odom_msg.pose.pose.position.z = Twb.translation().z();

  odom_msg.pose.pose.orientation.w = Twb.unit_quaternion().coeffs().w();
  odom_msg.pose.pose.orientation.x = Twb.unit_quaternion().coeffs().x();
  odom_msg.pose.pose.orientation.y = Twb.unit_quaternion().coeffs().y();
  odom_msg.pose.pose.orientation.z = Twb.unit_quaternion().coeffs().z();

  odom_msg.twist.twist.linear.x = Vwb.x();
  odom_msg.twist.twist.linear.y = Vwb.y();
  odom_msg.twist.twist.linear.z = Vwb.z();

  odom_msg.twist.twist.angular.x = Wwb.x();
  odom_msg.twist.twist.angular.y = Wwb.y();
  odom_msg.twist.twist.angular.z = Wwb.z();

  publisher->publish(odom_msg);
}

void publish_tracking_img(
  const rclcpp::Publisher<ImageMsg>::SharedPtr& publisher,
  const rclcpp::Time& stamp,
  cv::Mat image, 
  std::string frame_id)
{
  std_msgs::msg::Header header;

  header.stamp = stamp;
  header.frame_id = frame_id;

  sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

  publisher->publish(*image_msg.get());
}

void publish_tf(
  const std::unique_ptr<tf2_ros::TransformBroadcaster>& tf_broadcaster,
  const rclcpp::Time& stamp,
  Sophus::SE3f T,
  std::string parent_frame_id,
  std::string child_frame_id)
{
  // T: pose matrix from parent frame to child frame
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = stamp;
  t.header.frame_id = parent_frame_id;
  t.child_frame_id = child_frame_id;

  t.transform.translation.x = T.translation().x();
  t.transform.translation.y = T.translation().y();
  t.transform.translation.z = T.translation().z();

  t.transform.rotation.w = T.unit_quaternion().coeffs().w();
  t.transform.rotation.x = T.unit_quaternion().coeffs().x();
  t.transform.rotation.y = T.unit_quaternion().coeffs().y();
  t.transform.rotation.z = T.unit_quaternion().coeffs().z();

  tf_broadcaster->sendTransform(t);
}