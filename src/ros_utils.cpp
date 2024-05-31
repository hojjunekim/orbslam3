#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/se3.hpp"
#include "cv_bridge/cv_bridge.h"

using ImageMsg = sensor_msgs::msg::Image;
using PcdMsg = sensor_msgs::msg::PointCloud2;
using PoseMsg = geometry_msgs::msg::PoseStamped;
using OdomMsg = nav_msgs::msg::Odometry;

// WIP: report all the pose/odom in FLU coordinate (currently all OpenCV)
// Notation:
// Tcw: Rigidbody Transform from camera coordinate to world coordinate
// Tbw: Rigidbody Transform from body coordinate to world coordinate
// Vwb: Linear velocity of body in world coordinate
// Wwb: Angular velocity of body in world coordinate

void publish_camera_pose(
  const rclcpp::Publisher<PoseMsg>::SharedPtr& publisher, 
  const rclcpp::Time& stamp,
  Sophus::SE3f& Tcw,  
  std::string world_frame_id)
{
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = world_frame_id;
  pose_msg.header.stamp = stamp;

  pose_msg.pose.position.x = Tcw.translation().x();
  pose_msg.pose.position.y = Tcw.translation().y();
  pose_msg.pose.position.z = Tcw.translation().z();

  pose_msg.pose.orientation.w = Tcw.unit_quaternion().coeffs().w();
  pose_msg.pose.orientation.x = Tcw.unit_quaternion().coeffs().x();
  pose_msg.pose.orientation.y = Tcw.unit_quaternion().coeffs().y();
  pose_msg.pose.orientation.z = Tcw.unit_quaternion().coeffs().z();

  publisher->publish(pose_msg);
}

void publish_body_odometry(
  const rclcpp::Publisher<OdomMsg>::SharedPtr& publisher,
  const rclcpp::Time& stamp, 
  Sophus::SE3f Tbw, 
  Eigen::Vector3f Vwb, 
  Eigen::Vector3f Wwb,
  std::string world_frame_id,
  std::string odom_frame_id)
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = world_frame_id;
  odom_msg.child_frame_id = odom_frame_id;
  odom_msg.header.stamp = stamp;

  odom_msg.pose.pose.position.x = Tbw.translation().x();
  odom_msg.pose.pose.position.y = Tbw.translation().y();
  odom_msg.pose.pose.position.z = Tbw.translation().z();

  odom_msg.pose.pose.orientation.w = Tbw.unit_quaternion().coeffs().w();
  odom_msg.pose.pose.orientation.x = Tbw.unit_quaternion().coeffs().x();
  odom_msg.pose.pose.orientation.y = Tbw.unit_quaternion().coeffs().y();
  odom_msg.pose.pose.orientation.z = Tbw.unit_quaternion().coeffs().z();

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

  publisher->publish(image_msg);
}

void publish_tf(
  const std::shared_ptr<tf2_ros::TransformBroadcaster>& tf_broadcaster,
  const rclcpp::Time& stamp,
  Sophus::SE3f T,
  std::string parent_frame_id,
  std::string child_frame_id)
{
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