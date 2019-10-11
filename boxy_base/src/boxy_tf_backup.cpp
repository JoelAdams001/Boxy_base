#include <ros/ros.h>
#include "std_msgs/String.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sstream>

geometry_msgs::TransformStamped odom;
geometry_msgs::TransformStamped base_to_realsense;

void odomCallback(const nav_msgs::Odometry::ConstPtr& o){
  geometry_msgs::Transform odom_to_realsense;
  geometry_msgs::Transform odom_to_base;
  tf2::Transform tf2_odom_to_realsense;
  tf2::Transform tf2_base_to_realsense;
  tf2::Transform tf2_odom_to_base;
  geometry_msgs::PoseWithCovariance p = o->pose;

  odom_to_realsense.translation.x = p.pose.position.x;
  odom_to_realsense.translation.y = p.pose.position.y;
  odom_to_realsense.translation.z = p.pose.position.z;

  odom_to_realsense.rotation.x = p.pose.orientation.x;
  odom_to_realsense.rotation.y = p.pose.orientation.y;
  odom_to_realsense.rotation.z = p.pose.orientation.z;
  odom_to_realsense.rotation.w = p.pose.orientation.w;

  tf2::fromMsg(odom_to_realsense, tf2_odom_to_realsense);
  tf2::fromMsg(base_to_realsense.transform, tf2_base_to_realsense);
  tf2_odom_to_realsense = tf2_base_to_realsense.inverse();
  tf2_odom_to_base = tf2_odom_to_realsense * tf2_base_to_realsense;


  tf2::convert(tf2_odom_to_base,odom_to_base);
  odom.transform = odom_to_base;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
}

int main(int argc, char** argv){
  ros::init(argc, argv, "boxy_tf_publisher");

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped base_to_lidar;
  geometry_msgs::TransformStamped base_to_usb_cam;
  tf2::Quaternion q;

  //Transform between base to lidar
  base_to_lidar.header.frame_id = "base_link";
  base_to_lidar.child_frame_id = "laser";
  base_to_lidar.transform.translation.x = -0.0030;
  base_to_lidar.transform.translation.y = -0.0885;
  base_to_lidar.transform.translation.z = 0.0510;
  q.setRPY(0, 0, 0);
  base_to_lidar.transform.rotation.x = q.x();
  base_to_lidar.transform.rotation.y = q.y();
  base_to_lidar.transform.rotation.z = q.z();
  base_to_lidar.transform.rotation.w = q.w();

  //Transform between base to usb camera
  base_to_usb_cam.header.frame_id = "base_link";
  base_to_usb_cam.child_frame_id = "usb_cam";
  base_to_usb_cam.transform.translation.x = 0.0722;
  base_to_usb_cam.transform.translation.y = 0.02540;
  base_to_usb_cam.transform.translation.z = -0.0250;
  q.setRPY(0, 0, 0);
  base_to_usb_cam.transform.rotation.x = q.x();
  base_to_usb_cam.transform.rotation.y = q.y();
  base_to_usb_cam.transform.rotation.z = q.z();
  base_to_usb_cam.transform.rotation.w = q.w();

  //Transform between base to usb camera
  base_to_realsense.header.frame_id = "base_link";
  base_to_realsense.child_frame_id = "camera_pose_frame";
  base_to_realsense.transform.translation.x = 0.0722;
  base_to_realsense.transform.translation.y = -0.0722;
  base_to_realsense.transform.translation.z = -0.0110;
  q.setRPY(0, 0, 0);
  base_to_realsense.transform.rotation.x = q.x();
  base_to_realsense.transform.rotation.y = q.y();
  base_to_realsense.transform.rotation.z = q.z();
  base_to_realsense.transform.rotation.w = q.w();

  ros::NodeHandle n;
  ros::Rate rate(10.0);
  ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 10, odomCallback);

  while (n.ok()){
    odom.header.stamp = ros::Time::now();
    base_to_lidar.header.stamp = ros::Time::now();
    base_to_usb_cam.header.stamp = ros::Time::now();
    base_to_realsense.header.stamp = ros::Time::now();
    br.sendTransform(odom);
    br.sendTransform(base_to_lidar);
    br.sendTransform(base_to_usb_cam);
    br.sendTransform(base_to_realsense);
  }

  return 0;
};
