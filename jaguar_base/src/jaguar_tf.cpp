#include <ros/ros.h>
#include "std_msgs/String.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <sstream>

int main(int argc, char** argv){
  ros::init(argc, argv, "boxy_tf_publisher");

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped base_to_lidar;
  geometry_msgs::TransformStamped base_to_usb_cam;
  geometry_msgs::TransformStamped base_to_realsense;
  tf2::Quaternion q;

  //Transform between base to lidar
  base_to_lidar.header.frame_id = "base_link";
  base_to_lidar.child_frame_id = "velodyne";
  base_to_lidar.transform.translation.x = -0.0800;
  base_to_lidar.transform.translation.y = 0.0350;
  base_to_lidar.transform.translation.z = -0.0850;
  q.setRPY(3.1415, 0, 0);
  base_to_lidar.transform.rotation.x = q.x();
  base_to_lidar.transform.rotation.y = q.y();
  base_to_lidar.transform.rotation.z = q.z();
  base_to_lidar.transform.rotation.w = q.w();

  //Transform between base to usb camera
  base_to_usb_cam.header.frame_id = "base_link";
  base_to_usb_cam.child_frame_id = "usb_cam";
  base_to_usb_cam.transform.translation.x = 0.000;
  base_to_usb_cam.transform.translation.y = 0.0900;
  base_to_usb_cam.transform.translation.z = -0.0130;
  //q.setRPY(-3.1415, 1.5708, 0);
  base_to_usb_cam.transform.rotation.x = 0.5;
  base_to_usb_cam.transform.rotation.y = -0.5;
  base_to_usb_cam.transform.rotation.z = 0.5;
  base_to_usb_cam.transform.rotation.w = -0.5;

  //Transform between base to usb camera
  base_to_realsense.header.frame_id = "camera_pose_frame";
  base_to_realsense.child_frame_id = "base_link";
  base_to_realsense.transform.translation.x = 0.0;
  base_to_realsense.transform.translation.y = 0.0;
  base_to_realsense.transform.translation.z = 0.0;
  q.setRPY(0, 0, 0);
  base_to_realsense.transform.rotation.x = q.x();
  base_to_realsense.transform.rotation.y = q.y();
  base_to_realsense.transform.rotation.z = q.z();
  base_to_realsense.transform.rotation.w = q.w();

  ros::NodeHandle n;
  ros::Rate rate(10.0);

  while (n.ok()){
    base_to_lidar.header.stamp = ros::Time::now();
    base_to_usb_cam.header.stamp = ros::Time::now();
    base_to_realsense.header.stamp = ros::Time::now();
    br.sendTransform(base_to_lidar);
    br.sendTransform(base_to_usb_cam);
    br.sendTransform(base_to_realsense);
  }

  return 0;
};
