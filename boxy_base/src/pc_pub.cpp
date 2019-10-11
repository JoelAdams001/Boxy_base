#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <cmath>
#include <tf2/convert.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PCLCloud;
tf2_ros::Buffer tfbuffer;

class Assembler {
    public:
    Assembler(){
	 //Setup the pcl object
	 pclcloud.width = 307200;
	 pclcloud.height = 1;
	 pclcloud.is_dense = false;
	 pclcloud.header.frame_id = "camera_odom_frame";
	 pclcloud.points.resize(pclcloud.width * pclcloud.height);

   cloud_pub = node.advertise<PCLCloud> ("pclcloud", 1000);
 	 scan_sub = node.subscribe<sensor_msgs::LaserScan>("/scan", 1, &Assembler::scanCallback, this);
    }
    ~Assembler(){
    pcl::io::savePCDFileASCII ("uncolored.pcd", pclcloud);  
    }

 ros::Subscriber scan_sub;
 ros::Publisher cloud_pub;
 ros::NodeHandle node;
 int pc_point = 0;
 PCLCloud pclcloud;

 void scanCallback(const sensor_msgs::LaserScan::ConstPtr& s);
};

void Assembler::scanCallback(const sensor_msgs::LaserScan::ConstPtr& s) {
  float a = s->angle_min;
  std::size_t n = s->ranges.size();
  float angle_increment = s->angle_increment;
  geometry_msgs::TransformStamped transform;
  tf2::Transform t;

  try {
    transform =  tfbuffer.lookupTransform("camera_odom_frame", "laser", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  for (int i = 0; i < 2*n; ++i) {
    //Transform from laser frame to odom frame to get point
    tf2::Vector3 v1(s->ranges[i]*std::cos(a), s->ranges[i]*std::sin(a), 0.00f);
    tf2::fromMsg(transform.transform, t);
    v1 = t * v1;
    a += angle_increment;

 //   ROS_INFO("%10.5f\t%10.5f", s->ranges[i]*std::cos(a), s->ranges[i]*std::sin(a));

    pclcloud[pc_point + i].x = static_cast<float>(v1.getX());
    pclcloud[pc_point + i].y = static_cast<float>(v1.getY());
    pclcloud[pc_point + i].z = static_cast<float>(v1.getZ());
   // pc_point++;
 //   ROS_INFO("%10.5f\t%10.5f\t%10.5f", v1.getX(), v1.getY(), v1.getZ());
    }
  cloud_pub.publish(pclcloud);
  }



int main(int argc, char **argv) {
  ros::init(argc, argv, "pc_pub");
  tf2_ros::TransformListener tf_listener(tfbuffer);
  Assembler object;
  ros::spin();
}
