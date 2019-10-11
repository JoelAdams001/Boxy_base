#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
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
#include <pcl/filters/voxel_grid.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PCLCloud;
tf2_ros::Buffer tfbuffer;

class PlyAssembler {
  public:
  PlyAssembler(){
    pclcloud.width = 1900000;
    pclcloud.height = 1;
    pclcloud.is_dense = false;
    pclcloud.header.frame_id = "camera_odom_frame";
    pclcloud.points.resize(pclcloud.width * pclcloud.height);

    cloud_pub = node.advertise<PCLCloud> ("pclcloud", 5);
    scan_sub = node.subscribe<sensor_msgs::LaserScan>("/scan", 5, &PlyAssembler::scanCallback, this);

  }
  ~PlyAssembler(){
    //pcl::io::savePCDFileASCII ("test_pcd.pcd", pclcloud);
  }

  PCLCloud pclcloud;
  int cloudsize = 0;
  ros::Publisher cloud_pub;
  ros::NodeHandle node;
  geometry_msgs::TransformStamped transform;
  ros::Subscriber scan_sub;
  int pc_point = 0;
  tf2::Transform t, t2;

  float a;
  float angle_increment;
  uchar b, g, r;
  unsigned long count = 0;

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& s);
};



void PlyAssembler::scanCallback(const sensor_msgs::LaserScan::ConstPtr& s) {
 // ROS_INFO("Scan Callback!");
  tf2_ros::TransformListener tf_listener(tfbuffer);
  a = s->angle_min;
  angle_increment = s->angle_increment;

//  if(cloudsize >= pclcloud.size() - 100){
//    pclcloud.points.resize(cloudsize + 1000);
//  }

  try {
    transform =  tfbuffer.lookupTransform("camera_odom_frame", "laser", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  for(int i = 0; i <= 720; ++i) {
      //Transform from laser frame to odom frame to get point
      tf2::Vector3 v1(s->ranges[i]*std::cos(a), s->ranges[i]*std::sin(a), 0.00f);
      tf2::fromMsg(transform.transform, t);
      v1 = t * v1;
      a += angle_increment;
      if(v1.getX() < 9 & v1.getX() > -9 & v1.getY() < 9 & v1.getY() > -9){
         ROS_INFO("%10.5f\t%10.5f\t%10.5f\t" , v1.getX() , v1.getY(), v1.getZ());
         pclcloud[cloudsize + i].x = (static_cast<float>(v1.getX()));
         pclcloud[cloudsize + i].y = (static_cast<float>(v1.getY()));
         pclcloud[cloudsize + i].z = (static_cast<float>(v1.getZ()));
         cloudsize++;
      }
    }
  cloud_pub.publish(pclcloud);
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "proj_img_to_pc");
  tf2_ros::TransformListener tf_listener(tfbuffer);
  PlyAssembler stf;
  ros::spin();
}
