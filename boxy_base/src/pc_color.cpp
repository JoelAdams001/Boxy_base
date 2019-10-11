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
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PCLCloud;
tf2_ros::Buffer tfbuffer;

class Colorizer {
  public:
  Colorizer() :
    it(node) {
    pclcloud.width = 307200;
    pclcloud.height = 1;
    pclcloud.is_dense = false
        ;
    pclcloud.header.frame_id = "camera_odom_frame";
    pclcloud.points.resize(pclcloud.width * pclcloud.height);

    cloud_pub = node.advertise<PCLCloud> ("pclcloud", 100);
    scan_sub = node.subscribe<sensor_msgs::LaserScan>("/scan", 1, &Colorizer::scanCallback, this);
    cam_sub = it.subscribeCamera("/usb_cam/image_rect_color", 1, &Colorizer::imageCallback, this);

  }
  ~Colorizer(){
    pcl::io::savePCDFileASCII ("test_pcd.pcd", pclcloud);
  }
  PCLCloud pclcloud;
  ros::Publisher cloud_pub;
  ros::NodeHandle node;
  cv::Point3d point;
  cv::Point3d point2;
  sensor_msgs::Image image;
  image_transport::ImageTransport it;
  image_transport::CameraSubscriber cam_sub;
  geometry_msgs::TransformStamped transform;
  geometry_msgs::TransformStamped transform2;
  image_geometry::PinholeCameraModel cam_model;
  ros::Subscriber scan_sub;
  std::size_t n;
  int pc_point = 0;

  float a;
  float angle_increment;
  uchar b, g, r;
  unsigned long count = 0;
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& s);
  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                     const sensor_msgs::CameraInfoConstPtr& info_msg);
  float m_ranges[];
};


void Colorizer::cloudCallback(const sensor_msgs::LaserScan::ConstPtr& s) {
 // ROS_INFO("Scan Callback!");
  tf2_ros::TransformListener tf_listener(tfbuffer);
  a = s->angle_min;
  n = s->ranges.size();
  angle_increment = s->angle_increment;

  for (int i = 0; i < 2*n; ++i) {

    m_ranges[i] = s->ranges[i];

   }  
}

void Colorizer::imageCallback( const sensor_msgs::ImageConstPtr& image_msg,
                                  const sensor_msgs::CameraInfoConstPtr& info_msg) {
  //ROS_INFO("Camera Callback!");
  try {
    transform =  tfbuffer.lookupTransform("camera_odom_frame", "laser", ros::Time(0));
    transform2 =  tfbuffer.lookupTransform("usb_cam", "camera_odom_frame", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  while(pc_point < pclcloud.width){
  for (int i = 0; i < 2*n; ++i) {

    //Transform from laser frame to odom frame to get point
    tf2::Transform t, t2;
    tf2::Vector3 v1(m_ranges[i]*std::cos(a), m_ranges[i]*std::sin(a), 0.00f);
    tf2::fromMsg(transform.transform, t);
    v1 = t * v1;
    a += angle_increment;

    point.x = static_cast<float>(v1.getX());
    point.y = static_cast<float>(v1.getY());
    point.z = static_cast<float>(v1.getZ());

    //Transform point1 from the odom frame to the camera frame as point2
    tf2::fromMsg(transform2.transform, t2);
    tf2::Vector3 v2(point2.x, point2.y, point2.z);
    v2 = t2 * v1;
    point2.x = v2.getX();
    point2.y = v2.getY();
    point2.z = v2.getZ();

    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
/*
    //Convert to OpenCV format
    cam_model.fromCameraInfo(info_msg); //Put inside of pinhole object
    int width = cam_model.fullResolution().width;
    int height = cam_model.fullResolution().height;
    cv::Point2d px = cam_model.project3dToPixel(point2); //Get pixel coordinate (u,v)
    ROS_INFO("%10.5f\t%10.5f\t%10.5f", point2.x, point2.y, point2.z);

    int u = static_cast<int>(px.y);
    int v = static_cast<int>(px.x);
    //ROS_INFO("%10.5d\t%10.5d", width, height);
    //ROS_INFO("%10.5f\t%10.5f", px.y, px.x);

    // Bound pixel to image size
    cv::Vec3b intensity;
    if ( u < 0 || u > width ||
         v < 0 || v > height ) {
      //pclcloud[pc_point + i].x = 0;
      //pclcloud[pc_point + i].y = 0;
      //pclcloud[pc_point + i].z = 0;
      intensity = cv::Vec3b(255,0,255);
    }
    else {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      image = input_bridge->image;
//      ROS_INFO("%10.5f\t%10.5f", px.x, px.y);
      intensity = image.at<cv::Vec3b>(u,v);
      }
    */
    cv::Vec3b intensity = cv::Vec3b(255,0,255);

    uchar b = intensity.val[0];
    uchar g = intensity.val[1];
    uchar r = intensity.val[2];

    pclcloud[pc_point + i].x = static_cast<float>(point.x);
    pclcloud[pc_point + i].y = static_cast<float>(point.y);
    pclcloud[pc_point + i].z = static_cast<float>(point.z);
    pclcloud[pc_point + i].r = r;
    pclcloud[pc_point + i].g = g;
    pclcloud[pc_point + i].b = b;
    pc_point++;
    }
  }
   // ROS_INFO("%10.5f\t%10.5f\t%10.5d\t%10.5d\t%10.5d", px.y, px.x, b, g, r);
    cloud_pub.publish(pclcloud);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "pc_color");
  tf2_ros::TransformListener tf_listener(tfbuffer);
  PlyAssembler stf;
  ros::spin();
}
