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

class PlyAssembler {
  public:
  PlyAssembler() :
    it(node) {
    pclcloud.width = 307200;
    pclcloud.height = 1;
    pclcloud.is_dense = false;
    pclcloud.header.frame_id = "camera_odom_frame";
    pclcloud.points.resize(pclcloud.width * pclcloud.height);

    cloud_pub = node.advertise<PCLCloud> ("pclcloud", 10);
    scan_sub = node.subscribe<sensor_msgs::LaserScan>("/scan", 1, &PlyAssembler::scanCallback, this);
    cam_sub = it.subscribeCamera("/usb_cam/image_raw", 1, &PlyAssembler::imageCallback, this);

  }
  ~PlyAssembler(){
    //pcl::io::savePCDFileASCII ("test_pcd.pcd", pclcloud);
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
  int pc_point = 0;
  tf2::Transform t, t2;
  std::vector<cv::Point3f> points_odom;
  std::vector<cv::Point3f> points_cam;

  float a;
  float angle_increment;
  uchar b, g, r;
  unsigned long count = 0;

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& s);
  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                     const sensor_msgs::CameraInfoConstPtr& info_msg);
};



void PlyAssembler::scanCallback(const sensor_msgs::LaserScan::ConstPtr& s) {
 // ROS_INFO("Scan Callback!");
  tf2_ros::TransformListener tf_listener(tfbuffer);
  a = s->angle_min;
  angle_increment = s->angle_increment;

  try {
    transform =  tfbuffer.lookupTransform("camera_odom_frame", "laser", ros::Time(0));
    transform2 =  tfbuffer.lookupTransform("usb_cam", "camera_odom_frame", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  for (int i = 0; i <= 720; ++i) {
    //Transform from laser frame to odom frame to get point
    tf2::Vector3 v1(s->ranges[i]*std::cos(a), s->ranges[i]*std::sin(a), 0.00f);
    tf2::fromMsg(transform.transform, t);
    v1 = t * v1;
    a += angle_increment;

    cv::Point3f P(static_cast<float>(v1.getX()),
                  static_cast<float>(v1.getY()),
                  static_cast<float>(v1.getZ()));
    points_odom.push_back(P);

    //Transform point1 from the odom frame to the camera frame as point2
    tf2::fromMsg(transform2.transform, t2);
    tf2::Vector3 v2(point2.x, point2.y, point2.z);
    v2 = t2 * v1;

    cv::Point3f P2(static_cast<float>(v2.getX()),
                  static_cast<float>(v2.getY()),
                  static_cast<float>(v2.getZ()));
    points_cam.push_back(P2);
    }
}

void print(cv::Mat &m,const std::string &label) {
  std::cout << label.c_str() << std:: endl;
  for(int i = 0; i < m.rows; ++i) {
    for(int j = 0; j < m.cols; ++j)
      std::cout << m.at<double>(i,j) << '\t';
    std::cout << std::endl;
  }
}


void PlyAssembler::imageCallback( const sensor_msgs::ImageConstPtr& image_msg,
                                  const sensor_msgs::CameraInfoConstPtr& info_msg) {
//  cv::Mat image;
//  cv::Vec3f tvec(0.0,0.0,0.0);
//  cv::Vec3f R(0.0, 0.0, 0.0);
//  double im[] = { 0.0, 0.0, 0.0,
//                  0.0, 0.0, 0.0,
//                  0.0, 0.0, 0.0 };
//  cv::Mat I(3, 3, CV_64F, im);

//  double cm[] = { 479.418685,   0.000000, 323.650437,
//                    0.000000, 480.726384, 230.820885,
//                    0.000000,   0.000000,   1.000000};

//  cv::Mat C(3, 3, CV_64F, cm);
//  double dm[] =  {-0.363481, 0.124380, 0.002767, 0.001344, 0.000000};
//  cv::Mat D(1, 5, CV_64F, dm);


//  std::vector<cv::Point2f> imagePoints;
//  cv_bridge::CvImagePtr input_bridge;

//  cam_model.fromCameraInfo(info_msg); //Put inside of pinhole object
//  int width = cam_model.fullResolution().width;
//  int height = cam_model.fullResolution().height;
//  cv::Matx33d C = cam_model.intrinsicMatrix();
//  cv::Mat D = cam_model.distortionCoeffs();

//  cv::projectPoints(points_cam, R, tvec, C, D, imagePoints);

  for (int i = 0; i <= 720 ; ++i) {
//    cv::Point2d imagePoint = cam_model.project3dToPixel(points_cam[i]);
//    int u = static_cast<int>(imagePoint.x);
//    int v = static_cast<int>(imagePoint.y);
//    ROS_INFO("%10.5d\t%10.5d", u, v);
      // Bound pixel to image size
//    cv::Vec3b intensity;
//    intensity = cv::Vec3b(0,0,0);
//    if ( u < 0 || u > width ||
//         v < 0 || v > height ) {
//      intensity = cv::Vec3b(255,0,0);
//    }
//   else {
//      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
//      image = input_bridge->image;
//      intensity = image.at<cv::Vec3b>(u,v);

//      uchar b = intensity.val[0];
//      uchar g = intensity.val[1];
//      uchar r = intensity.val[2];
//      ROS_INFO("%10.5f\t%10.5f", imagePoint.x, imagePoint.y);

      pclcloud[i].x = (points_odom[i].x);
      pclcloud[i].y = (points_odom[i].y);
      pclcloud[i].z = (points_odom[i].z);
//      pclcloud[i].r = r;
//      pclcloud[i].g = g;
//      pclcloud[i].b = b;
//      }
  }
   // ROS_INFO("%10.5f\t%10.5f\t%10.5d\t%10.5d\t%10.5d", px.y, px.x, b, g, r);
    points_cam.clear();
    points_odom.clear();
    cloud_pub.publish(pclcloud);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "proj_img_to_pc");
  tf2_ros::TransformListener tf_listener(tfbuffer);
  PlyAssembler stf;
  ros::spin();
}
