#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/voxel_grid.h>

using namespace message_filters;
typedef pcl::PointCloud<pcl::PointXYZRGB> PCLCloud;
tf2_ros::Buffer tfbuffer;
PCLCloud full_cloud;
ros::Publisher cloud_pub;

void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoPtr& info, const sensor_msgs::PointCloud2Ptr& cloud)
{
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(info);
  cv_bridge::CvImagePtr input_bridge;

  cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
    cv_ptr = cv_bridge::toCvShare(image, "bgr8");
  }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  PCLCloud pcl_cloud;
  pcl::PointCloud<pcl::PointXYZ> pc_xyz;
  pcl::fromROSMsg(*cloud, pc_xyz);
  pcl::copyPointCloud(pc_xyz, pcl_cloud);

  for(size_t i = 0; i < pcl_cloud.size() ; ++i)
  {
    cv::Point3d xyz(pc_xyz.points[i].z,
                    pc_xyz.points[i].x,
                    pc_xyz.points[i].y);
    cv::Point2d imagePoint = cam_model.project3dToPixel(xyz);
    //ROS_INFO("h:%10.5f  w:%10.5f ", imagePoint.x, imagePoint.y);
    if(imagePoint.x > 0 && imagePoint.x < info->width
       && imagePoint.y > 0 && imagePoint.y < info->height){
      int u = static_cast<int>(imagePoint.x);
      int v = static_cast<int>(imagePoint.y);
      cv::Vec3b colour = cv_ptr->image.at<cv::Vec3b>(u, v);

      uchar b = colour.val[0];
      uchar g = colour.val[1];
      uchar r = colour.val[2];

      ROS_INFO("r:%10.5d  g:%10.5d  b:%10.5d", r, g, b);
      pcl_cloud.points[i].r = r;
      pcl_cloud.points[i].g = g;
      pcl_cloud.points[i].b = b;
    }
    else {
      pcl_cloud.points[i].r = 0;
      pcl_cloud.points[i].g = 255;
      pcl_cloud.points[i].b = 0;
    }
//    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
//    vg.setInputCloud(full_cloud);
//    vg.setLeafSize(0.05f, 0.05f, 0.05f);
//    vg.filter(*full_cloud);
  }
//  full_cloud.operator+(pcl_cloud);
  cloud_pub.publish(pcl_cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jag_color");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  tf2_ros::TransformListener tf_listener(tfbuffer);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("pclcloud", 100);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "usb_cam/image_rect_color", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "usb_cam/camera_info", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "velodyne_points", 1);
  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> SyncPolicy;
  Synchronizer<SyncPolicy> sync(SyncPolicy(10), image_sub, info_sub, cloud_sub);
  sync.registerCallback(callback);
  ros::spin();
}
