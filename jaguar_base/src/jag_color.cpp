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
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace message_filters;
typedef pcl::PointCloud<pcl::PointXYZRGB> PCLCloud;
tf2_ros::Buffer tfbuffer;
//PCLCloud full_cloud;
ros::Publisher cloud_pub;
pcl::PCLPointCloud2::Ptr full_cloud(new pcl::PCLPointCloud2);

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

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PCLPointCloud2> ext;
  
  PCLCloud pcl_cloud;
  pcl::PointCloud<pcl::PointXYZ> pc_xyz;
  pcl::fromROSMsg(*cloud, pc_xyz);
  pcl::copyPointCloud(pc_xyz, pcl_cloud);

  for(int i = 0; i < pcl_cloud.size() ; ++i)
  {
    geometry_msgs::PointStamped pt_velo, pt_usb;
    pt_velo.header.frame_id = "velodyne";
    pt_velo.point.x = pc_xyz.points[i].x;
    pt_velo.point.y = pc_xyz.points[i].y;
    pt_velo.point.z = pc_xyz.points[i].z;
    try {
      tfbuffer.transform(pt_velo, pt_usb, "usb_cam");
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    cv::Point3d xyz(pt_usb.point.x,
                    pt_usb.point.y,
                    pt_usb.point.z);

//    cv::Point3d xyz(pc_xyz.points[i].z,
//                    pc_xyz.points[i].y,
//                    pc_xyz.points[i].x);

    cv::Point2d imagePoint = cam_model.project3dToPixel(xyz);
//    ROS_INFO("h:%10.5f  w:%10.5f ", pt_usb.point.x, pt_usb.point.y);
    if(imagePoint.x > 0 && imagePoint.x < info->width
       && imagePoint.y > 0 && imagePoint.y < info->height
       && pc_xyz.points[i].x > 0){
      int u = static_cast<int>(imagePoint.x);
      int v = static_cast<int>(imagePoint.y);
      cv::Vec3b colour = cv_ptr->image.at<cv::Vec3b>(v + 45, u - 6);

      uchar b = colour.val[0];
      uchar g = colour.val[1];
      uchar r = colour.val[2];

//      ROS_INFO("r:%10.5d  g:%10.5d  b:%10.5d", r, g, b);
      pcl_cloud.points[i].r = r;
      pcl_cloud.points[i].g = g;
      pcl_cloud.points[i].b = b;
    }
    else {
      inliers->indices.push_back(i);
    }

  }  
  geometry_msgs::TransformStamped lid_odom;
  try {
    lid_odom = tfbuffer.lookupTransform("camera_odom_frame", "velodyne", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  tf::Transform lid_odom_tf;
  tf::transformMsgToTF(lid_odom.transform, lid_odom_tf);
  PCLCloud cloud_out;
  pcl_ros::transformPointCloud(pcl_cloud, cloud_out, lid_odom_tf);

  cloud_out.header.frame_id = "camera_odom_frame";
  pcl::PCLPointCloud2::Ptr pcl_partial(new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(cloud_out, *pcl_partial);

  ext.setInputCloud(pcl_partial);
  ext.setIndices(inliers);
  ext.setNegative(true);
  ext.filter(*pcl_partial);

  pcl::concatenatePointCloud(*full_cloud, *pcl_partial, *full_cloud);

  pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
  vg.setInputCloud(full_cloud);
  vg.setLeafSize(0.02f, 0.02f, 0.02f);
  vg.filter(*full_cloud);
//  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> outlier;
//  outlier.setInputCloud(full_cloud);
//  outlier.setMeanK(50);
//  outlier.setStddevMulThresh(2.0);
//  outlier.filter(*full_cloud);

  cloud_pub.publish(full_cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jag_color");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  tf2_ros::TransformListener tf_listener(tfbuffer);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("pclcloud", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "usb_cam/image_rect_color", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "usb_cam/camera_info", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "velodyne_points", 1);
  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> SyncPolicy;
  Synchronizer<SyncPolicy> sync(SyncPolicy(10), image_sub, info_sub, cloud_sub);
  sync.registerCallback(callback);
  ros::spin();
}
