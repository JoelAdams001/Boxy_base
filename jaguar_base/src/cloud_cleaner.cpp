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
#include <pcl/PCLPointCloud2.h>

using namespace message_filters;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PCLCloud;
tf2_ros::Buffer tfbuffer;
pcl::PCLPointCloud2::Ptr full_cloud(new pcl::PCLPointCloud2);
ros::Publisher cloud_pub;

void callback(const sensor_msgs::PointCloud2Ptr& cloud)
{
  //pcl::PCLPointCloud2::Ptr full_cloud_temp(new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr pcl_partial(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*cloud, *pcl_partial);
  pcl::concatenatePointCloud(*full_cloud, *pcl_partial, *full_cloud);

  pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
  vg.setInputCloud(full_cloud);
  vg.setLeafSize(0.01f, 0.01f, 0.01f);
  vg.filter(*full_cloud);
  cloud_pub.publish(full_cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_cleaner");
  ros::NodeHandle nh;
  tf2_ros::TransformListener tf_listener(tfbuffer);
  ros::Subscriber cloud_sub = nh.subscribe("pclcloud", 1, callback);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("colored_cloud", 1);
  ros::spin();
}
