#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace rviz_visual_tools
{
class RvizVisual
{
  private:
    ros::NodeHandle n_;
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  public:
    RvizVisual()
    {
      visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("usb_cam","/rviz_visual"));
      visual_tools_->loadMarkerPub();
      visual_tools_->enableBatchPublishing();
      visual_tools_->enableFrameLocking();
    }

  void publishPlane(double size)
  {

   //Set base pose for lines
   Eigen::Isometry3d pose_base = Eigen::Isometry3d::Identity();

   Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

   //Create plane
   pose.translation().z() = 3;
   visual_tools_->publishXYPlane(pose, rviz_visual_tools::RED, size);

   //Create lines
   pose.translation().x() = size;
   pose.translation().y() = size;
   visual_tools_->publishLine(pose_base, pose, rviz_visual_tools::RED);

   pose.translation().x() = -size;
   pose.translation().y() = size;
   visual_tools_->publishLine(pose_base, pose, rviz_visual_tools::RED);

   pose.translation().x() = size;
   pose.translation().y() = -size;
   visual_tools_->publishLine(pose_base, pose, rviz_visual_tools::RED);

   pose.translation().x() = -size;
   pose.translation().y() = -size;
   visual_tools_->publishLine(pose_base, pose, rviz_visual_tools::RED);

   double y_shift = 0.35;
   double x_shift = -0.028;
   double z_shift = 3.1;
   //Create plane2
   pose_base.translation().y() = y_shift;
   pose_base.translation().x() = x_shift;
   pose_base.translation().z() = 0.1;
   pose.translation().y() = y_shift;
   pose.translation().x() = x_shift;
   pose.translation().z() = z_shift;
   visual_tools_->publishXYPlane(pose, rviz_visual_tools::BLUE, size);

   //Create lines2
   pose.translation().x() = size + x_shift;
   pose.translation().y() = size + y_shift;
 //  pose.translation().z() = size + z_shift;
   visual_tools_->publishLine(pose_base, pose, rviz_visual_tools::BLUE);

   pose.translation().x() = -size + x_shift;
   pose.translation().y() = size + y_shift;
//   pose.translation().z() = size + z_shift;
   visual_tools_->publishLine(pose_base, pose, rviz_visual_tools::BLUE);

   pose.translation().x() = size + x_shift;
   pose.translation().y() = -size + y_shift;
 //  pose.translation().z() = size + z_shift;
   visual_tools_->publishLine(pose_base, pose, rviz_visual_tools::BLUE);

   pose.translation().x() = -size + x_shift;
   pose.translation().y() = -size + y_shift;
 //  pose.translation().z() = size + z_shift;
   visual_tools_->publishLine(pose_base, pose, rviz_visual_tools::BLUE);

   visual_tools_->trigger();
  }
}; //Class end
} //Namespace end

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fov_project");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ROS_INFO("Node started!");

  rviz_visual_tools::RvizVisual obj;

  obj.publishPlane(0.5);

}
