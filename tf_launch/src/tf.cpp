#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_broadcaster.h>

geometry_msgs::PoseWithCovariance pose;
double vehicle_yaw, vehicle_pitch, vehicle_roll;
double UTM_OFFSET_X_, UTM_OFFSET_Y_;

void pose_callback(const geometry_msgs::PoseWithCovariance &msg)
{
    static tf::TransformBroadcaster map_odom;
    static tf::TransformBroadcaster odom_lidar;
    static tf::TransformBroadcaster odom_local;

    pose = msg;
    pose.pose.position.x = msg.pose.position.x - UTM_OFFSET_X_;
    pose.pose.position.y = msg.pose.position.y - UTM_OFFSET_Y_;

    tf::Quaternion car_orientation(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Matrix3x3 matrix(car_orientation);
    matrix.getRPY(vehicle_roll, vehicle_pitch, vehicle_yaw);

    map_odom.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w), tf::Vector3(pose.pose.position.x, pose.pose.position.y, 0.0)),
        ros::Time::now(),"map", "odom"));

   odom_lidar.sendTransform(
     tf::StampedTransform(
       tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.7, 0.0, 0.3)),
       ros::Time::now(),"odom", "lidar"));

//   odom_local.sendTransform(
//     tf::StampedTransform(
//       tf::Transform(tf::Quaternion(0, 0, vehicle_yaw, 1), tf::Vector3(0.0, 0.0, 0.0)),
//       ros::Time::now(),"odom", "local"));

}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_make");
  ros::NodeHandle n;
  ros::NodeHandle pnh;

  pnh.param("UTM_OFFSET_X_",UTM_OFFSET_X_,361001.412425);
  pnh.param("UTM_OFFSET_Y_",UTM_OFFSET_Y_,4065821.07176);

  ros::Subscriber pose_sub = n.subscribe("/Perception/Localization/LocalPose",10, pose_callback);

  ros::spin();
  return 0;
 }
