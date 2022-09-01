#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/Odometry.h>

#include <fstream>

geometry_msgs::Vector3 getEstimateTfOdom(tf2_ros::Buffer *tf_buffer) {
  geometry_msgs::TransformStamped transformStamped;
  while (not tf_buffer->canTransform("map", "base_footprint", ros::Time::now(),
                                     ros::Duration(0.1))) {
  };
  try {
    transformStamped = tf_buffer->lookupTransform(
        "map", "base_footprint", ros::Time::now() - ros::Duration(0.1));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
  return transformStamped.transform.translation;
}

void writeCsvTfPose(geometry_msgs::Point ground_truth_tf_odom,
                    tf2_ros::Buffer *tf_buffer) {
  auto EstimateTfOdom = getEstimateTfOdom(tf_buffer);
  std::ofstream ofs;
  ofs.open("odom_comparison_data.csv", std::ios::app);
  ofs << ground_truth_tf_odom.x << ", " << ground_truth_tf_odom.y << ", "
      << EstimateTfOdom.x << ", " << EstimateTfOdom.y << "\n";
}

void broadcasteTfGroundTruth(const nav_msgs::Odometry::ConstPtr &msg,
                             tf2_ros::TransformBroadcaster &tf_broadcaster,
                             tf2_ros::Buffer *tf_buffer) {

  geometry_msgs::TransformStamped map2odom;

  map2odom.header.frame_id = "map";
  map2odom.child_frame_id = "odom_ground_truth";
  map2odom.transform.translation.x = 0.0;
  map2odom.transform.translation.y = 0.0;
  map2odom.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  map2odom.transform.rotation.x = q.x();
  map2odom.transform.rotation.y = q.y();
  map2odom.transform.rotation.z = q.z();
  map2odom.transform.rotation.w = q.w();
  map2odom.header.stamp = ros::Time::now();
  tf_broadcaster.sendTransform(map2odom);

  geometry_msgs::TransformStamped odom2base_footprint;

  odom2base_footprint.header.frame_id = "odom_ground_truth";
  odom2base_footprint.child_frame_id = "base_footprint_ground_truth";
  odom2base_footprint.transform.translation.x = msg->pose.pose.position.x;
  odom2base_footprint.transform.translation.y = msg->pose.pose.position.y;
  odom2base_footprint.transform.translation.z = msg->pose.pose.position.z;

  tf2::Quaternion ground_truth_quaternion;
  ground_truth_quaternion.setRPY(0, 0, tf2::getYaw(msg->pose.pose.orientation));
  odom2base_footprint.transform.rotation = tf2::toMsg(ground_truth_quaternion);

  odom2base_footprint.header.stamp = ros::Time::now();
  tf_broadcaster.sendTransform(odom2base_footprint);

  writeCsvTfPose(msg->pose.pose.position, tf_buffer);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gazebo_ground_truth_tf_broadcaster");
  ros::NodeHandle nh;

  tf2_ros::Buffer tf_buffer(ros::Duration(10));
  tf2_ros::TransformBroadcaster tf_broadcaster;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
      "/odom", 1,
      boost::bind(broadcasteTfGroundTruth, _1, tf_broadcaster, &tf_buffer));

  ros::spin();
};