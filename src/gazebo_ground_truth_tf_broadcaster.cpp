#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

geometry_msgs::TransformStamped getTfMap2Odom(tf2_ros::Buffer &tf_buffer) {
  geometry_msgs::TransformStamped transformStamped;
  while (not tf_buffer.canTransform("map", "odom", ros::Time::now(),
                                    ros::Duration(0.1))) {
  };
  try {
    transformStamped = tf_buffer.lookupTransform(
        "map", "odom", ros::Time::now() - ros::Duration(0.1));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
  return transformStamped;
}

geometry_msgs::TransformStamped getTfOdom2Robot(tf2_ros::Buffer &tf_buffer) {
  geometry_msgs::TransformStamped transformStamped;
  while (not tf_buffer.canTransform("odom", "base_footprint", ros::Time::now(),
                                    ros::Duration(0.1))) {
  };
  try {
    transformStamped = tf_buffer.lookupTransform(
        "odom", "base_footprint", ros::Time::now() - ros::Duration(0.1));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
  return transformStamped;
}

void broadcasteTfGroundTruth(
    tf2_ros::Buffer &tf_buffer, tf2_ros::TransformBroadcaster &tf_broadcaster,
    geometry_msgs::TransformStamped transform_map_odom,
    geometry_msgs::TransformStamped transform_odom_robot) {

  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.frame_id = "odom_ground_truth";
  transformStamped.child_frame_id = "base_link_ground_truth";
  transformStamped.transform.translation.x =
      transform_map_odom.transform.translation.x +
      transform_odom_robot.transform.translation.x;
  transformStamped.transform.translation.y =
      transform_map_odom.transform.translation.y +
      transform_odom_robot.transform.translation.y;
  transformStamped.transform.translation.z =
      transform_map_odom.transform.translation.z +
      transform_odom_robot.transform.translation.z;
  tf2::Quaternion odom_ground_truth_quaternion,
      base_link_ground_truth_quaternion, ground_truth_quaternion;
  odom_ground_truth_quaternion.setRPY(
      0, 0, tf2::getYaw(transform_map_odom.transform.rotation));
  base_link_ground_truth_quaternion.setRPY(
      0, 0, tf2::getYaw(transform_odom_robot.transform.rotation));

  ground_truth_quaternion =
      odom_ground_truth_quaternion + base_link_ground_truth_quaternion;
  ground_truth_quaternion.normalize();
  transformStamped.transform.rotation.x = ground_truth_quaternion.x();
  transformStamped.transform.rotation.y = ground_truth_quaternion.y();
  transformStamped.transform.rotation.z = ground_truth_quaternion.z();
  transformStamped.transform.rotation.w = ground_truth_quaternion.w();

  transformStamped.header.stamp = ros::Time::now();
  tf_broadcaster.sendTransform(transformStamped);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gazebo_ground_truth_tf_broadcaster");
  ros::NodeHandle node;

  tf2_ros::Buffer tf_buffer(ros::Duration(100));
  tf2_ros::TransformBroadcaster tf_broadcaster;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "odom_ground_truth";
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  ros::Rate rate(1.0);
  while (node.ok()) {
    transformStamped.header.stamp = ros::Time::now();
    tf_broadcaster.sendTransform(transformStamped);
    auto transform_map_odom = getTfMap2Odom(tf_buffer);
    auto transform_odom_robot = getTfOdom2Robot(tf_buffer);
    broadcasteTfGroundTruth(tf_buffer, tf_broadcaster, transform_map_odom,
                            transform_odom_robot);
    rate.sleep();
  }
};