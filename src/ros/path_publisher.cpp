#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <vector>
#include <string>
#include "init_utils.h"

static double tf_wait_for_transform_duration() {
  double wait_for_transform_duration;
  ros::param::param<double>("wait_for_transform_duration", wait_for_transform_duration, 3.0);

  return wait_for_transform_duration;
}

static std::string tf_base_link_frame_id() {
  std::string base_link_frame_id;
  ros::param::param<std::string>("base_link_frame_id", base_link_frame_id, "base_link");

  return base_link_frame_id;
}

static int subscriber_queue_size() {
  int subscriber_queue_size;
  ros::param::param<int>("subscribers_queue_size", subscriber_queue_size, 1000);

  return subscriber_queue_size;
}

static double publishing_rate() {
  double publishing_rate;
  ros::param::param<double>("publishing_rate", publishing_rate, 5.0);

  return publishing_rate;
}

static std::string path_topic_name() {
  std::string path_topic_name;
  ros::param::param<std::string>("path_topic_name", path_topic_name, "path");

  return path_topic_name;
}

// Returns stamped base_link pose.
// In case of failure reports an error and does retries.
// If ros::shutdown is issued, will return a dummy pose object.
static geometry_msgs::PoseStamped get_current_pose() {
  tf::TransformListener tf_listener;
  tf::StampedTransform stamped_transform;

  while (ros::ok()) {
    try {
      auto stamp_latest = ros::Time(0);
      auto map_frame_id = tf_map_frame_id();
      auto robot_frame_id = tf_base_link_frame_id();

      tf_listener.waitForTransform(map_frame_id, robot_frame_id, stamp_latest, ros::Duration(tf_wait_for_transform_duration()));
      tf_listener.lookupTransform(map_frame_id, robot_frame_id, stamp_latest, stamped_transform);

      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = stamped_transform.frame_id_;
      pose_stamped.header.stamp = stamped_transform.stamp_;
      pose_stamped.pose.position.x = stamped_transform.getOrigin().x();
      pose_stamped.pose.position.y = stamped_transform.getOrigin().y();

      return pose_stamped;
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }

  return geometry_msgs::PoseStamped();
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "path_publisher");

  auto node_handle = ros::NodeHandle();
  auto publisher = node_handle.advertise<nav_msgs::Path>(path_topic_name(), subscriber_queue_size());
  auto loop_rate = ros::Rate(publishing_rate());

  nav_msgs::Path path;
  path.header.frame_id = tf_map_frame_id();

  while(node_handle.ok()) {
    auto pose_stamped = get_current_pose();
    path.header.stamp = pose_stamped.header.stamp;
    path.poses.push_back(pose_stamped);

    publisher.publish(path);

    loop_rate.sleep();
  }

  return 0;
}
