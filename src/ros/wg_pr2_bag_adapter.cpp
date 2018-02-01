#include <utility>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>

#include "launch_properties_provider.h"
#include "init_utils.h"

using OccGridMsg = nav_msgs::OccupancyGrid;
OccGridMsg provided_grid;

geometry_msgs::TransformStamped gt2map_transform;
bool gt2map_is_initialized = false;

void on_gt_grid(boost::shared_ptr<nav_msgs::OccupancyGrid> msg) {
  provided_grid = std::move(*msg);
  provided_grid.header.frame_id = "provided_map";
}

void on_tf_msg(boost::shared_ptr<tf::tfMessage> msg) {
  auto props = LaunchPropertiesProvider{};
  static tf::TransformBroadcaster br;
  static std::string map_frame_id = "/" + tf_map_frame_id();
  static std::string odom_frame_id = "/" + tf_odom_frame_id(props);

  for (auto &t : msg->transforms) {
    if (t.header.frame_id != map_frame_id ||
        t.child_frame_id != odom_frame_id) {
      br.sendTransform(t);
      continue;
    }
    // handle pose correction provided by bag:
    // - the correction is ignored, initial map->odom is sent instead
    if (!gt2map_is_initialized) {
      gt2map_transform = std::move(t);
      gt2map_transform.header.frame_id = "/provided_map";
      gt2map_transform.child_frame_id = "/map";
      gt2map_is_initialized = true;
    }
    br.sendTransform(gt2map_transform);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "WG-PR2-bag-adapter");

  ros::NodeHandle nh;

  auto provided_grid_pub = nh.advertise<OccGridMsg>("provided_grid", 10);
  auto provided_grid_sub = nh.subscribe("provided_grid_orig", 10, on_gt_grid);

  auto tf_sub = nh.subscribe("tf_in", 10, on_tf_msg);

  ros::Rate rate{7};
  while (ros::ok()) {
    provided_grid_pub.publish(provided_grid);
    ros::spinOnce();
    rate.sleep();
  }
}
