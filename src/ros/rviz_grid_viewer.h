#ifndef __RVIZ_GRID_VIEWER_H
#define __RVIZ_GRID_VIEWER_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>

#include "../core/state_data.h"
#include "../core/slam_fascade.h"
#include "../core/maps/grid_map.h"
#include "topic_with_transform.h"

template <typename GridMapType>
class RvizGridViewer : public WorldObserver<GridMapType>,
                       public TopicObserver<sensor_msgs::LaserScan> {
public: // method
  RvizGridViewer(ros::Publisher pub): _map_pub(pub),
    _last_odom2base(tf::createQuaternionFromRPY(0, 0, 0),
                    tf::Vector3(0, 0, 0)){}

  // TODO: move 2D transform publishing part to a separate component

  virtual void handle_transformed_msg(
    const boost::shared_ptr<sensor_msgs::LaserScan>,
    const tf::StampedTransform& t) override {
    _last_odom2base = t;
    _last_odom2base.getOrigin().setZ(0);
  }

  virtual void on_pose_update(const RobotPose &rs) override {
    tf::Transform base2map = tf::Transform{
      tf::createQuaternionFromRPY(0, 0, rs.theta),
      tf::Vector3(rs.x, rs.y, 0.0)}.inverse();
    tf::Transform map2odom = (_last_odom2base * base2map).inverse();

    _tf_brcst.sendTransform(
      tf::StampedTransform(map2odom, ros::Time::now(),
                           "map","odom_combined"));
  }

  virtual void on_map_update(const GridMapType &map) override {
    // TODO: move map publishing rate to parameter
    if ((ros::Time::now() - _last_pub_time).toSec() < 5.0) {
      return;
    }

    nav_msgs::OccupancyGrid map_msg;
    map_msg.info.map_load_time = ros::Time::now();
    map_msg.info.width = map.width();
    map_msg.info.height = map.height();
    map_msg.info.resolution = map.scale();
    // move map to the middle
    nav_msgs::MapMetaData &info = map_msg.info;
    info.origin.position.x = -info.resolution * info.height / 2;
    info.origin.position.y = -info.resolution * info.width  / 2;
    info.origin.position.z = 0;

    map_msg.data.reserve(info.height * info.width);
    DiscretePoint2D pnt;
    for (pnt.y = 0; pnt.y < map.height(); ++pnt.y) {
      for (pnt.x = 0; pnt.x < map.width(); ++pnt.x) {
        double value = (double)map[pnt];
        int cell_value = value == -1 ? -1 : value * 100;
        map_msg.data.push_back(cell_value);
      }
    }

    _map_pub.publish(map_msg);
    _last_pub_time = ros::Time::now();
  }

private: // fields
  ros::Publisher _map_pub;
  ros::Time _last_pub_time;
  tf::TransformBroadcaster _tf_brcst;
  tf::Transform _last_odom2base;
};

#endif
