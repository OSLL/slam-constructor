#ifndef SLAM_CTOR_ROS_OCCUPANCY_GRID_PUBLISHER_H
#define SLAM_CTOR_ROS_OCCUPANCY_GRID_PUBLISHER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include "../core/states/state_data.h"
#include "../core/maps/grid_map.h"

template <typename GridMapType>
class OccupancyGridPublisher : public WorldMapObserver<GridMapType> {
public: // method
  OccupancyGridPublisher(ros::Publisher pub,
                         const std::string &tf_map_frame_id,
                         double publ_interval_secs = 5.0):
    _map_pub{pub}, _tf_map_frame_id{tf_map_frame_id},
    _publishing_interval{publ_interval_secs} {}

  void on_map_update(const GridMapType &map) override {
    if ((ros::Time::now() - _last_pub_time) < _publishing_interval) {
      return;
    }

    nav_msgs::OccupancyGrid map_msg;
    map_msg.header.frame_id = _tf_map_frame_id;
    map_msg.info.map_load_time = ros::Time::now();
    map_msg.info.width = map.width();
    map_msg.info.height = map.height();
    map_msg.info.resolution = map.scale();
    // move map to the middle
    nav_msgs::MapMetaData &info = map_msg.info;
    DiscretePoint2D origin = map.origin();
    info.origin.position.x = -info.resolution * origin.x;
    info.origin.position.y = -info.resolution * origin.y;
    info.origin.position.z = 0;
    map_msg.data.reserve(info.height * info.width);
    DiscretePoint2D pnt;
    DiscretePoint2D end_of_map = DiscretePoint2D(info.width,
                                                 info.height) - origin;
    for (pnt.y = -origin.y; pnt.y < end_of_map.y; ++pnt.y) {
      for (pnt.x = -origin.x; pnt.x < end_of_map.x; ++pnt.x) {
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
  std::string _tf_map_frame_id;
  ros::Time _last_pub_time;
  ros::Duration _publishing_interval;
};

#endif
