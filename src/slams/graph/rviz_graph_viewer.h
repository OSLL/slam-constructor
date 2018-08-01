#ifndef __RVIZ_GRAPH_VIEWER_H
#define __RVIZ_GRRAPH_VIEWER_H

#include <vector>
#include <memory>
#include <algorithm>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include "../../core/states/state_data.h"
#include "pose_graph_map.h"

// TODO: move common code to RvizSlamViewer
// TODO: add incremental map update (do not send entire graph all the time)
class RvizGraphViewer : public WorldMapObserver<PoseGraphMap> {
public: // method
  RvizGraphViewer(ros::Publisher pub):
    _map_pub(pub) {}

  void on_map_update(const PoseGraphMap &map) override {
    // TODO: move map publishing rate to parameter
    if ((ros::Time::now() - _last_pub_time).toSec() < 3.0) {
      return;
    }
    visualization_msgs::MarkerArray map_msg;
    map_msg.markers.push_back(create_nodes_marker(map.nodes()));
    map_msg.markers.push_back(create_edges_marker(map.edges()));

    _map_pub.publish(map_msg);
    _last_pub_time = ros::Time::now();
  }

private: // types
  //TODO: get from GraphMap type
  using NodePtr = std::shared_ptr<PoseGraphNode>;
  using EdgePtr = std::shared_ptr<PoseGraphEdge>;
private: // methods

  visualization_msgs::Marker create_nodes_marker(
      const std::vector<NodePtr> &nodes) {
    visualization_msgs::Marker nodes_marker;

    nodes_marker.header.frame_id = "odom_combined";
    nodes_marker.header.stamp = ros::Time::now();

    nodes_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    nodes_marker.ns = "graph_map_nodes";
    nodes_marker.id = 0;
    nodes_marker.action = visualization_msgs::Marker::ADD;

    nodes_marker.color.r = nodes_marker.color.a = 1.0;
    nodes_marker.color.g = nodes_marker.color.b = 0.0;
    nodes_marker.lifetime = ros::Duration();

    nodes_marker.scale.x = nodes_marker.scale.y = nodes_marker.scale.z = 0.2;

    for (auto& node : nodes) {
      nodes_marker.points.push_back(make_2D_point(node));
    }

    return nodes_marker;
  }

  visualization_msgs::Marker create_edges_marker(
      const std::vector<EdgePtr>& edges) {
    visualization_msgs::Marker edges_marker;

    edges_marker.header.frame_id = "odom_combined";
    edges_marker.header.stamp = ros::Time::now();

    edges_marker.type = visualization_msgs::Marker::LINE_LIST;
    edges_marker.ns = "graph_map_edges";
    edges_marker.id = 0;
    edges_marker.action = visualization_msgs::Marker::ADD;

    edges_marker.color.r = 0.0;
    edges_marker.color.g = 1.0;
    edges_marker.color.b = 1.0;
    edges_marker.color.a = 1.0;

    edges_marker.lifetime = ros::Duration();

    edges_marker.scale.x = 0.07;
    edges_marker.scale.y = edges_marker.scale.z = 0.0;

    for (auto& edge : edges) {
      edges_marker.points.push_back(make_2D_point(edge->from));
      edges_marker.points.push_back(make_2D_point(edge->to));
    }

    return edges_marker;
  }

  inline geometry_msgs::Point make_2D_point(
      const std::shared_ptr<PoseGraphNode>& node) {
    geometry_msgs::Point p;
    p.x = node->pose.x;
    p.y = node->pose.y;
    p.z = 0.0;
    return p;
  }

private: // fields
  ros::Publisher _map_pub;
  ros::Time _last_pub_time;
  tf::TransformBroadcaster _tf_brcst;
};

#endif
