#ifndef __POSE_GRAPH_MAP_H
#define __POSE_GRAPH_MAP_H

#include <memory>

#include <unordered_set>
#include <vector>
#include <list>
#include "../../core/states/sensor_data.h"
#include "../../core/states/state_data.h"
//#include "scan_diff_estimator.h"

struct PoseGraphNode;
struct PoseGraphEdge;

using NodePtr = std::shared_ptr<PoseGraphNode>;
using EdgePtr = std::shared_ptr<PoseGraphEdge>;

struct PoseGraphEdge {
  PoseGraphEdge(NodePtr from_node, NodePtr to_node, RobotPoseDelta delta) :
    pose_delta(delta), from(from_node), to(to_node) {}

  RobotPoseDelta pose_delta;
  NodePtr from, to;
};

struct PoseGraphNode {
  TransformedLaserScan scan;
  RobotPose pose;
  std::list<std::shared_ptr<PoseGraphEdge>> edges;
};

#include <iostream>
#include <memory>
#include <vector>
#include "../../core/states/sensor_data.h"

// TODO: tepmlate? node/edge common iface, sensor type
class PoseGraphMap {
private: // types
  using NodePtr = std::shared_ptr<PoseGraphNode>;
public:
  // TODO: add_edge function
  PoseGraphMap(): _current_node(nullptr) {}

  void add_node(const TransformedLaserScan& scan,
                const RobotPose& pose,
                double closure_lookup_sq_dist) {

    if (close_loop(scan, pose, closure_lookup_sq_dist)) {
      std::cout << "Loop is closed" << std::endl;
      return;
    }

    std::shared_ptr<PoseGraphNode> new_node{new PoseGraphNode()};
    new_node->scan = scan;
    new_node->pose = pose;
    _nodes.push_back(new_node);

    if (!_current_node) {
      _current_node = new_node;
      return;
    }

    add_edge(new_node, _current_node);
    _current_node = new_node;
  }

  // TODO: nodes iterator (BFS?)
  const std::vector<NodePtr>& nodes() const { return _nodes;}
  const std::vector<EdgePtr>& edges() const { return _edges;}

private:

  bool close_loop(const TransformedLaserScan& scan,
                  const RobotPose& pose,
                  double max_sq_dist) {
    return false;
    /* double min_diff = 0.2; */

    /* ScanDiffEstimator sde(scan, pose); */
    /* NodePtr similar_node(nullptr); */
    /* std::cout << "Loop lookup" << std::endl; */
    /* for (auto& node : nearest_nodes(pose, max_sq_dist)) { */
    /*   double diff = sde.estimate_diff(node->scan, node->pose); */
    /*   std::cout << diff << std::endl; */
    /*   if (diff < min_diff) { */
    /*     min_diff = diff; */
    /*     similar_node = node; */
    /*   } */
    /* } */
    /* if (!similar_node) { */
    /*   return false; */
    /* } */

    /* add_edge(similar_node, _current_node); */
    /* _current_node = similar_node; */
    /* return true; */
  }

  const std::unordered_set<NodePtr> nearest_nodes(
      const RobotPose& pose, double max_sq_dist) {

    // TODO: performance bug -- bf
    std::unordered_set<NodePtr> near_nodes;
    for (auto& node : nodes()) {
      if ((node->pose - pose).sq_dist() < max_sq_dist) {
        near_nodes.insert(node);
      }
    }
    // Do we really need the current node?
    if (_current_node) {
      near_nodes.insert(_current_node);
    }
    return near_nodes;
  }

  void add_edge(NodePtr n1, NodePtr n2) {
    EdgePtr new_edge{new PoseGraphEdge(n1, n2, n1->pose - n2->pose)};
    n1->edges.push_back(new_edge);
    n2->edges.push_back(new_edge);

    _edges.push_back(new_edge);
  }

private:
  NodePtr _current_node;
  std::vector<NodePtr> _nodes;
  std::vector<EdgePtr> _edges;
};

#endif
