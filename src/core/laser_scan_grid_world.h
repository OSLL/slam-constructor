#ifndef __LASER_SCAN_GRID_WORLD_H
#define __LASER_SCAN_GRID_WORLD_H

#include <memory>
#include <utility>

#include "sensor_data.h"
#include "world.h"
#include "maps/grid_cell_strategy.h"
#include "maps/grid_map.h"

template <typename Map>
class LaserScanGridWorld : public World<TransformedLaserScan, Map> {
public: //types
  using MapType = Map;
  using ScanType = TransformedLaserScan;
  using DPoint = DiscretePoint2D;
  using ScanMatcherObsPtr = std::shared_ptr<GridScanMatcherObserver>;
public: // methods

  LaserScanGridWorld(std::shared_ptr<GridCellStrategy> gcs,
                     const GridMapParams& params,
                     size_t scan_margin = 0) :
    _map(gcs->cell_prototype(), params), _scan_margin(scan_margin) {}


  virtual std::shared_ptr<GridScanMatcher> scan_matcher() { return nullptr; }

  void handle_sensor_data(TransformedLaserScan &scan) override {
    this->update_robot_pose(scan.pose_delta);
    handle_observation(scan);

    this->notify_with_pose(this->pose());
    this->notify_with_map(map());
  }

  void add_scan_matcher_observer(ScanMatcherObsPtr obs) {
    auto sm = scan_matcher();
    if (sm) { sm->subscribe(obs); }
  }

  void remove_scan_matcher_observer(ScanMatcherObsPtr obs) {
    auto sm = scan_matcher();
    if (sm) { sm->unsubscribe(obs); }
  }


protected:

  virtual void handle_observation(TransformedLaserScan &scan) {
    const RobotPose& pose = World<TransformedLaserScan, MapType>::pose();

    scan.trig_cache->set_theta(pose.theta);
    size_t last_pt_i = scan.points.size() - _scan_margin - 1;
    for (size_t pt_i = _scan_margin; pt_i <= last_pt_i; ++pt_i) {
      const ScanPoint &sp = scan.points[pt_i];
      // move to world frame assume sensor is in robots' (0,0)
      double c = scan.trig_cache->cos(sp.angle);
      double s = scan.trig_cache->sin(sp.angle);

      double x_world = pose.x + sp.range * c;
      double y_world = pose.y + sp.range * s;

      handle_scan_point(sp.is_occupied, scan.quality,
                        {{pose.x, pose.y}, {x_world, y_world}});
    }
  }

  virtual void handle_scan_point(bool is_occ, double scan_quality,
                                 const Segment2D &beam) {
    auto &map = this->map();
    auto pts = map.world_to_cells(beam);
    map[pts.back()] += sp2obs(pts.back(), is_occ, scan_quality, beam);
    pts.pop_back();

    for (const auto &pt : pts) {
      map[pt] += sp2obs(pt, false, scan_quality, beam);
    }
  }

  virtual AreaOccupancyObservation sp2obs(
    const DPoint &, bool is_occ, double quality, const Segment2D &beam) const {
    return AreaOccupancyObservation{is_occ, {(double)is_occ, 1.0},
                                    beam.end(), quality};
  }

  virtual const MapType& map() const { return _map; }
  virtual MapType& map() { return _map; }

private: // fields
  MapType _map;
  size_t _scan_margin;
};

#endif
