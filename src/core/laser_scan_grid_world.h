#ifndef __LASER_SCAN_GRID_WORLD_H
#define __LASER_SCAN_GRID_WORLD_H

#include <memory>
#include <utility>

#include "sensor_data.h"
#include "state_data.h"
#include "maps/grid_cell_strategy.h"
#include "maps/grid_map.h"

template <typename Map>
class LaserScanGridWorld : public World<TransformedLaserScan, Map> {
public: //types
  using MapType = Map;
  using DPoint = DiscretePoint2D;
public: // methods

  LaserScanGridWorld(std::shared_ptr<GridCellStrategy> gcs,
                    size_t scan_margin = 0) :
    _map(gcs->cell_prototype()), _scan_margin(scan_margin) {}

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
                        Beam{{pose.x, pose.y}, {x_world, y_world}});
    }
  }

  virtual void handle_scan_point(bool is_occ, double scan_quality,
                                 const Beam &beam) {
    auto &map = this->map();
    auto pts = std::move(DiscreteLine2D{map.world_to_cell(beam.beg),
                                        map.world_to_cell(beam.end)}.points());
    map[pts.back()] += sp2obs(pts.back(), is_occ, scan_quality, beam);
    pts.pop_back();

    for (const auto &pt : pts) {
      map[pt] += sp2obs(pt, false, scan_quality, beam);
    }
  }

  virtual AreaOccupancyObservation sp2obs(
    const DPoint &, bool is_occ, double quality, const Beam &beam) const {
    return AreaOccupancyObservation{{(double)is_occ, 1.0}, beam.end, quality};
  }

  virtual const MapType& map() const { return _map; }
  virtual MapType& map() { return _map; }

private: // fields
  MapType _map;
  size_t _scan_margin;
};

#endif
