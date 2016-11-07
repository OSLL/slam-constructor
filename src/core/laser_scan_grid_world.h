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

      handle_scan_point(map(), sp.is_occupied, scan.quality,
                        Point2D{pose.x, pose.y}, Point2D{x_world, y_world});
    }
  }

  virtual void handle_scan_point(MapType &map, bool is_occ, double scan_quality,
                                 const Point2D &lsr, const Point2D &beam_end) {
    const DPoint robot_pt = map.world_to_cell(lsr.x, lsr.y);
    const DPoint obst_pt = map.world_to_cell(beam_end.x, beam_end.y);

    std::vector<DPoint> pts = std::move(
      DiscreteLine2D(robot_pt, obst_pt).points());

    const DPoint &beam_end_pt = pts.back();
    const Rectangle &beam_end_pt_bnds = map.world_cell_bounds(beam_end_pt);

    std::unique_ptr<GridCell> new_value = _map.new_cell();
    new_value->quality = scan_quality;

    setup_cell_value(*new_value, beam_end_pt, beam_end_pt_bnds,
                     is_occ, lsr, beam_end);
    map[beam_end_pt] += *new_value;
    pts.pop_back();

    for (const auto &pt : pts) {
      const Rectangle pt_bnds = map.world_cell_bounds(pt);
      setup_cell_value(*new_value, pt, pt_bnds, false, lsr, beam_end);
      map[pt] += *new_value;
    }
  }

  virtual GridCell& setup_cell_value(
      GridCell &dst, const DPoint &pt, const Rectangle &pt_bounds,
      bool is_occ, const Point2D &lsr, const Point2D &obstacle) {

    dst.occupancy = Occupancy{is_occ ? 1.0 : 0.0, 1.0};
    return dst;
  }

  virtual const MapType& map() const { return _map; }
  virtual MapType& map() { return _map; }

private: // fields
  MapType _map;
  size_t _scan_margin;
};

#endif
