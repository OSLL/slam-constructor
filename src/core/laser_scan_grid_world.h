#ifndef __LASER_SCAN_GRID_WORLD_H
#define __LASER_SCAN_GRID_WORLD_H

#include <memory>

#include "sensor_data.h"
#include "state_data.h"
#include "maps/grid_cell_strategy.h"
#include "maps/grid_map.h"

class LaserScanGridWorld : public World<TransformedLaserScan, GridMap> {
public: //types
  using MapType = GridMap;
  using DPoint = DiscretePoint2D;
public: // methods

  LaserScanGridWorld(std::shared_ptr<GridCellStrategy> gcs,
                    size_t scan_margin = 0) :
    _gcf(gcs->cell_factory()), _map(_gcf), _scan_margin(scan_margin) {}

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
    DPoint robot_pt = map.world_to_cell(lsr.x, lsr.y);
    DPoint obst_pt = map.world_to_cell(beam_end.x, beam_end.y);

    std::vector<DPoint> pts = DiscreteLine2D(robot_pt, obst_pt).points();

    std::shared_ptr<GridCellValue> cell_value = _gcf->create_cell_value();
    const DPoint &beam_end_pt = pts.back();
    const Rectangle &beam_end_pt_bnds = map.world_cell_bounds(beam_end_pt);

    cell_value->reset();
    setup_cell_value(*cell_value, beam_end_pt, beam_end_pt_bnds,
                     is_occ, lsr, beam_end);
    map.update_cell(beam_end_pt, *cell_value, scan_quality);
    pts.pop_back();

    for (const auto &pt : pts) {
      const Rectangle pt_bnds = map.world_cell_bounds(pt);

      cell_value->reset();
      setup_cell_value(*cell_value, pt, pt_bnds, false, lsr, beam_end);
      map.update_cell(pt, *cell_value, scan_quality);
    }
  }

  virtual GridCellValue& setup_cell_value(
      GridCellValue &dst, const DPoint &pt, const Rectangle &pt_bounds,
      bool is_occ, const Point2D &lsr, const Point2D &obstacle) {

    dst.occupancy = Occupancy{is_occ ? 1.0 : 0.0, 1.0};
    return dst;
  }

  virtual const MapType& map() const { return _map; }
  virtual MapType& map() { return _map; }

private: // fields
  std::shared_ptr<GridCellFactory> _gcf;
  MapType _map;
  size_t _scan_margin;
};

#endif
