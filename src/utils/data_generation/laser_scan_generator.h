#ifndef SLAM_CTOR_UTILS_DG_LASER_SCAN_GENERATOR_H_INCLUDED
#define SLAM_CTOR_UTILS_DG_LASER_SCAN_GENERATOR_H_INCLUDED

#include <memory>
#include <cmath>

#include "../../core/math_utils.h"
#include "../../core/maps/grid_map.h"
#include "../../core/sensor_data.h"
#include "../../core/geometry_utils.h"

struct LaserScannerParams {
  LaserScannerParams(double mx_dist, double h_a_inc, double h_half_sector)
    : max_dist{mx_dist}, h_angle_inc{h_a_inc}, h_hsector{h_half_sector} {}
  LaserScannerParams() : LaserScannerParams{15, deg2rad(90), deg2rad(180)} {}

  double max_dist; // meters
  // "h" is for "horizontal"
  double h_angle_inc; // radians
  double h_hsector; // half sector, radians
};

class LaserScanGenerator {
public:
  LaserScanGenerator(LaserScannerParams ls_params = {}) : _lsp{ls_params} {}

  TransformedLaserScan generate_2D_laser_scan(
      const GridMap& map, const RobotPose& pose, double occ_threshold = 1) {

   TransformedLaserScan scan;
   scan.quality = 1.0;
   scan.trig_cache = std::make_shared<TrigonometricCache>();
   scan.trig_cache->update(-_lsp.h_hsector, _lsp.h_hsector, _lsp.h_angle_inc);

   auto robot_point = Point2D{pose.x, pose.y};
   auto hhsector = _lsp.h_hsector; // horiz. half-sector
   for (double a = -hhsector; a <= hhsector; a += _lsp.h_angle_inc) {
     if (2*M_PI <= hhsector + a) { break; }
     auto beam_end = robot_point +
                     Point2D{_lsp.max_dist * std::cos(a + pose.theta),
                             _lsp.max_dist * std::sin(a + pose.theta)};
     for (auto& coord : map.world_to_cells(Segment2D{robot_point, beam_end})) {
       if (map[coord] < occ_threshold) { continue; }

       auto coord_dist = coord - map.world_to_cell(pose.x, pose.y);
       auto range = map.scale() * std::sqrt(std::pow(coord_dist.x, 2) +
                                            std::pow(coord_dist.y, 2));
       scan.points.emplace_back(range, a);
       //std::cout << "==> Add point " << range << " " << a << std::endl;
       break;
     }
   }
   return scan;
  }

private:
  LaserScannerParams _lsp;
};

#endif
