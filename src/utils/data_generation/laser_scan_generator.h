#ifndef SLAM_CTOR_UTILS_DG_LASER_SCAN_GENERATOR_H
#define SLAM_CTOR_UTILS_DG_LASER_SCAN_GENERATOR_H

#include <memory>
#include <cmath>

#include "../../core/math_utils.h"
#include "../../core/maps/grid_map.h"
#include "../../core/states/sensor_data.h"
#include "../../core/geometry_utils.h"

struct LaserScannerParams {
  constexpr LaserScannerParams(double mx_dist, double h_a_inc,
                               double h_half_sector)
    : max_dist{mx_dist}, h_angle_inc{h_a_inc}, h_hsector{h_half_sector} {}
  constexpr LaserScannerParams()
    : LaserScannerParams{15, deg2rad(90), deg2rad(180)} {}

  const double max_dist; // meters
  // "h" is for "horizontal"
  const double h_angle_inc; // radians
  const double h_hsector; // half of horizontal sector (FoW), radians
};

// TODO: give proper name
constexpr static auto to_lsp(double max_dist, double fow_deg, unsigned pts_nm) {
  return LaserScannerParams{max_dist,
      deg2rad(fow_deg / pts_nm), deg2rad(fow_deg / 2.0)};
}

class LaserScanGenerator {
public:
  LaserScanGenerator(LaserScannerParams ls_params = {}) : _lsp{ls_params} {}

  LaserScan2D laser_scan_2D(const GridMap& map, const RobotPose& pose,
                            double occ_threshold = 1) {

    LaserScan2D scan;
    scan.trig_provider = std::make_shared<RawTrigonometryProvider>();
    auto robot_point = pose.point();
    auto robot_coord = map.world_to_cell(robot_point);
    assert(!are_equal(robot_point.x, robot_coord.x * map.scale()) &&
           !are_equal(robot_point.y, robot_coord.y * map.scale()) &&
           "LS Gen: robot at cell boundary is not supported");

    auto hhsector = _lsp.h_hsector; // horiz. half-sector
    for (double a = -hhsector; a <= hhsector; a += _lsp.h_angle_inc) {
      if (2*M_PI <= hhsector + a) { break; }
      auto beam_dir = Point2D{_lsp.max_dist * std::cos(a + pose.theta),
                              _lsp.max_dist * std::sin(a + pose.theta)};
      auto area_ids = map.world_to_cells({robot_point, robot_point + beam_dir});
      for (auto& area_id : area_ids) {
        if (map[area_id] < occ_threshold) { continue; }
        // NB: Beam-goes-through-the-cell-center assumption is not safe,
        //     so use more sophisticated analysis based on intersections.
        auto ray = Ray{robot_point.x, beam_dir.x, robot_point.y, beam_dir.y};
        auto inters = map.world_cell_bounds(area_id).find_intersections(ray);
        if (inters.size() == 1) { // Assume beam touches the cell.
          // NB: Ignores "pierce" case for simplicity,
          //     increase max_dist in this case.
          continue;
        }

        assert(2 == inters.size() && "BUG! LS Gen: obstacle is not pierced");
        auto obst_pnt = Point2D{(inters[0].x + inters[1].x) / 2,
                                (inters[0].y + inters[1].y) / 2};
        auto range = std::sqrt(robot_point.dist_sq(obst_pnt));
        auto scan_point = ScanPoint2D::make_polar(range, a, true);
        scan.points().push_back(scan_point);

        auto added_wp = scan_point.move_origin(pose.x, pose.y, pose.theta);
        auto sp_area_id = map.world_to_cell(added_wp);
        assert(area_id == sp_area_id &&
               "[BUG] Estimated scan point doesn't lie is expected area");
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
