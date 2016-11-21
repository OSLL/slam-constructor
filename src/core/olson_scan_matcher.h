#include "sensor_data.h"
#include "state_data.h"
#include "grid_scan_matcher.h"
#include "maps/lazy_layered_grid_map.h"
#include "../tiny_slam/tiny_grid_cell.h"

//for using this scan mathcer decrease the quantity of week bounds
class Olson_scan_matcher : public GridScanMatcher{
private:
  struct Point{
    double x,y;
    Point(double x, double y) {
      this->x = x; this->y = y;
    }
  };
public:
  Olson_scan_matcher(double _sigma = 0.2, double _meters_per_cell = 0.1) :
                     GridScanMatcher(nullptr), sigma(_sigma),
                     meters_per_cell(_meters_per_cell), table(nullptr) {
    radius = meters_per_cell*4.01;
  }

  double process_scan(const RobotPose &init_pose,
                      const TransformedLaserScan &scan,
                      const GridMap &map,
                      RobotPoseDelta &pose_delta) override {
    // if it is a first call of this function;
    if(prev_scan.points.empty()) {
      prev_scan = scan;
      prev_pose = init_pose;
      return 0;
    }

    double max_prob = -1.0;
    double mdx = 0,mdy = 0,mdt = 0;

    double odom_x = std::abs(scan.pose_delta.x);
    double odom_y = std::abs(scan.pose_delta.y);
    double odom_theta = std::abs(scan.pose_delta.theta);

    RobotPose prob_pose(prev_pose.x + odom_x, prev_pose.y + odom_y,
                        prev_pose.theta + odom_theta);
    generate_table(prev_pose, prev_scan);

    for (double dx = -odom_x; dx < odom_x; dx += odom_x/10.0) {
      for (double dy = -odom_y; dy < odom_y; dy += odom_y/10.0) {
        for (double dt = -odom_theta; dt < odom_theta; dt += odom_theta/10.0) {
          double scan_prob = std::exp(esimate_scan_prob(
            scan,
            RobotPose(init_pose.x + dx, init_pose.y + dy, init_pose.theta + dt),
            map));
          double pos_prob = scan_prob * normal_variation(dx, dy, dt,0.5);
          if (pos_prob > max_prob) {
            max_prob = pos_prob;
            mdx = dx;
            mdy = dy;
            mdt = dt;
          }
        }
      }
    }

    pose_delta.x = mdx;
    pose_delta.y = mdy;
    pose_delta.theta = mdt;

    prev_scan = scan;
    prev_pose = init_pose;
    prev_pose += RobotPoseDelta(mdx,mdy,mdt);
    return -max_prob;
  }

  void reset_state() {
    table.reset();
  };

private:
  void blur_probability(Point point) {
    if(table == nullptr) {
      return;
    }

    double step = table->scale();
    for (double i = -radius; i <= radius; i += step) {
      double range_j = std::sqrt(QUAD(radius) - QUAD(i));
      for (double j = -range_j; j <= range_j; j += step) {
        DiscretePoint2D cur_cell = table->world_to_cell(i+point.x, j+point.y);
        double value = table->operator[](cur_cell);
        value = value + normal_variation(i/radius,j/radius,0.0, sigma)/2.0 -
                      value*normal_variation(i/radius,j/radius,0.0, sigma)/2.0;
        value = value > 1.0 ? 1.0 : value;
        table->operator[](cur_cell) = Occupancy(value, 1.0);
      }
    }
  }

  std::vector<Point> scan_to_cartezian(const RobotPose& pose,
                                        const TransformedLaserScan& scan,
                                        double &x_min, double &x_max,
                                        double &y_min, double &y_max) {
    std::vector<Point> result;
    x_min = 10000;
    y_min = 10000;
    x_max = -10000;
    y_max = -10000;
    int i = 0;
    for (auto point : scan.points) {
      i++;
      if(!point.is_occupied) {
        continue;
      }
      if(i%20) continue;
      double x_world = pose.x + point.range * std::cos(point.angle+pose.theta);
      x_max = x_max < x_world ? x_world : x_max;
      x_min = x_world < x_min ? x_world : x_min;

      double y_world = pose.y + point.range * std::sin(point.angle+pose.theta);
      y_max = y_max < y_world ? y_world : y_max;
      y_min = y_world < y_min ? y_world : y_min;

      result.push_back(Point(x_world, y_world));
    }
    return result;
  }

  void generate_table(const RobotPose& pose,
                      const TransformedLaserScan& scan) {
    double x_min, y_min, x_max, y_max;
    std::vector<Point> current_cartezian_scan =
                    scan_to_cartezian(pose, scan, x_min, x_max, y_min, y_max);

    //TODO put this in a constructor for table
    //table_params.meters_per_cell = meters_per_cell;
    //table_params.height = y_max - y_min + 2.0 * radius;
    //table_params.width  = x_max - x_min + 2.0 * radius;

    std::shared_ptr<BaseTinyCell> cell_prototype(new BaseTinyCell);
    table = std::shared_ptr<LazyLayeredGridMap>(
                                       new LazyLayeredGridMap(cell_prototype));

    for (auto point : current_cartezian_scan) {
      blur_probability(point);
    }
  }

  double esimate_scan_prob(const TransformedLaserScan& scan,
                           const RobotPose &pose, const GridMap& map){
    int i =0;
    double result = 0;
    for (auto point : scan.points) {
      i++;
      if(!point.is_occupied) {
        continue;
      }
      if(i%20) continue;

      double x_world = pose.x + point.range*std::cos(point.angle + pose.theta);
      double y_world = pose.y + point.range*std::sin(point.angle + pose.theta);

      //change here "map" to "table" for using comparison with prev scan
      DiscretePoint2D cell_coord = map.world_to_cell(x_world, y_world);
      double point_prob = map.operator[](cell_coord);
      result += std::log(point_prob);
    }
    result /= static_cast<double>(i);
    return result;
  }

  double radius;
  const double sigma;
  double meters_per_cell;
  std::shared_ptr<LazyLayeredGridMap> table;
  TransformedLaserScan prev_scan;
  RobotPose prev_pose;
};

