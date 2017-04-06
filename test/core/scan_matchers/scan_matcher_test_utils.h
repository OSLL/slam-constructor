#ifndef SLAM_CTOR_TESTS_SCAN_MATCHER_TEST_UTILS_H_INCLUDED
#define SLAM_CTOR_TESTS_SCAN_MATCHER_TEST_UTILS_H_INCLUDED

#include <limits>
#include <memory>
#include <gtest/gtest.h>

#include "../mock_grid_cell.h"
#include "../../../src/utils/data_generation/map_primitives.h"
#include "../../../src/utils/data_generation/grid_map_patcher.h"
#include "../../../src/utils/data_generation/laser_scan_generator.h"

#include "../../../src/core/scan_matchers/grid_scan_matcher.h"

// TODO: rm code duplication for cost estimators
class DiscrepancySumCostEstimator : public ScanCostEstimator {
public:
  double estimate_scan_cost(const RobotPose &pose,
                            const TransformedLaserScan &scan,
                            const GridMap &map,
                            double min_cost) override {
    auto OCCUPIED_OBSERVATION = AreaOccupancyObservation{
      true, Occupancy{1.0, 1.0}, Point2D{0, 0}, 1.0};
    double cost = 0;
    for (const auto &sp : scan.points) {
      if (!sp.is_occupied) {
        continue;
      }
      auto cell_coord = map.world_to_cell_by_vec(pose.x, pose.y, sp.range,
                                                 pose.theta + sp.angle);
      cost += map[cell_coord].discrepancy(OCCUPIED_OBSERVATION);
      /*
      std::cout << "(" << x_world << ", " << y_world << ") -> "
                << cell_coord << " -> "
                << map[cell_coord].discrepancy(OCCUPIED_OBSERVATION)
                << std::endl;
      */
    }
    return cost;
  }
};

template<typename MapType>
class ScanMatcherTestBase : public ::testing::Test {
protected: // consts
  static constexpr RobotPoseDelta Acceptable_Error = {
    std::numeric_limits<double>::epsilon(),
    std::numeric_limits<double>::epsilon(),
    std::numeric_limits<double>::epsilon()
  };
protected: // methods

  static LaserScannerParams to_lsp(double max_dist, int fow_deg, int pts_nm) {
    return LaserScannerParams{max_dist,
        deg2rad(fow_deg / pts_nm), deg2rad(fow_deg / 2.0)};
  }

  ScanMatcherTestBase(std::shared_ptr<ScanCostEstimator> scan_cost_estimator,
                      int map_w, int map_h, double map_scale,
                      const LaserScannerParams &dflt_lsp)
    : map{std::make_shared<MockGridCell>(), {map_w, map_h, map_scale}}
    , rpose{map.scale() / 2, map.scale() / 2, 0}
    , sce{scan_cost_estimator}
    , default_lsp{dflt_lsp} {}

  virtual void add_primitive_to_map(const TextRasterMapPrimitive &mp,
                                    const DiscretePoint2D &offset,
                                    int w_scale, int h_scale) {
    GridMapPatcher{}.apply_text_raster(map, mp.to_stream(),
                                       offset, w_scale, h_scale);
  }

  virtual GridScanMatcher& scan_matcher() = 0;
  virtual RobotPoseDelta default_acceptable_error() { return Acceptable_Error; }

  virtual bool is_result_noise_acceptable(const TransformedLaserScan &scan,
                                          const RobotPoseDelta &init_noise,
                                          const RobotPoseDelta &result_noise) {
    return sce->estimate_scan_cost(rpose, scan, map) ==
           sce->estimate_scan_cost(rpose + result_noise, scan, map);
  }

  void test_scan_matcher(const LaserScannerParams &lsp,
                         const RobotPoseDelta &noise,
                         const RobotPoseDelta &acc_error) {
    auto scan = LaserScanGenerator{lsp}.generate_2D_laser_scan(map, rpose, 1);
    ASSERT_TRUE(scan.points.size() != 0);

    auto correction = RobotPoseDelta{};
    scan_matcher().process_scan(rpose + noise, scan, map, correction);
    auto result_noise = noise + correction;
    if (is_result_noise_acceptable(scan, noise, result_noise)) {
      return;
    }

    // OR scan cost is the same as actual
    ASSERT_NEAR(0, std::abs(result_noise.x), acc_error.x);
    ASSERT_NEAR(0, std::abs(result_noise.y), acc_error.y);
    ASSERT_NEAR(0, std::abs(result_noise.theta), acc_error.theta);
  }

  void test_scan_matcher(const RobotPoseDelta &noise,
                         const RobotPoseDelta &acc_error) {
    test_scan_matcher(default_lsp, noise, acc_error);
  }

  void test_scan_matcher(const RobotPoseDelta &noise) {
    test_scan_matcher(default_lsp, noise, default_acceptable_error());
  }

protected: // fields
  MapType map;
  RobotPose rpose;
  std::shared_ptr<ScanCostEstimator> sce;
  LaserScannerParams default_lsp;
};

#endif
