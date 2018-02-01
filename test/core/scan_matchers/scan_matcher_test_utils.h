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
#include "../../../src/core/scan_matchers/weighted_mean_point_probability_spe.h"


template<typename MapType>
class ScanMatcherTestBase : public ::testing::Test {
protected: // type aliases
  using DefaultSPE = WeightedMeanPointProbabilitySPE;
protected: // consts
  static constexpr RobotPoseDelta Acceptable_Error = {
    std::numeric_limits<double>::epsilon(),
    std::numeric_limits<double>::epsilon(),
    std::numeric_limits<double>::epsilon()
  };
protected: // methods

  ScanMatcherTestBase(std::shared_ptr<ScanProbabilityEstimator> prob_est,
                      int map_w, int map_h, double map_scale,
                      const LaserScannerParams &dflt_lsp)
    : map{std::make_shared<MockGridCell>(), {map_w, map_h, map_scale}}
    , rpose{map.scale() / 2, map.scale() / 2, 0}
    , spe{prob_est}
    , default_lsp{dflt_lsp} {}

  virtual void add_primitive_to_map(const TextRasterMapPrimitive &mp,
                                    const DiscretePoint2D &offset,
                                    int w_scale, int h_scale) {
    GridMapPatcher{}.apply_text_raster(map, mp.to_stream(),
                                       offset, w_scale, h_scale);
  }

  virtual GridScanMatcher& scan_matcher() = 0;
  virtual RobotPoseDelta default_acceptable_error() { return Acceptable_Error; }

  virtual bool is_result_noise_acceptable(const TransformedLaserScan &raw_scan,
                                          const RobotPoseDelta &init_noise,
                                          const RobotPoseDelta &noise) {
    auto scan = spe->filter_scan(raw_scan.scan, rpose, map);
    auto no_noise_prob = spe->estimate_scan_probability(scan, rpose, map);
    auto noise_prob = spe->estimate_scan_probability(scan, rpose + noise, map);
    //std::cout << no_noise_prob << " vs " << noise_prob << std::endl;
    return are_equal(no_noise_prob, noise_prob);
  }

  void test_scan_matcher(const LaserScannerParams &lsp,
                         const RobotPoseDelta &noise,
                         const RobotPoseDelta &acc_error) {
    auto tr_scan = TransformedLaserScan{};
    tr_scan.pose_delta = RobotPoseDelta{rpose.x, rpose.y, rpose.theta};
    tr_scan.scan = LaserScanGenerator{lsp}.laser_scan_2D(map, rpose, 1);
    tr_scan.quality = 1.0;

    ASSERT_TRUE(tr_scan.scan.points().size() != 0);

    auto correction = RobotPoseDelta{};
    auto original_map_scale = map.scale();
    scan_matcher().process_scan(tr_scan, rpose + noise,  map, correction);
    assert(map.scale() == original_map_scale);
    auto result_noise = noise + correction;
    if (is_result_noise_acceptable(tr_scan, noise, result_noise)) {
      return;
    }

    // OR scan probability is the same as actual
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
  std::shared_ptr<ScanProbabilityEstimator> spe;
  LaserScannerParams default_lsp;
};

#endif
