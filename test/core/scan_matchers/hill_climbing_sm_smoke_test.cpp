#include <gtest/gtest.h>

#include <limits>

#include "../../../src/core/scan_matchers/hill_climbing_scan_matcher.h"

#include "../../../src/core/maps/plain_grid_map.h"
#include "../../../src/utils/data_generation/grid_map_patcher.h"
#include "../../../src/utils/data_generation/laser_scan_generator.h"

// NB: the test suit checks base ability to find a correction.
//     More sophisticated testing (e.g. an arbitrary noise cases,
//     high res map/scanner, map state, etc.) is supposed to be implemented
//     with a separate test suite.

const std::string Cecum_Corridor_Map_Patch =
  "+-------------+\n"
  "|             |\n"
  "|             |\n"
  "|             |\n"
  "|             |\n"
  "|             |\n"
  "|             |\n"
  "|             |\n"
  "|             |\n"
  "|             |\n"
  "|             |\n"
  "|             |\n"
  "|             |\n";
constexpr int Cecum_Patch_W = 15, Cecum_Patch_H = 13;

constexpr int Cecum_Free_X_Start = 1, Cecum_Free_Y_Start = -1;
constexpr int Cecum_Free_W = 13, Cecum_Free_H = 12;

class TestGridCell : public GridCell {
public:
  TestGridCell() : GridCell{Occupancy{0, 0}} {}
  std::unique_ptr<GridCell> clone() const override {
    return std::make_unique<TestGridCell>(*this);
  }

  double discrepancy(const AreaOccupancyObservation &aoo) const override {
    return std::abs(_occupancy - aoo.occupancy);
  }
};

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
    int occ_pts_nm = 0;
    for (const auto &sp : scan.points) {
      if (!sp.is_occupied) {
        continue;
      }
      double x_world = pose.x + sp.range * std::cos(sp.angle + pose.theta);
      double y_world = pose.y + sp.range * std::sin(sp.angle + pose.theta);

      DiscretePoint2D cell_coord = map.world_to_cell(x_world, y_world);
      occ_pts_nm += 1;
      cost += map[cell_coord].discrepancy(OCCUPIED_OBSERVATION);
    }
    return occ_pts_nm == 0 ? 0 : 1 - cost / occ_pts_nm;
  }
};

class HillClimbingScanMatcherTest : public ::testing::Test {
protected: // methods
  HillClimbingScanMatcherTest()
    : map{std::make_shared<TestGridCell>(),
          {Map_Width, Map_Height, Map_Scale}}
    , rpose{map.scale() / 2, map.scale() / 2, 0} // middle of a cell
    , hcsm{std::make_shared<DiscrepancySumCostEstimator>(),
           Max_SM_Shirnks_Nm, Init_Linear_Delta, Init_Angular_Delta} {}
protected: // consts
  static constexpr int Map_Width = 100;
  static constexpr int Map_Height = 100;
  static constexpr double Map_Scale = 0.1;

  static constexpr int Patch_Scale = 1;

  static constexpr double Laser_Scan_Max_Dist = 15;
  static constexpr int Laser_Scan_FoW = 270;
  static constexpr int Points_Per_Scan = 10;

  static constexpr int Max_SM_Shirnks_Nm = 10;
  static constexpr double Init_Linear_Delta = 0.1;
  static constexpr double Init_Angular_Delta = deg2rad(30);

protected: // fields
  void prepare_map_and_robot_pose(const RobotPoseDelta &rpd,
                                  int scale = Patch_Scale) {
    patch_map_with_cecum(scale);
    rpose += rpd;
  }

  void patch_map_with_cecum(int scale = Patch_Scale) {
    auto gm_patcher = GridMapPatcher{};
    std::stringstream raster{Cecum_Corridor_Map_Patch};
    gm_patcher.apply_text_raster(map, raster, {}, scale, scale);
  }

  RobotPoseDelta middle_of_cecum_entrance_offset(double th_deg) {
    return {(Cecum_Patch_W * Patch_Scale / 2) * map.scale(),
            (-Cecum_Patch_H * Patch_Scale + 1) * map.scale(),
            deg2rad(90)};
  }

  void check_scan_matcher(const RobotPoseDelta &init_robot_offset,
                         const RobotPoseDelta &noise,
                         const RobotPoseDelta &acc_error =
                           {std::numeric_limits<double>::epsilon(),
                            std::numeric_limits<double>::epsilon(),
                            std::numeric_limits<double>::epsilon()},
                         const LaserScannerParams &lsp =
                           {Laser_Scan_Max_Dist,
                            deg2rad(Laser_Scan_FoW / Points_Per_Scan),
                            deg2rad(Laser_Scan_FoW / 2.0)},
                         int raster_scale = Patch_Scale) {
    prepare_map_and_robot_pose(init_robot_offset, raster_scale);
    auto scan = LaserScanGenerator{lsp}.generate_2D_laser_scan(map, rpose, 1);
    ASSERT_TRUE(scan.points.size() != 0);

    auto correction = RobotPoseDelta{};
    hcsm.process_scan(rpose + noise, scan, map, correction);

    auto result_noise = noise + correction;
    ASSERT_NEAR(0, result_noise.x, acc_error.x);
    ASSERT_NEAR(0, result_noise.y, acc_error.y);
    ASSERT_NEAR(0, result_noise.theta, acc_error.theta);
  }

protected: // fields
  UnboundedPlainGridMap map;
  RobotPose rpose;
  HillClimbingScanMatcher hcsm;
};

//------------------------------------------------------------------------------
// Tests

TEST_F(HillClimbingScanMatcherTest, cecumNoPoseNoise) {
  check_scan_matcher(middle_of_cecum_entrance_offset(deg2rad(90)),
                     RobotPoseDelta{0, 0, 0});
}

TEST_F(HillClimbingScanMatcherTest, cecumLinStepXLeftDrift) {
  check_scan_matcher(middle_of_cecum_entrance_offset(deg2rad(90)),
                     RobotPoseDelta{-Init_Linear_Delta, 0, 0});
}

TEST_F(HillClimbingScanMatcherTest, cecumLinStepXRightDrift) {
  check_scan_matcher(middle_of_cecum_entrance_offset(deg2rad(90)),
                     RobotPoseDelta{Init_Linear_Delta, 0, 0});
}

TEST_F(HillClimbingScanMatcherTest, cecumLinStepYUpDrift) {
  check_scan_matcher(middle_of_cecum_entrance_offset(deg2rad(90)),
                     RobotPoseDelta{0, -Init_Linear_Delta, 0});
}

TEST_F(HillClimbingScanMatcherTest, cecumLinStepYDownDrift) {
  check_scan_matcher(middle_of_cecum_entrance_offset(deg2rad(90)),
                     RobotPoseDelta{0, Init_Linear_Delta, 0});
}

TEST_F(HillClimbingScanMatcherTest, cecumAngStepThetaCcwDrift) {
  check_scan_matcher(middle_of_cecum_entrance_offset(deg2rad(90)),
                     RobotPoseDelta{0, 0, Init_Angular_Delta});
}

TEST_F(HillClimbingScanMatcherTest, cecumAngStepThetaCwDrift) {
  check_scan_matcher(middle_of_cecum_entrance_offset(deg2rad(90)),
                     RobotPoseDelta{0, 0, -Init_Angular_Delta});
}

/* FIXME: Doesn't work!
TEST_F(HillClimbingScanMatcherTest, cecumComboStepsDrift) {
  check_scan_matcher(middle_of_cecum_entrance_offset(deg2rad(90)),
                     RobotPoseDelta{Init_Linear_Delta, -Init_Linear_Delta,
                                    Init_Angular_Delta});
}
*/


int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
