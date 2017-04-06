#include <gtest/gtest.h>

#include <limits>

#include "../mock_grid_cell.h"

#include "../../../src/core/scan_matchers/many_to_many_multires_scan_matcher.h"

#include "../../../src/core/maps/zoomable_grid_map.h"
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
      // TODO: move to map (e.g. coord by direction)
      double x_world = pose.x + sp.range * std::cos(sp.angle + pose.theta);
      double y_world = pose.y + sp.range * std::sin(sp.angle + pose.theta);

      DiscretePoint2D cell_coord = map.world_to_cell(x_world, y_world);
      /*
      std::cout << "(" << x_world << ", " << y_world << ") -> "
                << cell_coord << " -> "
                << map[cell_coord].discrepancy(OCCUPIED_OBSERVATION)
                << std::endl;
      */
      occ_pts_nm += 1;
      cost += map[cell_coord].discrepancy(OCCUPIED_OBSERVATION);
    }
    return cost;
  }
};

class M3RScanMatcherTest : public ::testing::Test {
protected: // methods
  M3RScanMatcherTest()
    : map{std::make_shared<MockGridCell>(),
          {Map_Width, Map_Height, Map_Scale}}
    , rpose{map.scale() / 2, map.scale() / 2, 0} // middle of a cell
    , sce{std::make_shared<DiscrepancySumCostEstimator>()}
    , m3rsm{sce, SM_Ang_Step, SM_Ang_Range, SM_Transl_Range, SM_Transl_Range} {}
protected: // consts
  // TODO: calc with respect to raster size/scale
  static constexpr int Map_Width = 200;
  static constexpr int Map_Height = 200;
  static constexpr double Map_Scale = 0.1;

  static constexpr int Patch_Scale = 1;

  static constexpr double Laser_Scan_Max_Dist = 15;
  static constexpr int Laser_Scan_FoW = 270;
  static constexpr int Points_Per_Scan = 10;

  static constexpr double SM_Ang_Step = deg2rad(0.5);
  static constexpr double SM_Ang_Range = deg2rad(10);
  static constexpr double SM_Transl_Range = 1; // meter

protected: // fields
  void prepare_map_and_robot_pose(const RobotPoseDelta &rpd,
                                  int scale = Patch_Scale) {
    //TODO: fix top level zoom map - the map doesn't cover entire space
    //WA: work on a grid part with positive coords
    auto offset = DiscretePoint2D{int(Cecum_Patch_W * scale / Map_Scale + 0.5),
                                  int(Cecum_Patch_H * scale / Map_Scale + 0.5)};
    patch_map_with_cecum(scale);
    rpose += RobotPoseDelta{offset.x * map.scale(), offset.y * map.scale(), 0};
    rpose += rpd;
  }

  void patch_map_with_cecum(int scale = Patch_Scale) {
    auto gm_patcher = GridMapPatcher{};
    std::stringstream raster{Cecum_Corridor_Map_Patch};
    //TODO: fix top level zoom map - the map doesn't cover entire space
    //WA: work on a grid part with positive coords
    auto offset = DiscretePoint2D{int(Cecum_Patch_W * scale / Map_Scale + 0.5),
                                  int(Cecum_Patch_H * scale / Map_Scale + 0.5)};
    gm_patcher.apply_text_raster(map, raster, offset, scale, scale);
  }

  RobotPoseDelta middle_of_cecum_entrance_offset(double th_deg) {
    return {(Cecum_Patch_W * Patch_Scale / 2) * map.scale(),
            (-Cecum_Patch_H * Patch_Scale + 1) * map.scale(),
            deg2rad(th_deg)};
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
    map.set_zoom_level(0);
    prepare_map_and_robot_pose(init_robot_offset, raster_scale);
    auto scan = LaserScanGenerator{lsp}.generate_2D_laser_scan(map, rpose, 1);
    ASSERT_TRUE(scan.points.size() != 0);

    auto correction = RobotPoseDelta{};
    m3rsm.process_scan(rpose + noise, scan, map, correction);
    auto result_noise = noise + correction;
    //std::cout << "Correction: " << correction << std::endl;
    //std::cout << "Result Nse: " << result_noise << std::endl;
    map.set_zoom_level(0);
    if (sce->estimate_scan_cost(rpose, scan, map) ==
        sce->estimate_scan_cost(rpose + result_noise, scan, map)) {
      // corrected noise is equivalent to the noise absense
      // from the cost function point of view
      return;
    }

    std::cout << sce->estimate_scan_cost(rpose, scan, map) << " VS "
              << sce->estimate_scan_cost(rpose + result_noise, scan, map)
              << std::endl;

    // OR scan cost is the same as actual
    ASSERT_NEAR(0, std::abs(result_noise.x), acc_error.x);
    ASSERT_NEAR(0, std::abs(result_noise.y), acc_error.y);
    ASSERT_NEAR(0, std::abs(result_noise.theta), acc_error.theta);
  }

protected: // fields
  ZoomableGridMap<UnboundedPlainGridMap> map;
  RobotPose rpose;
  std::shared_ptr<ScanCostEstimator> sce;
  ManyToManyMultiResoultionScanMatcher m3rsm;
};

//------------------------------------------------------------------------------
// Tests

TEST_F(M3RScanMatcherTest, cecumNoPoseNoise) {
  check_scan_matcher(middle_of_cecum_entrance_offset(deg2rad(90)),
                     RobotPoseDelta{0, 0, 0},
                     RobotPoseDelta{map.scale() / 2, map.scale() / 2,
                                    SM_Ang_Step});
}


TEST_F(M3RScanMatcherTest, cecumLinStepXLeftDrift) {
  check_scan_matcher(middle_of_cecum_entrance_offset(deg2rad(90)),
                     RobotPoseDelta{-SM_Transl_Range / 2, 0, 0},
                     RobotPoseDelta{map.scale() / 2, map.scale() / 2,
                                    SM_Ang_Step});
}

TEST_F(M3RScanMatcherTest, cecumLinStepXRightDrift) {
  check_scan_matcher(middle_of_cecum_entrance_offset(deg2rad(90)),
                     RobotPoseDelta{SM_Transl_Range / 2, 0, 0},
                     RobotPoseDelta{map.scale() / 2, map.scale() / 2,
                                    SM_Ang_Step});
}

TEST_F(M3RScanMatcherTest, cecumLinStepYUpDrift) {
  check_scan_matcher(middle_of_cecum_entrance_offset(deg2rad(90)),
                     RobotPoseDelta{0, -SM_Transl_Range / 2, 0},
                     RobotPoseDelta{map.scale() / 2, map.scale() / 2,
                                    SM_Ang_Step});
}

TEST_F(M3RScanMatcherTest, cecumLinStepYDownDrift) {
  check_scan_matcher(middle_of_cecum_entrance_offset(deg2rad(90)),
                     RobotPoseDelta{0, SM_Transl_Range / 2, 0},
                     RobotPoseDelta{map.scale() / 2, map.scale() / 2,
                                    SM_Ang_Step});
}

TEST_F(M3RScanMatcherTest, cecumAngStepThetaCcwDrift) {
  check_scan_matcher(middle_of_cecum_entrance_offset(deg2rad(90)),
                     RobotPoseDelta{0, 0, SM_Ang_Range / 2},
                     RobotPoseDelta{map.scale() / 2, map.scale() / 2,
                                    SM_Ang_Step});
}

TEST_F(M3RScanMatcherTest, cecumAngStepThetaCwDrift) {
  check_scan_matcher(middle_of_cecum_entrance_offset(deg2rad(90)),
                     RobotPoseDelta{0, 0, -SM_Ang_Range / 2},
                     RobotPoseDelta{map.scale() / 2, map.scale() / 2,
                                    SM_Ang_Step});
}

TEST_F(M3RScanMatcherTest, cecumComboStepsDrift) {
  check_scan_matcher(middle_of_cecum_entrance_offset(deg2rad(90)),
                     RobotPoseDelta{-SM_Transl_Range / 2, SM_Transl_Range / 2,
                                    -SM_Ang_Range / 2},
                     RobotPoseDelta{map.scale() / 2, map.scale() / 2,
                                    SM_Ang_Step});
}

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
