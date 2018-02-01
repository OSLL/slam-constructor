#include <gtest/gtest.h>

#include "../../core/mock_grid_cell.h"

#include "../../../src/core/math_utils.h"
#include "../../../src/core/states/robot_pose.h"
#include "../../../src/core/maps/plain_grid_map.h"
#include "../../../src/utils/data_generation/map_primitives.h"
#include "../../../src/utils/data_generation/grid_map_patcher.h"
#include "../../../src/utils/data_generation/laser_scan_generator.h"

#include "../../../src/utils/console_view.h"

class LaserScanGeneratorTest : public ::testing::Test {
protected: // methods
  LaserScanGeneratorTest()
    : map{std::make_shared<MockGridCell>(),
          GridMapParams{Map_Width, Map_Height, Map_Scale}}
    , rpose{map.scale() / 2, map.scale() / 2, 0} /* middle of a cell */ {}
protected:
  using ScanPoints = std::vector<ScanPoint2D>;
protected: // consts
  static constexpr int Map_Width = 100;
  static constexpr int Map_Height = 100;
  static constexpr double Map_Scale = 1;

  static constexpr int Patch_Scale = 10;
protected: // fields
  void prepare_map_and_robot_pose(const MapPrimitive &mp,
                                  const RobotPoseDelta &rpd,
                                  int scale = Patch_Scale) {
    patch_map(mp, scale);
    rpose += rpd;
  }

  void patch_map(const MapPrimitive &mp, int scale) {
    auto gm_patcher = GridMapPatcher{};
    gm_patcher.apply_text_raster(map, mp.to_stream(), {}, scale, scale);
  }

  void check_scan_points(const ScanPoints &expected, const ScanPoints &actual,
                         const ScanPoint2D &sp_err = {Map_Scale / 2, 0.001}) {
    ASSERT_EQ(expected.size(), actual.size());
    for (std::size_t sp_i = 0; sp_i < expected.size(); ++sp_i) {
      check_scan_point(expected[sp_i], actual[sp_i], sp_err);
      auto expected_occ = actual[sp_i].is_occupied() ? 1.0 : 0;
      auto wp = actual[sp_i].move_origin(rpose.x, rpose.y, rpose.theta);
      ASSERT_NEAR(expected_occ, map[map.world_to_cell(wp)], 0.01);
    }
  }

  void check_scan_point(const ScanPoint2D &expected, const ScanPoint2D &actual,
                        const ScanPoint2D &abs_err) {
    ASSERT_NEAR(expected.angle(), actual.angle(), abs_err.angle());

    // scale absolute range error according to relative angle
    auto range_err = abs_err.range() / std::cos(rpose.theta + actual.angle());
    ASSERT_NEAR(expected.range(), actual.range(), std::abs(range_err));
  }

  void dbg_show_map_near_robot(int l, int r, int u, int d) {
    auto scale = Patch_Scale * map.scale();
    show_grid_map(map, {rpose.x, rpose.y},
                  l * scale, r * scale, u * scale, d * scale);
  }

protected: // fields
  UnboundedPlainGridMap map;
  RobotPose rpose;
  LaserScanGenerator lsg{LaserScannerParams{150, deg2rad(90), deg2rad(180)}};
};

//------------------------------------------------------------------------------
// Degenerate cases

TEST_F(LaserScanGeneratorTest, emptyMap4beams) {
  auto scan = lsg.laser_scan_2D(map, rpose, 1);

  const auto Expected_SPs = ScanPoints{};
  check_scan_points(Expected_SPs, scan.points());
}

TEST_F(LaserScanGeneratorTest, insideObstacle4beams) {
  auto occ_obs = AreaOccupancyObservation{true, Occupancy{1.0, 1.0},
                                          Point2D{rpose.x, rpose.y}, 1};
  map.update(map.world_to_cell(rpose.x, rpose.y), occ_obs);

  auto scan = lsg.laser_scan_2D(map, rpose, 1);
  const auto Expected_SPs = ScanPoints {{0, deg2rad(-180)}, {0, deg2rad(-90)},
                                        {0, deg2rad(0)}, {0, deg2rad(90)}};

  check_scan_points(Expected_SPs, scan.points());
}

//------------------------------------------------------------------------------
// Perpendicular wall facing

TEST_F(LaserScanGeneratorTest, leftOfCecumEntranceFacingRight4beams) {
  const auto Top_Bnd_Pos = CecumTextRasterMapPrimitive::BoundPosition::Top;
  auto cecum_mp = CecumTextRasterMapPrimitive{7, 4, Top_Bnd_Pos};
  auto mp_free_space = cecum_mp.free_space()[0];

  auto pose_delta = RobotPoseDelta{
    (mp_free_space.left() * Patch_Scale) * map.scale(),
    (-cecum_mp.height() * Patch_Scale + 1) * map.scale(),
    deg2rad(0)
  };
  prepare_map_and_robot_pose(cecum_mp, pose_delta, Patch_Scale);
  auto scan = lsg.laser_scan_2D(map, rpose, 1);

  const double scale = map.scale() * Patch_Scale;
  const auto Expected_SPs = ScanPoints{
    {map.scale(), deg2rad(-180)},
    {mp_free_space.hside_len() * scale, deg2rad(0)},
    {mp_free_space.vside_len() * scale, deg2rad(90)}};
  check_scan_points(Expected_SPs, scan.points());
}

TEST_F(LaserScanGeneratorTest, rightOfCecumEntranceFacingTop4beams) {
  const auto Top_Bnd_Pos = CecumTextRasterMapPrimitive::BoundPosition::Top;
  auto cecum_mp = CecumTextRasterMapPrimitive{7, 4, Top_Bnd_Pos};
  auto mp_free_space = cecum_mp.free_space()[0];

  auto pose_delta = RobotPoseDelta{
    (mp_free_space.right() * Patch_Scale - 1) * map.scale(),
    (-cecum_mp.height() * Patch_Scale + 1) * map.scale(),
    deg2rad(90)
  };
  prepare_map_and_robot_pose(cecum_mp, pose_delta, Patch_Scale);
  const double scale = map.scale() * Patch_Scale;
  auto scan = lsg.laser_scan_2D(map, rpose, 1);

  const auto Expected_SPs = ScanPoints{
    {map.scale(), deg2rad(-90)},
    {mp_free_space.vside_len() * scale, deg2rad(0)},
    {mp_free_space.hside_len() * scale, deg2rad(90)}};
  check_scan_points(Expected_SPs, scan.points());
}

TEST_F(LaserScanGeneratorTest, rightOfCecumEndFacingLeft4beams) {
  const auto Top_Bnd_Pos = CecumTextRasterMapPrimitive::BoundPosition::Top;
  auto cecum_mp = CecumTextRasterMapPrimitive{7, 4, Top_Bnd_Pos};
  auto mp_free_space = cecum_mp.free_space()[0];

  auto pose_delta = RobotPoseDelta{
    (mp_free_space.right() * Patch_Scale - 1) * map.scale(),
    ((mp_free_space.top() - 1) * Patch_Scale) * map.scale(),
    deg2rad(180)
  };
  prepare_map_and_robot_pose(cecum_mp, pose_delta, Patch_Scale);
  auto scan = lsg.laser_scan_2D(map, rpose, 1);

  const auto Expected_SPs = ScanPoints{
    {map.scale(), deg2rad(-180)},
    {map.scale(), deg2rad(-90)},
    {mp_free_space.hside_len() * Patch_Scale * map.scale(), deg2rad(0)}};
  check_scan_points(Expected_SPs, scan.points());
}

TEST_F(LaserScanGeneratorTest, leftOfCecumEndFacingDown4beams) {
  const auto Top_Bnd_Pos = CecumTextRasterMapPrimitive::BoundPosition::Top;
  auto cecum_mp = CecumTextRasterMapPrimitive{7, 4, Top_Bnd_Pos};
  auto mp_free_space = cecum_mp.free_space()[0];

  auto pose_delta = RobotPoseDelta{
    (mp_free_space.left() * Patch_Scale) * map.scale(),
    ((mp_free_space.top() - 1) * Patch_Scale) * map.scale(),
    deg2rad(270)
  };
  prepare_map_and_robot_pose(cecum_mp, pose_delta, Patch_Scale);
  auto scan = lsg.laser_scan_2D(map, rpose, 1);

  const auto Expected_SPs = ScanPoints{
    {map.scale(), deg2rad(-180)},
    {map.scale(), deg2rad(-90)},
    {mp_free_space.hside_len() * Patch_Scale * map.scale(), deg2rad(90)}};
  check_scan_points(Expected_SPs, scan.points());
}

TEST_F(LaserScanGeneratorTest, middleOfCecumEntranceFacingIn4beams) {
  const auto Top_Bnd_Pos = CecumTextRasterMapPrimitive::BoundPosition::Top;
  auto cecum_mp = CecumTextRasterMapPrimitive{7, 4, Top_Bnd_Pos};
  auto mp_free_space = cecum_mp.free_space()[0];

  auto pose_delta = RobotPoseDelta{
    (cecum_mp.width() * Patch_Scale / 2) * map.scale(),
    (-cecum_mp.height() * Patch_Scale + 1) * map.scale(),
    deg2rad(90)};
  prepare_map_and_robot_pose(cecum_mp, pose_delta, Patch_Scale);
  auto scan = lsg.laser_scan_2D(map, rpose, 1);

  // extra 0.5*map.scale() is for robot offset inside the cell
  const auto Expected_SPs = ScanPoints{
    {((Patch_Scale * mp_free_space.hside_len() + 1) / 2) * map.scale(),
     deg2rad(-90)},
    {Patch_Scale * mp_free_space.vside_len() * map.scale(),
     deg2rad(0)},
    {(Patch_Scale * mp_free_space.hside_len() / 2 + 1) * map.scale(),
     deg2rad(90)}
  };

  check_scan_points(Expected_SPs, scan.points());
}

//------------------------------------------------------------------------------
// Misc fall facing

TEST_F(LaserScanGeneratorTest, rightOfCecumEndFacingLeftBot45deg4beams) {
  const auto Top_Bnd_Pos = CecumTextRasterMapPrimitive::BoundPosition::Top;
  auto cecum_mp = CecumTextRasterMapPrimitive{4, 7, Top_Bnd_Pos};
  auto mp_fspc = cecum_mp.free_space()[0];

  auto pose_delta = RobotPoseDelta{
    (mp_fspc.right() * Patch_Scale - 1) * map.scale(),
    ((mp_fspc.top() - 1) * Patch_Scale) * map.scale(),
    deg2rad(225)
  };
  prepare_map_and_robot_pose(cecum_mp, pose_delta, Patch_Scale);
  auto scan = lsg.laser_scan_2D(map, rpose, 1);

  const auto Expected_SPs = ScanPoints{
    {map.scale(), deg2rad(-180)},
    {map.scale(), deg2rad(-90)},
    {mp_fspc.hside_len() * map.scale() * Patch_Scale / std::cos(deg2rad(45)),
     deg2rad(0)},
    {map.scale(), deg2rad(90)}};
  check_scan_points(Expected_SPs, scan.points());
}

TEST_F(LaserScanGeneratorTest, leftOfCecumEndFacingRightBot30deg4beams) {
  const auto Top_Bnd_Pos = CecumTextRasterMapPrimitive::BoundPosition::Top;
  auto cecum_mp = CecumTextRasterMapPrimitive{3, 4, Top_Bnd_Pos};
  auto mp_fspc = cecum_mp.free_space()[0];

  auto pose_delta = RobotPoseDelta{
    (mp_fspc.left() * Patch_Scale) * map.scale(),
    ((mp_fspc.top() - 1) * Patch_Scale) * map.scale(),
    deg2rad(-30)
  };
  prepare_map_and_robot_pose(cecum_mp, pose_delta, Patch_Scale);
  auto scan = lsg.laser_scan_2D(map, rpose, 1);

  const auto Expected_SPs = ScanPoints{
    {map.scale(), deg2rad(-180)},
    {map.scale(), deg2rad(-90)},
    {mp_fspc.hside_len() * map.scale() * Patch_Scale / std::cos(deg2rad(30)),
     deg2rad(0)},
    {map.scale(), deg2rad(90)}};
  check_scan_points(Expected_SPs, scan.points());
}

TEST_F(LaserScanGeneratorTest, cecumCenterFacingDown8beams240FoW) {
  const auto Top_Bnd_Pos = CecumTextRasterMapPrimitive::BoundPosition::Top;
  // TODO: investigate expected scan points generation for the "4, 9" case
  auto cecum_mp = CecumTextRasterMapPrimitive{3, 9, Top_Bnd_Pos};
  auto mp_fspc = cecum_mp.free_space()[0];

  auto pose_delta = RobotPoseDelta{
    (cecum_mp.width() * Patch_Scale) * map.scale() / 2,
    ((mp_fspc.top() - 1) - mp_fspc.vside_len() / 2) * Patch_Scale * map.scale(),
    deg2rad(-90)
  };
  prepare_map_and_robot_pose(cecum_mp, pose_delta, Patch_Scale);
  auto lsgen = LaserScanGenerator{LaserScannerParams{30, deg2rad(270.0 / 8),
                                                         deg2rad(270.0 / 2)}};
  auto scan = lsgen.laser_scan_2D(map, rpose, 1);

  constexpr double A_Step = 270.0 / 8;
  double cecum_free_w = mp_fspc.hside_len();
  double left_w =  ((Patch_Scale * cecum_free_w + 1) / 2) * map.scale();
  double right_w = (Patch_Scale * cecum_free_w / 2) * map.scale();
  const auto Expected_SPs = ScanPoints{
    {left_w / std::cos(deg2rad(45 - 0 * A_Step)), deg2rad(-135 + 0 * A_Step)},
    {left_w / std::cos(deg2rad(45 - 1 * A_Step)), deg2rad(-135 + 1 * A_Step)},
    {left_w / std::cos(deg2rad(45 - 2 * A_Step)), deg2rad(-135 + 2 * A_Step)},
    {left_w / std::cos(deg2rad(45 - 3 * A_Step)), deg2rad(-135 + 3 * A_Step)},
    // 0.0 angle
    {right_w / std::cos(deg2rad(45 - 3 * A_Step)), deg2rad(-135 + 5 * A_Step)},
    {right_w / std::cos(deg2rad(45 - 2 * A_Step)), deg2rad(-135 + 6 * A_Step)},
    {right_w / std::cos(deg2rad(45 - 1 * A_Step)), deg2rad(-135 + 7 * A_Step)},
    {right_w / std::cos(deg2rad(45 - 0 * A_Step)), deg2rad(-135 + 8 * A_Step)}};
  check_scan_points(Expected_SPs, scan.points());
}

//---//

TEST_F(LaserScanGeneratorTest, trigonometricCacheCorrectness) {
  const auto Top_Bnd_Pos = CecumTextRasterMapPrimitive::BoundPosition::Top;
  auto cecum_mp = CecumTextRasterMapPrimitive{10, 10, Top_Bnd_Pos};
  auto mp_fspc = cecum_mp.free_space()[0];

  auto pose_delta = RobotPoseDelta{
    (cecum_mp.width() * Patch_Scale) * map.scale() / 2,
    ((mp_fspc.top() - 1) - mp_fspc.vside_len() / 2) * Patch_Scale * map.scale(),
    deg2rad(90)
  };
  prepare_map_and_robot_pose(cecum_mp, pose_delta, Patch_Scale);

  const int LS_Points_Nm = 4000;
  auto lsgen = LaserScanGenerator{
    LaserScannerParams{30, deg2rad(270.0 / LS_Points_Nm), deg2rad(270.0 / 2)}};
  auto scan = lsgen.laser_scan_2D(map, rpose, 1);

  const double D_Angle = deg2rad(1.3);
  scan.trig_provider->set_base_angle(D_Angle);
  for (auto &sp : scan.points()) {
    ASSERT_NEAR(std::cos(sp.angle() + D_Angle),
                scan.trig_provider->cos(sp.angle()), 0.001);
    ASSERT_NEAR(std::sin(sp.angle() + D_Angle),
                scan.trig_provider->sin(sp.angle()), 0.001);
  }
}

//------------------------------------------------------------------------------

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
