#include <gtest/gtest.h>

#include <functional>

#include "../../../src/core/states/sensor_data.h"

#include "../mock_grid_cell.h"
#include "../../../src/core/maps/plain_grid_map.h"
#include "../../../src/utils/data_generation/map_primitives.h"
#include "../../../src/utils/data_generation/grid_map_patcher.h"
#include "../../../src/utils/data_generation/laser_scan_generator.h"


//----------------------------------------------------------------------------//
// ScanPoint2D

class ScanPoint2DTest : public ::testing::Test {
protected: // types
  using SPPT = ScanPoint2D::PointType;
protected: // consts
  static constexpr double Acc_Error = 0.001;
protected: // methods

  void smoke_check_point(SPPT type,
                         const double range, const double angle_rad,
                         const double x, const double y) {
    ScanPoint2D sp;
    switch (type) {
    case SPPT::Polar:
      sp = ScanPoint2D{type, range, angle_rad, true};
      break;
    case SPPT::Cartesian:
      sp = ScanPoint2D{type, x, y, true};
      break;
    default:
      assert(0 && "Unknown ScanPoint2D type");
    }

    ASSERT_NEAR(range, sp.range(), Acc_Error);
    ASSERT_NEAR(angle_rad, sp.angle(), Acc_Error);
    ASSERT_NEAR(x, sp.x(), Acc_Error);
    ASSERT_NEAR(y, sp.y(), Acc_Error);
  }

  void assert_scan_point(const ScanPoint2D &expected,
                         const ScanPoint2D &actual) {
    ASSERT_NEAR(expected.range(), actual.range(), Acc_Error);
    ASSERT_NEAR(expected.angle(), actual.angle(), Acc_Error);
    ASSERT_NEAR(expected.x(), actual.x(), Acc_Error);
    ASSERT_NEAR(expected.y(), actual.y(), Acc_Error);
    ASSERT_EQ(expected.is_occupied(), actual.is_occupied());
  }
};

TEST_F(ScanPoint2DTest, polar1Quad) {
  smoke_check_point(SPPT::Polar, 5, deg2rad(30), 4.3301, 2.5);
}

TEST_F(ScanPoint2DTest, polar2Quad) {
  smoke_check_point(SPPT::Polar, 4, deg2rad(120), -2, 3.4641);
}

TEST_F(ScanPoint2DTest, polar3Quad) {
  smoke_check_point(SPPT::Polar, 2, deg2rad(-150), -1.7320, -1);
}

TEST_F(ScanPoint2DTest, polar4Quad) {
  smoke_check_point(SPPT::Polar, 4, deg2rad(-45), 2.8284, -2.8284);
}

TEST_F(ScanPoint2DTest, cartesian1Quad) {
  smoke_check_point(SPPT::Cartesian, 2.8284, deg2rad(45), 2, 2);
}

TEST_F(ScanPoint2DTest, cartesian2Quad) {
  smoke_check_point(SPPT::Cartesian, 3.6055, deg2rad(146.309), -3, 2);
}

TEST_F(ScanPoint2DTest, cartesian3Quad) {
  smoke_check_point(SPPT::Cartesian, 3.6055, deg2rad(-123.69), -2, -3);
}

TEST_F(ScanPoint2DTest, cartesian4Quad) {
  smoke_check_point(SPPT::Cartesian, 3.1622, deg2rad(-71.565), 1, -3);
}

TEST_F(ScanPoint2DTest, convertFromPolarNoChanges) {
  auto polar_sp = ScanPoint2D{SPPT::Polar, 5, deg2rad(57), false};

  assert_scan_point(polar_sp, polar_sp.to_cartesian(0, 0));
  assert_scan_point(polar_sp, polar_sp.to_polar(0, 0));
}

TEST_F(ScanPoint2DTest, convertToCartesianFromPolarWithAltering) {
  auto polar_sp = ScanPoint2D{SPPT::Polar, 5, deg2rad(57), true};
  const double D_Angle = deg2rad(3), D_Range = -0.1;
  auto cartesian_sp = polar_sp.to_cartesian(D_Angle, D_Range);

  ASSERT_NEAR(polar_sp.range() + D_Range, cartesian_sp.range(), Acc_Error);
  ASSERT_NEAR(polar_sp.angle() + D_Angle, cartesian_sp.angle(), Acc_Error);
  ASSERT_EQ(polar_sp.is_occupied(), cartesian_sp.is_occupied());
}

TEST_F(ScanPoint2DTest, convertFromCartesianNoChanges) {
  auto cartesian_sp = ScanPoint2D{SPPT::Cartesian, 6, 3, true};

  assert_scan_point(cartesian_sp, cartesian_sp.to_cartesian(0, 0));
  assert_scan_point(cartesian_sp, cartesian_sp.to_polar(0, 0));
}

TEST_F(ScanPoint2DTest, convertToPolarFromCartesianWithAltering) {
  auto cartesian_sp = ScanPoint2D{SPPT::Cartesian, 1, -8, false};
  const double D_X = -8, D_Y = 4;
  auto polar_sp = cartesian_sp.to_polar(D_X, D_Y);

  ASSERT_NEAR(cartesian_sp.x() + D_X, polar_sp.x(), Acc_Error);
  ASSERT_NEAR(cartesian_sp.y() + D_Y, polar_sp.y(), Acc_Error);
  ASSERT_EQ(cartesian_sp.is_occupied(), polar_sp.is_occupied());
}

TEST_F(ScanPoint2DTest, convertToCartesianWithProvider) {
  auto polar_sp = ScanPoint2D{SPPT::Polar, 4, deg2rad(10), true};

  const double D_Angle = deg2rad(23);
  auto rtp = std::make_shared<RawTrigonometryProvider>();
  rtp->set_base_angle(D_Angle);
  auto cartesian_sp = polar_sp.to_cartesian(rtp);

  ASSERT_NEAR(polar_sp.range(), cartesian_sp.range(), Acc_Error);
  ASSERT_NEAR(polar_sp.angle() + D_Angle, cartesian_sp.angle(), Acc_Error);
  ASSERT_EQ(polar_sp.is_occupied(), cartesian_sp.is_occupied());
}

//----------------------------------------------------------------------------//
// LaserScan2D

class LaserScan2DTest : public ::testing::Test {
protected: // consts
  static constexpr double Acc_Error = 0.001;
protected: // methods
  void check_sp_rotation(double d_angle, const ScanPoint2D &expected,
                         const ScanPoint2D &actual) {
    ASSERT_NEAR(expected.range(), actual.range(), Acc_Error);
    ASSERT_NEAR(expected.angle() + d_angle, actual.angle(), Acc_Error);
    ASSERT_EQ(expected.is_occupied(), actual.is_occupied());
  }
};

TEST_F(LaserScan2DTest, toCartesianWithRotation) {
  auto map = UnboundedPlainGridMap{std::make_shared<MockGridCell>(),
                                   GridMapParams{100, 100, 1}};
  const auto Top_Bnd_Pos = CecumTextRasterMapPrimitive::BoundPosition::Top;
  auto cecum_mp = CecumTextRasterMapPrimitive{7, 4, Top_Bnd_Pos};
  GridMapPatcher{}.apply_text_raster(map, cecum_mp.to_stream(), {}, 1, 1);
  auto mp_free_space = cecum_mp.free_space()[0];

  auto pose = RobotPose{mp_free_space.left() + mp_free_space.hside_len() / 2,
                        mp_free_space.bot() + mp_free_space.vside_len() / 2,
                        deg2rad(90)};

  auto lsg = LaserScanGenerator{{30, deg2rad(30.0), deg2rad(135.0)}};
  auto scan = lsg.laser_scan_2D(map, pose, 1);

  const auto D_Angle = deg2rad(7);
  auto rotated_scan = scan.to_cartesian(D_Angle);

  ASSERT_EQ(scan.points().size(), rotated_scan.points().size());
  for (std::size_t i = 0; i < scan.points().size(); ++i) {
    check_sp_rotation(D_Angle, scan.points()[i], rotated_scan.points()[i]);
  }
}

//------------------------------------------------------------------------------

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
