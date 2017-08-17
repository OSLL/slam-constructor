#include <gtest/gtest.h>

#include <limits>

#include "../mock_grid_cell.h"
#include "scan_matcher_test_utils.h"

#include "../../../src/core/scan_matchers/hill_climbing_scan_matcher.h"
#include "../../../src/core/scan_matchers/occupancy_observation_probability.h"
#include "../../../src/core/maps/plain_grid_map.h"

//------------------------------------------------------------------------------
// Smoke Tests Suite
// NB: the suit checks _fundamental_ abilities to find a correction.

class HillClimbingScanMatcherSmokeTest
  : public ScanMatcherTestBase<UnboundedPlainGridMap> {
protected: // consts
  // map params
  static constexpr int Map_Width = 100;
  static constexpr int Map_Height = 100;
  static constexpr double Map_Scale = 0.1;

  // map patching params
  static constexpr int Cecum_Patch_W = 15, Cecum_Patch_H = 13;
  static constexpr int Patch_Scale = 1;

  // laser scanner params
  static constexpr double LS_Max_Dist = 15;
  static constexpr int LS_FoW = 270;
  static constexpr int LS_Pts_Nm = 10;

  // scan matcher params
  static constexpr int Max_SM_Shirnks_Nm = 10;
  static constexpr double Init_Lin_Step = 0.1;
  static constexpr double Init_Ang_Step = deg2rad(30);
protected: // type aliases
  using SPE = typename ScanMatcherTestBase<UnboundedPlainGridMap>::DefaultSPE;
  using OOPE = ObstacleBasedOccupancyObservationPE;
  using SPW = EvenSPW;
protected: // methods
  HillClimbingScanMatcherSmokeTest()
    : ScanMatcherTestBase{std::make_shared<SPE>(std::make_shared<OOPE>(),
                                                std::make_shared<SPW>()),
                          Map_Width, Map_Height, Map_Scale,
                          to_lsp(LS_Max_Dist, LS_FoW, LS_Pts_Nm)}
    , _hcsm{spe, Max_SM_Shirnks_Nm, Init_Lin_Step, Init_Ang_Step} {}

  GridScanMatcher& scan_matcher() override { return _hcsm; };

  void init_pose_facing_top_cecum_bound() {
    using CecumMp = CecumTextRasterMapPrimitive;
    auto bnd_pos = CecumMp::BoundPosition::Top;
    auto cecum_mp = CecumMp{Cecum_Patch_W, Cecum_Patch_H, bnd_pos};
    add_primitive_to_map(cecum_mp, {}, Patch_Scale, Patch_Scale);

    rpose += RobotPoseDelta{
      (cecum_mp.width() * Patch_Scale / 2) * map.scale(),
      (-cecum_mp.height() * Patch_Scale + 1) * map.scale(),
      deg2rad(90)
    };
  }

protected: // fields
  HillClimbingScanMatcher _hcsm;
};

TEST_F(HillClimbingScanMatcherSmokeTest, cecumNoPoseNoise) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, 0, 0});
}

TEST_F(HillClimbingScanMatcherSmokeTest, cecumLinStepXLeftDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{-Init_Lin_Step, 0, 0});
}

TEST_F(HillClimbingScanMatcherSmokeTest, cecumLinStepXRightDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{Init_Lin_Step, 0, 0});
}

TEST_F(HillClimbingScanMatcherSmokeTest, cecumLinStepYUpDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, -Init_Lin_Step, 0});
}

TEST_F(HillClimbingScanMatcherSmokeTest, cecumLinStepYDownDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, Init_Lin_Step, 0});
}

TEST_F(HillClimbingScanMatcherSmokeTest, cecumAngStepThetaCcwDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, 0, Init_Ang_Step});
}

TEST_F(HillClimbingScanMatcherSmokeTest, cecumAngStepThetaCwDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, 0, -Init_Ang_Step});
}

/* FIXME: looks like the method itself should be fixed.
TEST_F(HillClimbingScanMatcherSmokeTest, cecumComboStepsDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher({Init_Lin_Step, -Init_Lin_Step, Init_Ang_Step});
}
*/

//------------------------------------------------------------------------------

// TODO: More sophisticated testing (e.g. an arbitrary noise cases,
//       high res map/scanner, map state, etc.) is supposed to be implemented
//       with a separate test suite.

//------------------------------------------------------------------------------

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
