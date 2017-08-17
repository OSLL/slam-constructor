#include <gtest/gtest.h>

#include "../mock_grid_cell.h"
#include "scan_matcher_test_utils.h"

#include "../../../src/core/scan_matchers/brute_force_scan_matcher.h"
#include "../../../src/core/scan_matchers/occupancy_observation_probability.h"
#include "../../../src/core/maps/plain_grid_map.h"

// TODO: BruteForcePoseEnumeratorTest

//------------------------------------------------------------------------------
// Smoke Tests Suite
// NB: the suit checks _fundamental_ abilities to find a correction.

class BruteForceScanMatcherSmokeTest
  : public ScanMatcherTestBase<UnboundedPlainGridMap> {

  // TODO: remove duplication (hc-test, m3mrsm-test)
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
  static constexpr double From_Translation = -0.5;
  static constexpr double To_Translation   = 0.5;
  static constexpr double Step_Translation = 0.05;
  static constexpr double From_Rotation = deg2rad(-10);
  static constexpr double To_Rotation   = deg2rad(10);
  static constexpr double Step_Rotation = deg2rad(1);
protected: // type aliases
  using SPE = typename ScanMatcherTestBase<UnboundedPlainGridMap>::DefaultSPE;
  using OOPE = ObstacleBasedOccupancyObservationPE;
  using SPW = EvenSPW;
protected: // methods
  BruteForceScanMatcherSmokeTest()
    : ScanMatcherTestBase{std::make_shared<SPE>(std::make_shared<OOPE>(),
                                                std::make_shared<SPW>()),
                          Map_Width, Map_Height, Map_Scale,
                          to_lsp(LS_Max_Dist, LS_FoW, LS_Pts_Nm)}
    , _bfsm{spe,
            From_Translation, To_Translation, Step_Translation,
            From_Translation, To_Translation, Step_Translation,
            From_Rotation, To_Rotation, Step_Rotation} {}

  GridScanMatcher& scan_matcher() override { return _bfsm; };

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
  BruteForceScanMatcher _bfsm;
};

TEST_F(BruteForceScanMatcherSmokeTest, cecumNoPoseNoise) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, 0, 0});
}

TEST_F(BruteForceScanMatcherSmokeTest, cecumLinStepXLeftDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{From_Translation, 0, 0});
}

TEST_F(BruteForceScanMatcherSmokeTest, cecumLinStepXRightDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{2*Step_Translation, 0, 0});
}

TEST_F(BruteForceScanMatcherSmokeTest, cecumLinStepYUpDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, -2*Step_Translation, 0});
}

TEST_F(BruteForceScanMatcherSmokeTest, cecumLinStepYDownDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, To_Translation, 0});
}

TEST_F(BruteForceScanMatcherSmokeTest, cecumAngStepThetaCcwDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, 0, -3*Step_Rotation});
}

TEST_F(BruteForceScanMatcherSmokeTest, cecumAngStepThetaCwDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, 0, To_Rotation});
}

TEST_F(BruteForceScanMatcherSmokeTest, cecumComboStepsDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher({2*Step_Translation, -3*Step_Translation, Step_Rotation});
}

//----------------------------------------------------------------------------//

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
