#include <gtest/gtest.h>

#include <limits>

#include "../mock_grid_cell.h"
#include "scan_matcher_test_utils.h"

#include "../../../src/core/scan_matchers/bf_multi_res_scan_matcher.h"
#include "../../../src/core/maps/plain_grid_map.h"

//------------------------------------------------------------------------------
// Smoke Tests Suite
// NB: the suit checks _fundamental_ abilities to find a correction.
//     The suit check raw (w/o a map approximator speed up) BFMRSM correctness.

class BFMRScanMatcherSmokeTest
  :  public ScanMatcherTestBase<UnboundedPlainGridMap> {
protected: // methods
  BFMRScanMatcherSmokeTest()
    : ScanMatcherTestBase{std::make_shared<DefaultSPE>(),
                          Map_Width, Map_Height, Map_Scale,
                          to_lsp(LS_Max_Dist, LS_FoW, LS_Pts_Nm)}
    , bfmrsm{spe, SM_Ang_Step, SM_Transl_Step} {
    bfmrsm.set_lookup_ranges(SM_Max_Translation_Error, SM_Max_Translation_Error,
                            SM_Max_Rotation_Error);
  }
protected: // consts
  // map patching params
  static constexpr int Cecum_Patch_W = 15, Cecum_Patch_H = 13;
  static constexpr int Patch_Scale = 1;

  static constexpr double Map_Scale = 0.1;
  static constexpr int Map_Width = 100;
  static constexpr int Map_Height = 100;

  // laser scanner params
  static constexpr double LS_Max_Dist = 15;
  static constexpr int LS_FoW = 270;
  static constexpr int LS_Pts_Nm = 1000;

  // scan matcher
  static constexpr double SM_Max_Rotation_Error = deg2rad(5);
  static constexpr double SM_Max_Translation_Error = 1; // meters
  static constexpr double SM_Ang_Step = deg2rad(0.5);
  static constexpr double SM_Transl_Step = Map_Scale / 2;

protected: // fields

  GridScanMatcher& scan_matcher() override { return bfmrsm; };

  RobotPoseDelta default_acceptable_error() override {
    return {SM_Transl_Step, SM_Transl_Step, SM_Ang_Step};
  }

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
  BruteForceMultiResoultionScanMatcher bfmrsm;
};

//------------------------------------------------------------------------------
// Tests

TEST_F(BFMRScanMatcherSmokeTest, cecumNoPoseNoise) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, 0, 0});
}

TEST_F(BFMRScanMatcherSmokeTest, cecumLinStepXLeftDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{-SM_Max_Translation_Error / 2, 0, 0});
}

TEST_F(BFMRScanMatcherSmokeTest, cecumLinStepXRightDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{SM_Max_Translation_Error / 2, 0, 0});
}

TEST_F(BFMRScanMatcherSmokeTest, cecumLinStepYUpDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, -SM_Max_Translation_Error / 2, 0});
}

TEST_F(BFMRScanMatcherSmokeTest, cecumLinStepYDownDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, SM_Max_Translation_Error / 2, 0});
}

TEST_F(BFMRScanMatcherSmokeTest, cecumAngStepThetaCcwDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, 0, SM_Max_Rotation_Error});
}

TEST_F(BFMRScanMatcherSmokeTest, cecumAngStepThetaCwDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, 0, -SM_Max_Rotation_Error});
}

TEST_F(BFMRScanMatcherSmokeTest, cecumComboStepsDrift) {
  init_pose_facing_top_cecum_bound();
  auto noise = RobotPoseDelta{-SM_Max_Translation_Error/2,
                              SM_Max_Translation_Error/2,
                              -SM_Max_Rotation_Error/2};
  test_scan_matcher(noise);
}

//------------------------------------------------------------------------------

// TODO: More sophisticated testing (e.g. an arbitrary noise cases,
//       high res map/scanner, map state, etc.) is supposed to be implemented
//       with a separate test suite.

//------------------------------------------------------------------------------

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
