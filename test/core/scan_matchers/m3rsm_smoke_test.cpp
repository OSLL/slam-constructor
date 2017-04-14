#include <gtest/gtest.h>

#include <limits>

#include "../mock_grid_cell.h"
#include "scan_matcher_test_utils.h"


#include "../../../src/core/scan_matchers/many_to_many_multires_scan_matcher.h"

#include "../../../src/core/maps/zoomable_grid_map.h"
#include "../../../src/core/maps/plain_grid_map.h"

//------------------------------------------------------------------------------
// Smoke Tests Suite
// NB: the suit checks _fundamental_ abilities to find a correction.

class M3RScanMatcherSmokeTest
  :  public ScanMatcherTestBase<ZoomableGridMap<UnboundedPlainGridMap>> {
protected: // methods
  M3RScanMatcherSmokeTest()
    : ScanMatcherTestBase{std::make_shared<DiscrepancySumCostEstimator>(),
                          Map_Width, Map_Height, Map_Scale,
                          to_lsp(LS_Max_Dist, LS_FoW, LS_Pts_Nm)}
    , m3rsm{sce, SM_Ang_Step, SM_Ang_Range,
            SM_Transl_Err_Factor, SM_Transl_Range, SM_Transl_Range} {}
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
  static constexpr int LS_Pts_Nm = 10;

  // scan matcher
  static constexpr double SM_Ang_Step = deg2rad(0.5);
  static constexpr double SM_Ang_Range = deg2rad(10);
  static constexpr double SM_Transl_Range = 1; // meter
  static constexpr double SM_Transl_Err_Factor = 2;

protected: // fields

  GridScanMatcher& scan_matcher() override { return m3rsm; };

  RobotPoseDelta default_acceptable_error() override {
    return {Map_Scale / SM_Transl_Err_Factor, Map_Scale / SM_Transl_Err_Factor,
            SM_Ang_Step};
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
  ManyToManyMultiResoultionScanMatcher<UnboundedPlainGridMap> m3rsm;
};

//------------------------------------------------------------------------------
// Tests

TEST_F(M3RScanMatcherSmokeTest, cecumNoPoseNoise) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, 0, 0});
}

TEST_F(M3RScanMatcherSmokeTest, cecumLinStepXLeftDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{-SM_Transl_Range / 2, 0, 0});
}

TEST_F(M3RScanMatcherSmokeTest, cecumLinStepXRightDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{SM_Transl_Range / 2, 0, 0});
}

TEST_F(M3RScanMatcherSmokeTest, cecumLinStepYUpDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, -SM_Transl_Range / 2, 0});
}

TEST_F(M3RScanMatcherSmokeTest, cecumLinStepYDownDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, SM_Transl_Range / 2, 0});
}

TEST_F(M3RScanMatcherSmokeTest, cecumAngStepThetaCcwDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, 0, SM_Ang_Range});
}

TEST_F(M3RScanMatcherSmokeTest, cecumAngStepThetaCwDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher(RobotPoseDelta{0, 0, -SM_Ang_Range});
}

TEST_F(M3RScanMatcherSmokeTest, cecumComboStepsDrift) {
  init_pose_facing_top_cecum_bound();
  test_scan_matcher({-SM_Transl_Range/2, SM_Transl_Range/2, -SM_Ang_Range/2});
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
