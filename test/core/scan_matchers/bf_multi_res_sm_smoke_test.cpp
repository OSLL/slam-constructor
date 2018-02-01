#include <gtest/gtest.h>

#include <limits>

#include "../mock_grid_cell.h"
#include "scan_matcher_test_utils.h"

#include "../../../src/core/scan_matchers/occupancy_observation_probability.h"
#include "../../../src/core/scan_matchers/bf_multi_res_scan_matcher.h"
#include "../../../src/core/maps/plain_grid_map.h"
#include "../../../src/core/maps/lazy_tiled_grid_map.h"
#include "../../../src/core/maps/rescalable_caching_grid_map.h"

template <typename Map>
class BFMRScanMatcherTestBase
  :  public ScanMatcherTestBase<Map> {
protected: // names
  using SPE = typename ScanMatcherTestBase<Map>::DefaultSPE;
  using OOPE = MaxOccupancyObservationPE;
  using SPW = EvenSPW;
protected: // methods
  BFMRScanMatcherTestBase()
    : ScanMatcherTestBase<Map>{std::make_shared<SPE>(std::make_shared<OOPE>(),
                                                     std::make_shared<SPW>()),
                                   Map_Width, Map_Height, Map_Scale,
                                   to_lsp(LS_Max_Dist, LS_FoW, LS_Pts_Nm)}
    , bfmrsm{this->spe, SM_Ang_Step, SM_Transl_Step} {
    bfmrsm.set_lookup_ranges(SM_Max_Translation_Error, SM_Max_Translation_Error,
                            SM_Max_Rotation_Error);
  }
protected: // consts
  // map patching params
  static constexpr int Cecum_Patch_W = 15, Cecum_Patch_H = 13;
  static constexpr int Patch_Scale = 1;

  // NB: use 1/2^n is order to increase perfromance
  static constexpr double Map_Scale = 0.125;
  static constexpr int Map_Width = 100;
  static constexpr int Map_Height = 100;

  // laser scanner params
  static constexpr double LS_Max_Dist = 15;
  static constexpr int LS_FoW = 270;
  static constexpr int LS_Pts_Nm = 5000;

  // scan matcher
  static constexpr double SM_Max_Rotation_Error = deg2rad(5);
  static constexpr double SM_Max_Translation_Error = Map_Scale * 10; // meters
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
    this->add_primitive_to_map(cecum_mp, {}, Patch_Scale, Patch_Scale);

    this->rpose += RobotPoseDelta{
      (cecum_mp.width() * Patch_Scale / 2) * this->map.scale(),
      (-cecum_mp.height() * Patch_Scale + 1) * this->map.scale(),
      deg2rad(90)
    };
  }

protected: // fields
  BruteForceMultiResolutionScanMatcher bfmrsm;
};


//------------------------------------------------------------------------------
// Smoke Tests Suite
// NB: the suit checks _fundamental_ abilities to find a correction.
//     The suit check raw (w/o a map approximator speed up) BFMRSM correctness.

template <typename MapType>
class BFMRScanMatcherSmokeTest
  :  public BFMRScanMatcherTestBase<MapType> {};

//------------------------------------------------------------------------------
// Tests

using MapTs = ::testing::Types<UnboundedPlainGridMap,
                               RescalableCachingGridMap<UnboundedPlainGridMap>>;

TYPED_TEST_CASE(BFMRScanMatcherSmokeTest, MapTs);

TYPED_TEST(BFMRScanMatcherSmokeTest, cecumNoPoseNoise) {
  this->init_pose_facing_top_cecum_bound();
  this->test_scan_matcher(RobotPoseDelta{0, 0, 0});
}

TYPED_TEST(BFMRScanMatcherSmokeTest, cecumLinStepXLeftDrift) {
  const auto Translation_Error = this->SM_Max_Translation_Error / 2;
  this->init_pose_facing_top_cecum_bound();
  this->test_scan_matcher(RobotPoseDelta{-Translation_Error, 0, 0});
}

TYPED_TEST(BFMRScanMatcherSmokeTest, cecumLinStepXRightDrift) {
  const auto Translation_Error = this->SM_Max_Translation_Error / 2;
  this->init_pose_facing_top_cecum_bound();
  this->test_scan_matcher(RobotPoseDelta{Translation_Error, 0, 0});
}

TYPED_TEST(BFMRScanMatcherSmokeTest, cecumLinStepYUpDrift) {
  const auto Translation_Error = this->SM_Max_Translation_Error / 2;
  this->init_pose_facing_top_cecum_bound();
  this->test_scan_matcher(RobotPoseDelta{0, -Translation_Error, 0});
}

TYPED_TEST(BFMRScanMatcherSmokeTest, cecumLinStepYDownDrift) {
  const auto Translation_Error = this->SM_Max_Translation_Error / 2;
  this->init_pose_facing_top_cecum_bound();
  this->test_scan_matcher(RobotPoseDelta{0, Translation_Error, 0});
}

TYPED_TEST(BFMRScanMatcherSmokeTest, cecumAngStepThetaCcwDrift) {
  const auto Rotation_Error = this->SM_Max_Rotation_Error;
  this->init_pose_facing_top_cecum_bound();
  this->test_scan_matcher(RobotPoseDelta{0, 0, Rotation_Error});
}

TYPED_TEST(BFMRScanMatcherSmokeTest, cecumAngStepThetaCwDrift) {
  const auto Rotation_Error = this->SM_Max_Rotation_Error;
  this->init_pose_facing_top_cecum_bound();
  this->test_scan_matcher(RobotPoseDelta{0, 0, -Rotation_Error});
}

TYPED_TEST(BFMRScanMatcherSmokeTest, cecumComboStepsDrift) {
  const auto Translation_Error = this->SM_Max_Translation_Error / 2;
  const auto Rotation_Error = this->SM_Max_Rotation_Error;
  this->init_pose_facing_top_cecum_bound();
  auto noise = RobotPoseDelta{-Translation_Error, Translation_Error,
                              -Rotation_Error};
  this->test_scan_matcher(noise);
}

//------------------------------------------------------------------------------
// A suit with tests specific to RescalabeMap
// Motivation: some test cases run slowly on a plain map.

using RescalableMap = RescalableCachingGridMap<UnboundedPlainGridMap>;

class BFMRScanMatcherResclalableMapSpecificTest
  :  public BFMRScanMatcherTestBase<RescalableMap> {};

TEST_F(BFMRScanMatcherResclalableMapSpecificTest, hugeWindowLookup) {
  const auto Translation_Error = this->SM_Max_Translation_Error / 2;
  const auto Rotation_Error = this->SM_Max_Rotation_Error;
  this->init_pose_facing_top_cecum_bound();
  auto noise = RobotPoseDelta{-Translation_Error, Translation_Error,
                              Rotation_Error};
  this->bfmrsm.set_lookup_ranges(this->SM_Max_Translation_Error * 10,
                                 this->SM_Max_Translation_Error * 10,
                                 this->SM_Max_Rotation_Error);
  this->test_scan_matcher(noise);
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
