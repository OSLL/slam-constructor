#include <gtest/gtest.h>

#include <limits>

#include "../mock_grid_cell.h"
#include "scan_matcher_test_utils.h"

#include "../../../src/core/scan_matchers/observation_impact_estimators.h"
#include "../../../src/core/scan_matchers/occupancy_observation_probability.h"
#include "../../../src/core/scan_matchers/bf_multi_res_scan_matcher.h"
#include "../../../src/core/maps/plain_grid_map.h"
#include "../../../src/core/maps/lazy_tiled_grid_map.h"

template <typename Map>
class BFMRScanMatcherTestBase
  : public ScanMatcherTestBase {
protected: // names
  using SPE = typename ScanMatcherTestBase::DefaultSPE;
  using OOPE = MaxOccupancyObservationPE;
  using OIE = DiscrepancyOIE;
  using SPW = EvenSPW;
protected: // methods
  BFMRScanMatcherTestBase(std::shared_ptr<Map> map)
    : ScanMatcherTestBase{
        // NB: assume OIE is stateless for M3RSMRescalableMap cases
        map,
        std::make_shared<SPE>(std::make_shared<OOPE>(std::make_shared<OIE>()),
                              std::make_shared<SPW>()),
        to_lsp(LS_Max_Dist, LS_FoW, LS_Pts_Nm)
      }
    , bfmrsm{spe, SM_Ang_Step, SM_Transl_Step} {
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
    add_primitive_to_map(cecum_mp, {}, Patch_Scale, Patch_Scale);

    double map_scale = map().scale();
    rpose += RobotPoseDelta{(cecum_mp.width() * Patch_Scale / 2) * map_scale,
                            (-cecum_mp.height() * Patch_Scale + 1) * map_scale,
                            deg2rad(90)};
  }

protected: // fields
  BruteForceMultiResolutionScanMatcher bfmrsm;
};

template <typename MapT, typename OIE>
std::shared_ptr<MapT> make_m3rsm_map(int w, int h, double scale) {
  return std::make_shared<MapT>(std::make_shared<OIE>(),
                                std::make_shared<MockGridCell>(),
                                GridMapParams{w, h, scale});
}

//------------------------------------------------------------------------------
// Smoke Tests Suite
// NB: the suit checks _fundamental_ abilities to find a correction.
//     The suit check raw (w/o a map approximator speed up) BFMRSM correctness.

template <typename MapT>
class BFMRScanMatcherSmokeTest
  : public BFMRScanMatcherTestBase<MapT> {
public:
  using T = BFMRScanMatcherTestBase<MapT>;
  BFMRScanMatcherSmokeTest()
    : T{make_test_map<MapT>(T::Map_Width, T::Map_Height, T::Map_Scale)} {}
};

template <typename BackMapT>
class BFMRScanMatcherSmokeTest<M3RSMRescalableGridMap<BackMapT>>
  : public BFMRScanMatcherTestBase<M3RSMRescalableGridMap<BackMapT>> {
public:
  using MapT = M3RSMRescalableGridMap<BackMapT>;
  using T = BFMRScanMatcherTestBase<MapT>;
  using OIE = typename T::OIE;
  BFMRScanMatcherSmokeTest()
    : T{make_m3rsm_map<MapT, OIE>(T::Map_Width, T::Map_Height, T::Map_Scale)} {}
};

//------------------------------------------------------------------------------
// Tests

using MapTs = ::testing::Types<
  UnboundedPlainGridMap,
  M3RSMRescalableGridMap<UnboundedPlainGridMap>,
  M3RSMRescalableGridMap<UnboundedLazyTiledGridMap>>;

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

using RescalableMap = M3RSMRescalableGridMap<UnboundedPlainGridMap>;

class BFMRScanMatcherResclalableMapSpecificTest
  : public BFMRScanMatcherTestBase<RescalableMap> {
public:
  using MapT = RescalableMap;
  using T = BFMRScanMatcherTestBase<MapT>;
  using OIE = typename T::OIE;
  BFMRScanMatcherResclalableMapSpecificTest()
    : T{make_m3rsm_map<MapT, OIE>(T::Map_Width, T::Map_Height, T::Map_Scale)} {}
};

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
