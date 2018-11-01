#include <gtest/gtest.h>

#include <limits>

#include "../mock_grid_cell.h"
#include "scan_matcher_test_utils.h"

#include "../../../src/core/scan_matchers/occupancy_observation_probability.h"
#include "../../../src/core/scan_matchers/bf_multi_res_scan_matcher.h"
#include "../../../src/core/maps/plain_grid_map.h"
#include "../../../src/core/maps/lazy_tiled_grid_map.h"
#include "../../../src/core/maps/rescalable_caching_grid_map.h"

#include "../../../src/slams/viny/viny_grid_cell.h"

template <typename Map, typename GridCellType = VinyDSCell>
class BFMRScanMatcherTestBase
  :  public ScanMatcherTestBase<Map, GridCellType> {
protected: // names
  using SPE = typename ScanMatcherTestBase<Map>::DefaultSPE;
  using OOPE = MaxOccupancyObservationPE;
  using SPW = EvenSPW;
protected: // methods
  BFMRScanMatcherTestBase()
    : ScanMatcherTestBase<Map, GridCellType> {
        std::make_shared<SPE>(std::make_shared<OOPE>(),
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

    // auto aoo = AreaOccupancyObservation{true, {1.0, 1.0}, {0, 0}, 1.0};
    // auto &map = this->map;
    // for (unsigned scale_id = map.finest_scale_id();
    //      scale_id <= map.coarsest_scale_id(); ++scale_id) {
    //   std::cout << " === " << scale_id << " === " << std::endl;
    //   map.set_scale_id(scale_id);
    //   auto raw_crd = DiscretePoint2D{0, 0};
    //   for (raw_crd.x = 0; raw_crd.x < map.width(); ++raw_crd.x) {
    //     for (raw_crd.y = 0; raw_crd.y < map.height(); ++raw_crd.y) {
    //       auto coord = map.internal2external(raw_crd);
    //       printf("%.1f", 1.0 - map[coord].discrepancy(aoo));
    //       if (raw_crd.y % 2) { std::cout << "|"; }
    //     }
    //     if (raw_crd.x % 2) { std::cout << "\n--------------------"; }
    //     std::cout << std::endl;
    //   }
    // }

    // std::cout << " [FINE] " << std::endl;
    // map.set_scale_id(map.finest_scale_id());
    // std::cout << "SCALE: " << map.scale() << " [15, 3]" << std::endl;
    // std::cout << map[map.internal2external({15, 3})].discrepancy(aoo)
    //           << std::endl;

    // auto fine_area   = (const VinyDSCell&)map[map.internal2external({15, 3})];
    // auto &ftbm = fine_area.belief();
    // std::cout << "PROB: " << double(fine_area) << std::endl;
    // std::cout << "DISC: " << 1.0 - fine_area.discrepancy(aoo) << std::endl;
    // std::cout << "U: " << ftbm.unknown() << "; E: " << ftbm.empty()
    //           << "; O: " << ftbm.occupied() << "; C: " << ftbm.conflict()
    //           << std::endl;

    // std::cout << " [COARSE] " << std::endl;
    // map.set_scale_id(1);
    // auto coarse_area = (const VinyDSCell&)map[map.internal2external({8, 2})];
    // auto &ctbm = coarse_area.belief();
    // std::cout << "PROB: " << double(coarse_area) << std::endl;
    // std::cout << "DISC: " << 1.0 - coarse_area.discrepancy(aoo) << std::endl;
    // std::cout << "U: " << ctbm.unknown() << "; E: " << ctbm.empty()
    //           << "; O: " << ctbm.occupied() << "; C: " << ctbm.conflict()
    //           << std::endl;

    // std::cout << "Scale: " << map.scale() << " [8, 2]: ";
    // std::cout << double(map[map.internal2external({8, 2})]) << std::endl;
    // std::cout << 1.0 - map[map.internal2external({8, 2})].discrepancy(aoo)
    //           << std::endl;
    // map.set_scale_id(map.finest_scale_id());  // FIXME: assert fail if commented
    // std::cout << "[DONE]\n";
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
  : public BFMRScanMatcherTestBase<MapType> {};

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
