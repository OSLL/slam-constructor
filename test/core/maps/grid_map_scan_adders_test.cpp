#include <gtest/gtest.h>

#include "../../../src/core/maps/grid_map_scan_adders.h"

#include <memory>

#include "../mock_grid_cell.h"
#include "../../../src/core/maps/const_occupancy_estimator.h"
#include "../../../src/core/maps/plain_grid_map.h"
#include "../../../src/core/math_utils.h"
#include "../../../src/utils/data_generation/grid_map_patcher.h"
#include "../../../src/utils/data_generation/laser_scan_generator.h"
#include "../../../src/utils/data_generation/map_primitives.h"

//============================================================================//
//===                                 Tests                                ===//
//============================================================================//

//----------------------------------------------------------------------------//
// Plain scan integration

constexpr Occupancy Occup = {1.0, 1.0}, Empty = {0.0, 1.0};

class PlainScanIntegrationTest : public ::testing::Test {
protected:
  static constexpr int MAP_WIDTH = 100, MAP_HEIGHT = 100;
  static constexpr double MAP_SCALE = 1;

protected:

  PlainScanIntegrationTest()
    : cell_proto{std::make_shared<MockGridCell>()}
    , src_map{cell_proto, {MAP_WIDTH, MAP_HEIGHT, MAP_SCALE}}
    , dst_map{cell_proto, {MAP_WIDTH, MAP_HEIGHT, MAP_SCALE}}
    , pose{0, 0, 0} {}

  std::shared_ptr<GridMapScanAdder> plain_adder() const {
    auto oe = std::make_shared<ConstOccupancyEstimator>(Occup, Empty);
    auto builder = WallDistanceBlurringScanAdder::builder();
    return builder.set_occupancy_estimator(oe)
                  .build();
  }

  bool map_was_modified(const GridMap &map) const {
    auto unknown_prob = static_cast<double>(*cell_proto);
    auto raw_coord = GridMap::Coord{0, 0};
    // FIXME: code duplication. Implement coords "iterator".
    for (raw_coord.x = 0; raw_coord.x < map.width(); ++raw_coord.x) {
      for (raw_coord.y = 0; raw_coord.y < map.width(); ++raw_coord.y) {
        auto coord = map.internal2external(raw_coord);
        if (!are_equal(unknown_prob, map[coord])) {
          return true;
        }
      }
    }
    return false;
  }

  bool map_includes(const GridMap &outer, const GridMap &inner) const {
    auto unknown_prob = static_cast<double>(*cell_proto);
    if (outer.width() != inner.width() || outer.height() != inner.height() ||
        outer.scale() != inner.scale() || outer.origin() != inner.origin()) {
      return false;
    }

    auto raw_coord = GridMap::Coord{0, 0};
    for (raw_coord.x = 0; raw_coord.x < inner.width(); ++raw_coord.x) {
      for (raw_coord.y = 0; raw_coord.y < inner.width(); ++raw_coord.y) {
        auto coord = inner.internal2external(raw_coord);
        if (are_equal(unknown_prob, inner[coord])) { continue; }
        if (!are_equal(inner[coord], outer[coord])) { return false; }
      }
    }
    return true;
  }

  bool map_contains_scan(const GridMap &map, const LaserScan2D &scan) {
    for (auto &sp : scan.points()) {
      const auto &wp = sp.move_origin(pose.point(), scan.trig_provider);
      const auto coord = map.world_to_cell(wp);
      if (!are_equal(1.0, map[coord])) { return false; }
    }
      return true;
  }

protected:
  std::shared_ptr<GridCell> cell_proto;
  UnboundedPlainGridMap src_map, dst_map;
  RobotPose pose;
};

TEST_F(PlainScanIntegrationTest, smokeScanAdditionTest) {
  // setup map, pose, scan
  auto cecum = CecumTextRasterMapPrimitive{50, 50,
    CecumTextRasterMapPrimitive::BoundPosition::Top};
  GridMapPatcher{}.apply_text_raster(src_map, cecum.to_stream());
  pose += RobotPoseDelta{src_map.scale() / 2, src_map.scale() / 2, deg2rad(90)};

  constexpr auto lsp = to_lsp(MAP_WIDTH * 2, 270, 100);
  auto scan = LaserScanGenerator{lsp}.laser_scan_2D(src_map, pose, 1);

  // test
  plain_adder()->append_scan(dst_map, pose, scan, 1.0 /* quality */, 0);
  ASSERT_TRUE(map_was_modified(dst_map));

  scan.trig_provider->set_base_angle(pose.theta);
  ASSERT_TRUE(map_contains_scan(dst_map, scan));
  ASSERT_TRUE(map_includes(src_map, dst_map));
}

//----------------------------------------------------------------------------//
// Distance-based wall blurring (hole-width from tinySLAM)

class DistanceBasedWallBlurringTest : public PlainScanIntegrationTest {
protected:
  static constexpr int MAP_WIDTH = 50, MAP_HEIGHT = 50;
  static constexpr double MAP_SCALE = 1;

protected:
  DistanceBasedWallBlurringTest()
    : cell_proto{std::make_shared<MockGridCell>()}
    , src_map{cell_proto, {MAP_WIDTH, MAP_HEIGHT, MAP_SCALE}}
    , dst_map{cell_proto, {MAP_WIDTH, MAP_HEIGHT, MAP_SCALE}}
    , pose{0, 0, 0} {}

  std::shared_ptr<GridMapScanAdder> adder(double blurring_width) const {
    auto oe = std::make_shared<ConstOccupancyEstimator>(Occup, Empty);
    auto builder = WallDistanceBlurringScanAdder::builder();
    return builder.set_occupancy_estimator(oe)
                  .set_blur_distance(blurring_width)
                  .build();
  }

protected:
  std::shared_ptr<GridCell> cell_proto;
  UnboundedPlainGridMap src_map, dst_map;
  RobotPose pose;
};

TEST_F(DistanceBasedWallBlurringTest, smokeDistanceBaseBlurringTest) {
  constexpr auto Obstacle_Coord = GridMap::Coord{20, 0};
  // setup pose and map
  pose += RobotPoseDelta{src_map.scale() / 2, src_map.scale() / 2, 0};
  auto aoo = AreaOccupancyObservation{true, {1.0, 1.0}, {0, 0}, 1.0};
  src_map.update(Obstacle_Coord, aoo);

  // setup scan
  constexpr auto lsp = to_lsp(MAP_WIDTH * 2, 360, 100);
  auto scan = LaserScanGenerator{lsp}.laser_scan_2D(src_map, pose, 1);

  // test
  constexpr double Blur_Distance = 3;
  adder(Blur_Distance)->append_scan(dst_map, pose, scan, 1.0, 0);
  ASSERT_TRUE(map_was_modified(dst_map));

  scan.trig_provider->set_base_angle(pose.theta);
  ASSERT_TRUE(map_contains_scan(dst_map, scan));

  double prev_prob = 0;
  auto beam_coord = dst_map.world_to_cell(pose.point());
  unsigned occupied_cells = 0;
  while (beam_coord.x <= Obstacle_Coord.x) {
    if (prev_prob == 0) {
      ASSERT_TRUE(0 <= dst_map[beam_coord]);
    } else {
      ASSERT_TRUE(prev_prob < dst_map[beam_coord]);
    }
    prev_prob = dst_map[beam_coord];
    occupied_cells += 0 < prev_prob ? 1 : 0;
    ++beam_coord.x;
  }
  ASSERT_TRUE(1 < occupied_cells);
}

//============================================================================//

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
