#include <iostream>
#include <memory>
#include <utility>
#include <chrono>

#include "../core/maps/grid_cell.h"
#include "../core/maps/plain_grid_map.h"
#include "data_generation/map_primitives.h"
#include "data_generation/grid_map_patcher.h"
#include "data_generation/laser_scan_generator.h"
#include "map_dumpers.h"

#include "../core/maps/grid_map_scan_adders.h"
#include "../core/maps/const_occupancy_estimator.h"
#include "../core/scan_matchers/occupancy_observation_probability.h"
#include "../core/scan_matchers/weighted_mean_point_probability_spe.h"
#include "../core/scan_matchers/brute_force_scan_matcher.h"

// FIXME: code duplication with tests's MockGridCell
class LastWriteWinsGridCell : public GridCell {
public:
  LastWriteWinsGridCell(double prob = 0.5) : GridCell{Occupancy{prob, 0}} {}
  std::unique_ptr<GridCell> clone() const override {
    return std::make_unique<LastWriteWinsGridCell>(*this);
  }
  void operator+= (const AreaOccupancyObservation &aoo) override {
    _occupancy = aoo.occupancy;
  }
};

class ScanMatcherSearchSpaceBuilder : public GridScanMatcherObserver {
public:

  ScanMatcherSearchSpaceBuilder(double resolution)
    : _map{
      std::make_shared<UnboundedPlainGridMap>(
        std::make_shared<LastWriteWinsGridCell>(),
        GridMapParams{100, 100, resolution}
       )} {}
  
  std::shared_ptr<GridMap> map() {
    return _map;
  }
  
  void on_scan_test(const RobotPose &pose, const LaserScan2D &,
                    double score) override {
    auto coord = _map->world_to_cell({pose.x, pose.y});
    auto eff_score = score;

    // if (are_equal(pose.x, 0.05)) {
    //    std::cout << pose << " -> " << score << " ~ "
    //              << eff_score << std::endl;
    // }
    
    _map->update(coord, {true, {eff_score, 0}, {0, 0}, 1.0});
  }

private:
  std::shared_ptr<GridMap> _map;
};

void run_evaluation(const GridMap &map, double resolution);

void run_closed_corridor_case() {
  std::cout << "[1] Closed Corridor" << std::endl;
  auto map = UnboundedPlainGridMap{std::make_shared<LastWriteWinsGridCell>(),
                                   {100, 100, 0.1}};
  auto primitive = CecumTextRasterMapPrimitive{
    40, 20, CecumTextRasterMapPrimitive::BoundPosition::Right};
  GridMapPatcher{}.apply_text_raster(map, primitive.to_stream(), {0, 10}, 1, 1);

  auto pr = CecumTextRasterMapPrimitive{
    20, 40, CecumTextRasterMapPrimitive::BoundPosition::Top};
  GridMapPatcher{}.apply_text_raster(map, pr.to_stream(), {-20, 49}, 1, 1);
  auto p = CecumTextRasterMapPrimitive{
    20, 40, CecumTextRasterMapPrimitive::BoundPosition::Bot};
  GridMapPatcher{}.apply_text_raster(map, p.to_stream(), {-20, -9}, 1, 1);
  for (int x = -20; x < 0; x++) {
    for (int y = -8; y < 10; y++) {
      map.update({x, y}, {false, {0, 0}, {0, 0}, 1});
    }
  }

  GridMapToPgmDumber<decltype(map)>{"input_map"}.on_map_update(map);
  // TODO: dump scan
  run_evaluation(map, 0.01);
}

void run_open_corridor_case() {
  std::cout << "[2] Open Corridor" << std::endl;
  auto map = UnboundedPlainGridMap{std::make_shared<LastWriteWinsGridCell>(),
                                   {100, 100, 0.1}};
  auto primitive = CecumTextRasterMapPrimitive{
    80, 20, CecumTextRasterMapPrimitive::BoundPosition::Right};
  GridMapPatcher{}.apply_text_raster(map, primitive.to_stream(),
                                     {-40, 10}, 1, 1);
  // FIXME: none bounded case
  for (int y = -8; y < 10; y++) {
    map.update({39, y}, {false, {0, 0}, {0, 0}, 1});
  }
  
  // auto pr = CecumTextRasterMapPrimitive{
  //   20, 40, CecumTextRasterMapPrimitive::BoundPosition::Top};
  // GridMapPatcher{}.apply_text_raster(map, pr.to_stream(), {-20, 49}, 1, 1);
  // auto p = CecumTextRasterMapPrimitive{
  //   20, 40, CecumTextRasterMapPrimitive::BoundPosition::Bot};
  // GridMapPatcher{}.apply_text_raster(map, p.to_stream(), {-20, -9}, 1, 1);
  // for (int x = -20; x < 0; x++) {
  //   for (int y = -8; y < 10; y++) {
  //     map.update({x, y}, {false, {0, 0}, {0, 0}, 1});
  //   }
  // }

  GridMapToPgmDumber<decltype(map)>{"input_map"}.on_map_update(map);
  // TODO: dump scan
  run_evaluation(map, 0.01);
}

void run_several_corridors_case() {
  std::cout << "[3] Several Corridors" << std::endl;
  auto map = UnboundedPlainGridMap{std::make_shared<LastWriteWinsGridCell>(),
                                   {100, 100, 0.1}};
  auto primitive = CecumTextRasterMapPrimitive{
    40, 5, CecumTextRasterMapPrimitive::BoundPosition::Right};
  GridMapPatcher{}.apply_text_raster(map, primitive.to_stream(), {0, 2}, 1, 1);
  GridMapPatcher{}.apply_text_raster(map, primitive.to_stream(), {0, 7}, 1, 1);


  GridMapToPgmDumber<decltype(map)>{"input_map"}.on_map_update(map);
  // TODO: dump scan
  run_evaluation(map, 0.01);
}

int main(int argc, char **argv) {
  run_closed_corridor_case();
  run_open_corridor_case();
  run_several_corridors_case();
  return 0;
}

void dump_scan(const LaserScan2D &scan, const RobotPose &pose) {
  auto map = UnboundedPlainGridMap{std::make_shared<LastWriteWinsGridCell>(),
                                   {100, 100, 0.1}};
  auto scan_adder = WallDistanceBlurringScanAdder::builder()
    .set_occupancy_estimator(
      std::make_shared<ConstOccupancyEstimator>(Occupancy{1.0, 1.0},
                                                Occupancy{0.0, 1.0}))
    .build();
  scan_adder->append_scan(map, pose, scan, 1.0);
  GridMapToPgmDumber<decltype(map)>{"scan"}.on_map_update(map);
}

void run_evaluation(const GridMap &map, double resolution) {
  auto sssb = std::make_shared<ScanMatcherSearchSpaceBuilder>(resolution);
  auto tscan = TransformedLaserScan{};
  tscan.pose_delta = RobotPoseDelta{};
  // FIXME: scan generation from 0, 0
  tscan.scan = LaserScanGenerator{to_lsp(100, 270, 1000)}
                       .laser_scan_2D(map, {0.05, 0.05, 0}, 1);
  dump_scan(tscan.scan, {0.05, 0.05, 0});
  // generate plan brute force search space map
  auto bfsm = BruteForceScanMatcher{
    std::make_shared<WeightedMeanPointProbabilitySPE>(
      std::make_shared<ObstacleBasedOccupancyObservationPE>(),
      std::make_shared<EvenSPW>()
     ),
    -1, 1, resolution,
    -1, 1, resolution,
    0, 0, 0.1}; // TODO: rotation; FIXME: hand if delta is 0; accurate poses
  bfsm.subscribe(sssb);
  
  auto result = RobotPoseDelta{};
  auto start_time = std::chrono::high_resolution_clock::now();
  bfsm.process_scan(tscan, RobotPose{0.05, 0.05, 0}, map, result);
  auto end_time = std::chrono::high_resolution_clock::now();
  auto diff = std::chrono::duration<double>(end_time - start_time);
  std::cout << "BF: " << diff.count() << std::endl;
  
  GridMapToPgmDumber<GridMap>{"sss_map"}
    .on_map_update(*(sssb->map()));

  // TODO: generate testee search space map
}
