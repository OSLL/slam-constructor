#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <memory>
#include <cassert>

#include "../core/sensor_data.h"
#include "../ros/topic_with_transform.h"
#include "../ros/laser_scan_observer.h"
#include "../ros/rviz_grid_viewer.h"
#include "gmapping_fascade.h"
#include <nav_msgs/OccupancyGrid.h>

unsigned init_particles_nm() {
  int particles_nm;
  ros::param::param<int>("~rbpf_particles_nm", particles_nm, 30);
  assert(0 < particles_nm && "Particles number must be positive");
  return particles_nm;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gMapping");

  // TODO: setup CostEstimator and OccEstimator
  std::shared_ptr<GridCellStrategy> gcs{new GridCellStrategy{
    std::make_shared<PlainGridCellFactory<GmappingBaseCell,
                                          GmappingCellValue>>(),
    std::shared_ptr<ScanCostEstimator>(nullptr),
    std::shared_ptr<CellOccupancyEstimator>(nullptr)
  }};
  std::shared_ptr<GmappingSlamFascade> slam{
    new GmappingSlamFascade(gcs, init_particles_nm())};

  ros::NodeHandle nh;
  TopicWithTransform<sensor_msgs::LaserScan> scan_provider{nh,
      "laser_scan", "odom_combined"};
  // TODO: setup scan skip policy via param
  std::shared_ptr<LaserScanObserver> scan_obs{new LaserScanObserver{slam}};
  scan_provider.subscribe(scan_obs);

  auto viewer = std::make_shared<RvizGridViewer<GmappingSlamFascade::MapType>>(
    nh.advertise<nav_msgs::OccupancyGrid>("/map", 5));
  slam->subscribe(viewer);

  ros::spin();
}
