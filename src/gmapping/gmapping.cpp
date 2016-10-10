#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <memory>

#include "../core/sensor_data.h"
#include "../ros/topic_with_transform.h"
#include "../ros/laser_scan_observer.h"
#include "../ros/rviz_grid_viewer.h"
#include "gmapping_fascade.h"
#include <nav_msgs/OccupancyGrid.h>

/**
 * Initializes constants for map
 * \return The structure contains requied paramteres
 */
GridMapParams init_grid_map_params() {
  GridMapParams params;
  ros::param::param<double>("~map_height_in_meters", params.height, 20);
  ros::param::param<double>("~map_width_in_meters", params.width, 20);
  ros::param::param<double>("~map_meters_per_cell", params.meters_per_cell,
                                                                         0.1);
  return params;
}

/**
 * Initializes constants for ros utils
 * \return Requied parameters
 */
void init_constants_for_ros(double &ros_tf_buffer_size,
                            double &ros_map_rate,
                            int &ros_filter_queue,
                            int &ros_subscr_queue) {
  ros::param::param<double>("~ros_tf_buffer_duration",ros_tf_buffer_size,5.0);
  ros::param::param<double>("~ros_rviz_map_publishing_rate", ros_map_rate, 5.0);
  ros::param::param<int>("~ros_filter_queue_size",ros_filter_queue,1000);
  ros::param::param<int>("~ros_subscribers_queue_size",ros_subscr_queue,1000);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gMapping");

  // TODO: setup CostEstimator and OccEstimator
  GridMapParams grid_map_params = init_grid_map_params();
  std::shared_ptr<GridCellStrategy> gcs(new GridCellStrategy(
    std::make_shared<PlainGridCellFactory<GmappingBaseCell>>(),
    std::shared_ptr<ScanCostEstimator>(nullptr),
    std::shared_ptr<CellOccupancyEstimator>(nullptr)
  ));
  std::shared_ptr<GmappingSlamFascade> slam{new GmappingSlamFascade(gcs,
    grid_map_params)};

  double ros_map_publishing_rate, ros_tf_buffer_size;
  int ros_filter_queue, ros_subscr_queue;
  init_constants_for_ros(ros_tf_buffer_size, ros_map_publishing_rate,
                         ros_filter_queue, ros_subscr_queue);
  ros::NodeHandle nh;
  TopicWithTransform<sensor_msgs::LaserScan> scan_provider(nh,
      "laser_scan", "odom_combined", ros_tf_buffer_size,
      ros_filter_queue, ros_subscr_queue);
  // TODO: setup scan skip policy via param
  std::shared_ptr<LaserScanObserver> scan_obs{new LaserScanObserver{slam}};
  scan_provider.subscribe(scan_obs);

  std::shared_ptr<RvizGridViewer> viewer(
    new RvizGridViewer(nh.advertise<nav_msgs::OccupancyGrid>("/map", 5),
                       ros_map_publishing_rate));
  slam->subscribe(viewer);

  ros::spin();
}
