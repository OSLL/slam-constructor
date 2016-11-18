#include <memory>
#include <cassert>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include "../core/sensor_data.h"
#include "../ros/topic_with_transform.h"
#include "../ros/laser_scan_observer.h"
#include "../ros/init_utils.h"
#include "gmapping_particle_filter.h"

GridMapParams init_grid_map_params() {
  GridMapParams params;
  ros::param::param<int>("~map_height_in_meters", params.height, 1000);
  ros::param::param<int>("~map_width_in_meters", params.width, 1000);
  ros::param::param<double>("~map_meters_per_cell", params.meters_per_cell,
                                                                         0.1);
  return params;
}

void init_constants_for_ros(double &ros_tf_buffer_size,
                            double &ros_map_rate,
                            int &ros_filter_queue,
                            int &ros_subscr_queue) {
  ros::param::param<double>("~ros_tf_buffer_duration",ros_tf_buffer_size,5.0);
  ros::param::param<double>("~ros_rviz_map_publishing_rate", ros_map_rate, 5.0);
  ros::param::param<int>("~ros_filter_queue_size",ros_filter_queue,1000);
  ros::param::param<int>("~ros_subscribers_queue_size",ros_subscr_queue,1000);
}

unsigned init_particles_nm() {
  int particles_nm;
  ros::param::param<int>("~rbpf_particles_nm", particles_nm, 30);
  assert(0 < particles_nm && "Particles number must be positive");
  return particles_nm;
}

using ObservT = sensor_msgs::LaserScan;
using GmappingMap = GmappingWorld::MapType;

int main(int argc, char** argv) {
  ros::init(argc, argv, "gMapping");

  GridMapParams map_params = init_grid_map_params();
  // TODO: setup CostEstimator and OccEstimator
  auto gcs = std::make_shared<GridCellStrategy>(
    std::make_shared<GmappingBaseCell>(),
    std::shared_ptr<ScanCostEstimator>(nullptr),
    std::shared_ptr<CellOccupancyEstimator>(nullptr)
  );
  auto slam = std::make_shared<GmappingParticleFilter>(
    gcs, map_params, init_particles_nm());

  ros::NodeHandle nh;
  double ros_map_publishing_rate, ros_tf_buffer_size;
  int ros_filter_queue, ros_subscr_queue;
  init_constants_for_ros(ros_tf_buffer_size, ros_map_publishing_rate,
                         ros_filter_queue, ros_subscr_queue);
  auto scan_provider = std::make_unique<TopicWithTransform<ObservT>>(
    nh, "laser_scan", tf_odom_frame_id(), ros_tf_buffer_size,
    ros_filter_queue, ros_subscr_queue
  );
  // TODO: setup scan skip policy via param
  auto scan_obs_pin = std::make_shared<LaserScanObserver>(
    slam, init_skip_exceeding_lsr());
  scan_provider->subscribe(scan_obs_pin);

  auto occup_grid_pub_pin = create_occupancy_grid_publisher<GmappingMap>(
    slam.get(), nh, ros_map_publishing_rate);

  auto pose_pub_pin = create_pose_correction_tf_publisher<ObservT, GmappingMap>(
    slam.get(), scan_provider.get());

  ros::spin();
}
