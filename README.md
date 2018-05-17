# SLAM Constructor Framework

The SLAM constructor framework provides common functionality and classes that may be used to create custom SLAM algorithms (currently only 2D laser scan-based methods are supported). It also includes implementation of several SLAM algorithms: [tinySLAM](https://ieeexplore.ieee.org/document/5707402/), [vinySLAM](https://ieeexplore.ieee.org/document/8206595/) and [GMapping](https://ieeexplore.ieee.org/document/4084563/). They show an example of the framework usage and can be used as a base of a new SLAM algorithm.

## Hardware Requirements

Current implementation requires odometry data and laser scans to be provided by the ROS topics (see [Subscribed topics](#subscribed-topics)). It also supposes that a laser scanner is fixed in (0, 0) of a robot and mounted horizontally.

## Examples

### Provided algorithms usage

Each algorithm is supplied with [launch](http://wiki.ros.org/roslaunch)-files for the [MIT Stata Center](http://projects.csail.mit.edu/stata/) and [PR2 – Willow Garage](http://google-cartographer-ros.readthedocs.io/en/latest/data.html#pr2-willow-garage) datasets that give the idea of how to launch algorithms. The `_mit_` launch-files can be used in general case if data provided by a dataset do not require any preprocessing. For example, tinySLAM can be launched in the following way:

>`roslaunch slam_constructor tiny_mit_run.launch path:=[path to dataset]`

In order to run an algorithm on data received in real time you can remove a dataset player node from a launch file, but make sure that sensor data are provided through [subscribed topics](#subscribed-topics).

### Single-hypothesis SLAM workflow

The following diagram shows the expected workflow of the general single-hypothesis SLAM algorithm:

![workflow](https://raw.githubusercontent.com/OSLL/slam-constructor/master/docs/1h-SLAM.png)

## Parameters

The following parameters are common for all provided SLAM algorithms:

#### ROS-specific parameters

* `~in/lscan2D/ros/topic/name` (*string*, default: `/base_scan`) – the laser scan topic name
* `~in/odometry/ros/tf/odom_frame_id` (*string*, default: `odom_combined`) – the odometry tf frame id
* `~ros/tf/map_frame_id` (*string*, default: `map`) – the map tf frame id
* `~ros/tf/robot_pose_frame_id` (*string*, default: `robot_pose`) – the output robot pose frame id
* `~ros/tf/async_correction` (*bool*, default: `false`) – set to `true` to publish the `map` → `odom` transformation asynchronous (in a different thread)
* tf listener parameters (refer to [`tf::MessageFilter`](http://docs.ros.org/diamondback/api/tf/html/c++/classtf_1_1MessageFilter.html#ab85a2ee3e8fdd13ff19ee966bd73888d) documentation):
  * `~ros/subscribers_queue_size` (*int*, default: `1000`) – the queue size for [`message_filters::Subscriber`](http://docs.ros.org/jade/api/message_filters/html/c++/classmessage__filters_1_1Subscriber.html)
  * `~ros/tf/buffer_duration` (*double*, default: `5.0`) – the buffer size in seconds for [`tf::TranformListner`](http://docs.ros.org/diamondback/api/tf/html/c++/classtf_1_1TransformListener.html)
  * `~ros/filter_queue_size` (*int*, default: `1000`) – the filter queue size
* `~ros/rviz/map_publishing_rate` (*double*, default: `5.0`) – the map publishing interval in seconds
* `~ros/skip_exceeding_lsr_vals` (*bool*, default: `false`) – set to `true` to skip laser scan outliers or to `false` to insert such measurements in a map as a free space

#### General SLAM parameters

* `~slam/map/height_in_meters` (*double*, default: `10.0`) – the map height in meters
* `~slam/map/width_in_meters` (*double*, default: `10.0`) – the map width in meters
* `~slam/map/meters_per_cell` (*double*, default: `0.1`) – the map resolution in meters
* `~slam/performance/use_trig_cache` (*bool*, default: `false`) – use trigonometry cache to speed up scan point operations
* cell occupancy estimator parameters:
  * `~slam/occupancy_estimator/type` (*string*, default: `const`) – the type of the cell occupancy estimator:
    * `const` – a cell is considered occupied if a laser scan point is inside the cell
    * `area` – a cell occupancy estimation is based on how a laser beam splits the cell
  * `~slam/occupancy_estimator/base_occupied/prob` (*double*, default: `0.95`) – the cell's probability to be occupied if a scan point is inside the cell
  * `~slam/occupancy_estimator/base_occupied/qual` (*double*, default: `1.0`) 
  * `~slam/occupancy_estimator/base_empty/prob` (*double*, default: `0.01`) – the cell's probability to be empty if a laser beam passes through the cell
  * `~slam/occupancy_estimator/base_empty/qual` (*double*, default: `1.0`)
* `~slam/mapping/blur` (*double*, default: `0.0`) – blur obstacles along the direction of a laser beam in the specified range (in meters)
* `~slam/mapping/max_range` (*double*, default: `<infinity>`) – maximum valid range for laser scan measurements when updating a map with a new laser scan

#### Scan-matcher parameters

* `~slam/scmtch/type` (*string*, default: `<undefined>`) – the scan matcher (SM) to be used. Currently the following scan matchers are supported:
  * `MC` – Monte-Carlo scan matcher. Parameters:
    * `~slam/scmtch/MC/attempts_limit` (*unsigned int*, default: `100`) – the maximum number of SM iterations 
    * `~slam/scmtch/MC/seed` (*int*, default: `<random>`) – the seed value for RNG
    * `~slam/scmtch/MC/dispersion/translation` (*double*, default: `0.2`) 
    * `~slam/scmtch/MC/dispersion/rotation` (*double*, default: `0.1`)
    * `~slam/scmtch/MC/dispersion/failed_attempts_limit` (*unsigned int*, default: `20`)
  * `HC` – hill climbing scan matcher. Parameters:
    * `~slam/scmtch/HC/distortion/translation` (*double*, default: `0.1`) – the initial step size in x/y direction in meters
    * `~slam/scmtch/HC/distortion/rotation` (*double*, default: `0.1`) – the initial step size in th direction in radians
    * `~slam/scmtch/HC/distortion/failed_attempts_limit` (*unsigned int*, default: `6`)
  * `BF` – brute-force scan matcher. Searches for the best scan position around the initial guess. Parameters set the search ranges and search steps for each coordinate (in meters for translation, radians for rotation):
    * `~slam/scmtch/BF/[x, y, t]/from` (*double*, default: `-0.5`, `-0.5`, `-5°`)
    * `~slam/scmtch/BF/[x, y, t]/to` (*double*, default: `0.5`, `0.5`, `5°`)
    * `~slam/scmtch/BF/[x, y, t]/step` (*double*, default: `0.1`, `0.1`, `1°`)
* `~slam/scmtch/spe/type` (*string*, default: `<undefined>`) – the scan probability estimator type. Currently only `wmpp` (weighted mean point probability) is supported. Parameters:
  * `~slam/scmtch/spe/wmpp/weighting/type` (*string*, default: `<undefined>`)
    * `even` – each point in a scan has the equal weight
    * `viny` – weighting scheme used in vinySLAM (see the [paper](https://ieeexplore.ieee.org/document/8206595/))
    * `ahr` – weighting scheme based on angle histograms
  * `~slam/scmtch/spe/wmpp/sp_skip_rate` (*unsigned int*, default: `0`) – skip every *n*-th point in scan
  * `~slam/scmtch/spe/wmpp/sp_max_usable_range` (*double*, default: `-1.0`) – max valid scan measurement range used in scan probability estimation
* `~slam/scmtch/oope/type` (*string*, default: `obstacle`) – the occupancy observation probability estimator type. Currently the following types are supported:
  * `obstacle`
  * `max`
  * `mean`
  * `overlap`

#### tinySLAM parameters

* `~slam/cell/type` (*string*, default: `base`) – the cell model type. Accepted values:
  * `base` – the cell model used in original [tinySLAM](https://ieeexplore.ieee.org/document/5707402/)
  * `avg` – the cell model proposed in [this paper](https://ieeexplore.ieee.org/document/7849536/)

#### GMapping parameters

GMapping has the following additional parameters (note that `~slam/scmtch/oope/type` shouldn't be provided or **must** be `custom`):

* `~slam/particles/number` (*unsigned int*, default: `30`)
* `~slam/particles/sample/xy/mean` (*double*, default: `0.0`)
* `~slam/particles/sample/xy/sigma` (*double*, default: `0.1`)
* `~slam/particles/sample/theta/mean` (*double*, default: `0.0`)
* `~slam/particles/sample/theta/sigma` (*double*, default: `0.03`)
* `~slam/particles/sm_delta_lim/xy/min` (*double*, default: `0.6`)
* `~slam/particles/sm_delta_lim/xy/max` (*double*, default: `0.8`)
* `~slam/particles/sm_delta_lim/theta/min` (*double*, default: `0.3`)
* `~slam/particles/sm_delta_lim/theta/max` (*double*, default: `0.4`)
* `~slam/scmtch/oope/custom/fullness_threshold` (*double*, default: `0.1`)
* `~slam/scmtch/oope/custom/window_size` (*unsigned int*, default: `1`)

## Subscribed topics

* `/tf` ([tf/tfMessage](http://docs.ros.org/api/tf/html/msg/tfMessage.html)) – should provide [required transforms](#required-tf-transforms).
* `/base_scan` ([sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)) – laser scans that should be used to create a map

## Published topics

* map ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html)) – the map data are periodically published to this topic

## Required tf transforms

* `<the frame attached to incoming scans>` (extracted from scan messages) → `odom_frame_id` (configurable, default: `odom_combined`) - usually provided by the odometry system

## Provided tf transforms

* The current estimate of the robot's pose within the map frame. Frame ids can be [configured](#ros-specific-parameters).
  * `map_frame_id` → `odom_frame_id`
  * `map_frame_id` → `robot_pose_frame_id`

## Offline mode

The package provides the utility `lslam2d_bag_runner` to launch algorithms in the offline mode to process datasets in [BAG](http://wiki.ros.org/rosbag) format.

#### Usage

```
lslam2d_bag_runner <slam type> <bag file>
                   [-v] [-t <traj file>] [-m <map file>]
                   [-p <properties file>]
```

#### Parameters

* `<slam type>` – `viny`, `tiny` or `gmapping`
* `<bag file>` – the path to a dataset
* `-v` – enable verbose output
* `-t <traj file>` – save a robot trajectory to `traj file` in [TUM](https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats) format
* `-m <map file>` – save an output map to `map file` in PGM format
* `-p <properties file>` – the path to a SLAM configuration file in `key=value` format. Example configurations can be found [here](https://github.com/OSLL/slam-constructor/tree/master/config/bag_runner)

## Contributors

* Artur Huletski
* Dmitriy Kartashov
* Kirill Krinkin

Copyright (c) 2018 JetBrains Research, Mobile Robot Algorithms Laboratory

