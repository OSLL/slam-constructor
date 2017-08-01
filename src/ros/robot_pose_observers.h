#ifndef SLAM_CTOR_ROS_ROBOT_POSE_OBSERVERS_H
#define SLAM_CTOR_ROS_ROBOT_POSE_OBSERVERS_H

#include <fstream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "../core/states/world.h"

class RobotPoseTfPublisher : public WorldPoseObserver {
public: //methods
  RobotPoseTfPublisher(const std::string &tf_map_frame_id,
                       const std::string &tf_robot_frame_id) :
    _tf_map_frame_id{tf_map_frame_id},
    _tf_robot_frame_id{tf_robot_frame_id} {}

  void on_pose_update(const RobotPose &pose) override {
    auto map2robot = tf::Transform{
      tf::createQuaternionFromRPY(0, 0, pose.theta),
      tf::Vector3(pose.x, pose.y, 0.0)};

    auto transform = tf::StampedTransform{map2robot, ros::Time::now(),
                                          _tf_map_frame_id, _tf_robot_frame_id};
    _tf_brcst.sendTransform(transform);
  }

private: // fields
  std::string _tf_map_frame_id, _tf_robot_frame_id;
  tf::TransformBroadcaster _tf_brcst;
};

class RobotPoseTumTrajectoryDumper : public WorldPoseObserver {
public:
  RobotPoseTumTrajectoryDumper(const std::string &fname,
                               bool flush_every_entry = false)
    : _fstream{fname, std::ios::out}
    , _flush_every_entry{flush_every_entry} {}

  void on_pose_update(const RobotPose &pose) override {
    auto q = tf::createQuaternionFromRPY(0, 0, pose.theta);
    auto ts = ros::Time::now();
    _fstream << ts.sec << '.' << ts.nsec
             << ' ' << pose.x << ' ' << pose.y << ' ' << '0'
             << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w()
             << '\n';
    if (_flush_every_entry) {
      _fstream.flush();
    }
  }

private:
  std::ofstream _fstream;
  bool _flush_every_entry;
};

#endif
