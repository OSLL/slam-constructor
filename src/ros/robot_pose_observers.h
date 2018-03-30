#ifndef SLAM_CTOR_ROS_ROBOT_POSE_OBSERVERS_H
#define SLAM_CTOR_ROS_ROBOT_POSE_OBSERVERS_H

#include <fstream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "topic_with_transform.h"
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

    auto transform = tf::StampedTransform{map2robot, robot_pose_timestamp(),
                                          _tf_map_frame_id, _tf_robot_frame_id};
    _tf_brcst.sendTransform(transform);
  }

private:

  virtual ros::Time robot_pose_timestamp() {
    return ros::Time::now();
  }


private: // fields
  std::string _tf_map_frame_id, _tf_robot_frame_id;
  tf::TransformBroadcaster _tf_brcst;
};

template <typename ObsType>
class ObservationStampedRoboPoseTfPublisher : public RobotPoseTfPublisher
                                            , public TopicObserver<ObsType> {
public:
  ObservationStampedRoboPoseTfPublisher(const std::string &tf_map_frame_id,
                                        const std::string &tf_robot_frame_id)
    : RobotPoseTfPublisher{tf_map_frame_id, tf_robot_frame_id} {}

  void handle_transformed_msg(
    const boost::shared_ptr<ObsType> obs,
    const tf::StampedTransform&) override {

    _observation_time = ros::message_traits::TimeStamp<ObsType>::value(*obs);
  }

private:

  ros::Time robot_pose_timestamp() override {
    return _observation_time;
  }

private:
  ros::Time _observation_time;
};

class RobotPoseTumTrajectoryDumper : public WorldPoseObserver {
public:
  RobotPoseTumTrajectoryDumper(const std::string &fname,
                               bool flush_every_entry = false)
    : _fstream{fname, std::ios::out}
    , _flush_every_entry{flush_every_entry} {}

  void on_pose_update(const RobotPose &pose) override {
    log_robot_pose(ros::Time::now(), pose);
  }

  void log_robot_pose(const ros::Time &time,  const RobotPose &pose) {
    auto q = tf::createQuaternionFromRPY(0, 0, pose.theta);
    _fstream << time.sec << '.' << time.nsec
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
