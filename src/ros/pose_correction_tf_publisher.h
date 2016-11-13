#ifndef __POSE_CORRECTION_TF_PUBLISHER_H_INCLUDED
#define __POSE_CORRECTION_TF_PUBLISHER_H_INCLUDED

#include <thread>
#include <mutex>
#include <utility>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "topic_with_transform.h"
#include "../core/world.h"

template <typename ObservationType>
class PoseCorrectionTfPublisher : public TopicObserver<ObservationType>,
                                  public WorldPoseObserver {
public: //methods
  PoseCorrectionTfPublisher(bool is_async = false) :
    _last_odom2base{tf::createQuaternionFromRPY(0, 0, 0)},
    _map2odom{tf::createQuaternionFromRPY(0, 0, 0)} {
    if (!is_async) {
      return;
    }
    auto loop_f = &PoseCorrectionTfPublisher<ObservationType>::publishing_loop;
    _transform_publisher = std::move(std::thread{loop_f, this});
  }

  virtual void handle_transformed_msg(
    const boost::shared_ptr<ObservationType>,
    const tf::StampedTransform& t) override {
    // work with 2D pose
    _last_odom2base = t;
    _last_odom2base.getOrigin().setZ(0);
  }

  virtual void on_pose_update(const RobotPose &pose) override {
    tf::Transform base2map = tf::Transform{
      tf::createQuaternionFromRPY(0, 0, pose.theta),
      tf::Vector3(pose.x, pose.y, 0.0)}.inverse();
    _map2odom_mutex.lock();
    _map2odom = (_last_odom2base * base2map).inverse();
    _map2odom_mutex.unlock();
    publish_map_to_odom();
  }

private: //methods

  void publishing_loop() {
    ros::Rate rate(1.0 / 0.05);
    while (ros::ok()) {
      publish_map_to_odom();
      rate.sleep();
    }
  }

  void publish_map_to_odom() {
    _map2odom_mutex.lock();
    _tf_brcst.sendTransform(
      tf::StampedTransform{_map2odom, ros::Time::now(),
                           "map","odom_combined"});
    _map2odom_mutex.unlock();
  }
private: // fields
  tf::TransformBroadcaster _tf_brcst;
  tf::Transform _last_odom2base;
  tf::Transform _map2odom;
  std::mutex _map2odom_mutex;
  std::thread _transform_publisher;
};

#endif
