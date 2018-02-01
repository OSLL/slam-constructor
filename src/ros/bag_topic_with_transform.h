#ifndef SLAM_CTOR_ROS_BAG_TOPIC_WITH_TRANSFORM_H
#define SLAM_CTOR_ROS_BAG_TOPIC_WITH_TRANSFORM_H

#include <vector>
#include <string>
#include <tuple>
#include <queue>
#include <unordered_map>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>

#include <boost/shared_ptr.hpp>

template <typename MsgType>
class BagTopicWithTransform {
protected:
  using vstr = std::vector<std::string>;
  using Transforms = std::unordered_map<std::string, std::vector<std::string>>;
public:
  BagTopicWithTransform(const std::string &bag_fname,
                        const std::string &topic_name,
                        const std::string &tf_topic_name,
                        const std::string &target_frame)
    : _topic_name{topic_name}, _tf_topic_name{tf_topic_name}
    , _target_frame{target_frame}
    , _bag{bag_fname}
    , _view{_bag, rosbag::TopicQuery{vstr{_topic_name, _tf_topic_name}}}
    , _view_iter{_view.begin()}
    , _tf_cache{true, ros::Duration{1000.0}} {}

  ~BagTopicWithTransform() {
    _bag.close();
  }

  BagTopicWithTransform(const BagTopicWithTransform&) = delete;
  BagTopicWithTransform(BagTopicWithTransform&&) = delete;
  BagTopicWithTransform& operator=(const BagTopicWithTransform&) = delete;
  BagTopicWithTransform& operator=(BagTopicWithTransform&&) = delete;

  void set_tf_ignores(const Transforms &tf_ignores) {
    _tf_ignores = tf_ignores;
  }

  bool extract_next_msg() {
    while (_view_iter != _view.end()) {
      auto ros_msg = *(_view_iter++);
      auto msg_topic = ros_msg.getTopic();
      if (msg_topic == _topic_name) {
        _msg_cache.push(ros_msg.instantiate<MsgType>());
      } else if (msg_topic == _tf_topic_name) {
        auto tf_msg = ros_msg.instantiate<tf::tfMessage>();
        for(auto t : tf_msg->transforms) {
          if (should_skip_tf_transform(t)) {
            continue;
          }
          tf::StampedTransform st;
          tf::transformStampedMsgToTF(t, st);
          _tf_cache.setTransform(st);
        }
      }
      if (has_synced_msg()) { return true; }
    }

    return has_synced_msg();
  }

  const auto &msg() const { return _msg;}
  const auto &transform() const { return _transform;}
  auto timestamp() const {
    return ros::message_traits::TimeStamp<MsgType>::value(*_msg);
  }

private:

  bool should_skip_tf_transform(const geometry_msgs::TransformStamped &t) {
    auto from = t.header.frame_id, to = t.child_frame_id;
    if (_tf_ignores.find(from) == _tf_ignores.end()) {
      return false;
    }
    auto skip_tos = _tf_ignores.at(from);
    return std::find(skip_tos.begin(), skip_tos.end(), to) != skip_tos.end();
  }

  bool has_synced_msg() {
    if (_msg_cache.empty()) { return false;}

    auto msg = _msg_cache.front();

    using namespace ros::message_traits;
    auto frame_id = FrameId<MsgType>::value(*msg);
    auto time = TimeStamp<MsgType>::value(*msg);
    tf::StampedTransform transform;
    try {
      _tf_cache.lookupTransform(_target_frame, frame_id, time, transform);
    } catch (const tf::ExtrapolationException &e) {
      auto reason = std::string{e.what()};
      if (reason.find("extrapolation into the future") != std::string::npos) {
        // ok, wait for tf transforms
        return false;
      }

      std::cout << "[WARN][BagTopic] ";
      if (reason.find("past") != std::string::npos) {
         _msg_cache.pop();
         std::cout << "Scan dropped. ";
      }
      std::cout << reason << std::endl;;
      return false;
    }

    _msg_cache.pop();
    _msg = std::move(msg);
    _transform = std::move(transform);
    return true;
  }

private:
  std::string _topic_name, _tf_topic_name, _target_frame;
  rosbag::Bag _bag;
  rosbag::View _view;
  rosbag::View::const_iterator _view_iter;

  Transforms _tf_ignores;
  tf::Transformer _tf_cache;
  std::queue<boost::shared_ptr<MsgType>> _msg_cache;

  boost::shared_ptr<MsgType> _msg;
  tf::StampedTransform _transform;
};

#endif
