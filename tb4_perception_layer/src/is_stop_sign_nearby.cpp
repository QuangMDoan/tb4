#include "tb4_perception_layer/is_stop_sign_nearby.hpp"

#include <cmath>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "tf2_ros/buffer.h"

namespace tb4_perception_layer
{

IsStopSignNearby::IsStopSignNearby(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
}

BT::NodeStatus IsStopSignNearby::tick()
{
  if (!initialized_) {
    // Get the shared ROS node from the BT blackboard
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    double dist = 2.0;
    getInput("distance_threshold", dist);
    distance_threshold_ = dist;

    std::string topic = "/semantic_obstacles";
    getInput("semantic_topic", topic);

    sub_ = node_->create_subscription<msg::SemanticObstacleArray>(
      topic, 10,
      std::bind(&IsStopSignNearby::obstacleCallback, this,
        std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(),
      "IsStopSignNearby: listening on '%s', threshold=%.1f m",
      topic.c_str(), distance_threshold_);

    initialized_ = true;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (stop_sign_nearby_) {
    RCLCPP_WARN(node_->get_logger(),
      "Stop sign detected nearby! Triggering replan.");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsStopSignNearby::obstacleCallback(
  const tb4_perception_layer::msg::SemanticObstacleArray::SharedPtr msg)
{
  // Get robot position from TF via the blackboard
  double robot_x = 0.0, robot_y = 0.0;
  try {
    auto tf_buffer = config().blackboard->get<
      std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    if (tf_buffer) {
      std::string global_frame = "map";
      getInput("global_frame", global_frame);

      auto transform = tf_buffer->lookupTransform(
        global_frame, "base_link", tf2::TimePointZero);
      robot_x = transform.transform.translation.x;
      robot_y = transform.transform.translation.y;
    }
  } catch (...) {
    // If TF fails, use (0,0) — conservative fallback
  }

  std::lock_guard<std::mutex> lock(mutex_);
  stop_sign_nearby_ = false;

  for (const auto & obs : msg->obstacles) {
    if (obs.class_id != "stop sign") {
      continue;
    }
    double dx = obs.x - robot_x;
    double dy = obs.y - robot_y;
    double dist = std::hypot(dx, dy);
    if (dist <= distance_threshold_) {
      stop_sign_nearby_ = true;
      return;
    }
  }
}

}  // namespace tb4_perception_layer

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<tb4_perception_layer::IsStopSignNearby>(
    "IsStopSignNearby");
}
