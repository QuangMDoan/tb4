#ifndef TB4_PERCEPTION_LAYER__IS_STOP_SIGN_NEARBY_HPP_
#define TB4_PERCEPTION_LAYER__IS_STOP_SIGN_NEARBY_HPP_

#include <mutex>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tb4_perception_layer/msg/semantic_obstacle_array.hpp"

namespace tb4_perception_layer
{

/**
 * BT condition node that returns SUCCESS when a "stop sign" is detected
 * within a configurable distance of the robot, FAILURE otherwise.
 */
class IsStopSignNearby : public BT::ConditionNode
{
public:
  IsStopSignNearby(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance_threshold", 2.0,
        "Max distance (m) to consider a stop sign nearby"),
      BT::InputPort<std::string>("semantic_topic", "/semantic_obstacles",
        "SemanticObstacleArray topic"),
      BT::InputPort<std::string>("global_frame", "map",
        "Global frame for distance computation"),
    };
  }

  BT::NodeStatus tick() override;

private:
  void obstacleCallback(
    const tb4_perception_layer::msg::SemanticObstacleArray::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<msg::SemanticObstacleArray>::SharedPtr sub_;
  std::mutex mutex_;
  bool stop_sign_nearby_{false};
  double distance_threshold_{2.0};
  bool initialized_{false};
};

}  // namespace tb4_perception_layer

#endif  // TB4_PERCEPTION_LAYER__IS_STOP_SIGN_NEARBY_HPP_
