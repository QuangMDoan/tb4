#ifndef TB4_PERCEPTION_LAYER__PERCEPTION_LAYER_HPP_
#define TB4_PERCEPTION_LAYER__PERCEPTION_LAYER_HPP_

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tb4_perception_layer/msg/semantic_obstacle_array.hpp"

namespace tb4_perception_layer
{

class PerceptionLayer : public nav2_costmap_2d::Layer
{
public:
  PerceptionLayer();

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j,
    int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return true;}

private:
  void obstacleCallback(
    const tb4_perception_layer::msg::SemanticObstacleArray::SharedPtr msg);

  struct CachedObstacle
  {
    std::string class_id;
    double x;
    double y;
    double radius;
    unsigned char cost;
    rclcpp::Time stamp;
  };

  unsigned char costForClass(const std::string & class_id) const;
  double radiusForClass(const std::string & class_id) const;

  rclcpp::Subscription<msg::SemanticObstacleArray>::SharedPtr sub_;
  std::mutex mutex_;
  std::vector<CachedObstacle> obstacles_;

  // Parameters
  std::string semantic_topic_;
  double obstacle_timeout_;
  unsigned char default_cost_;
  double default_radius_;
  std::unordered_map<std::string, unsigned char> class_cost_map_;
  std::unordered_map<std::string, double> class_radius_map_;
};

}  // namespace tb4_perception_layer

#endif  // TB4_PERCEPTION_LAYER__PERCEPTION_LAYER_HPP_
