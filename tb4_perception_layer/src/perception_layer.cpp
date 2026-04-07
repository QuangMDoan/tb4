// Build and Run:  
//     # cd /home/qd/turtlebot4_ws 
//     # source /opt/ros/jazzy/setup.bash 
//     # colcon build --packages-select tb4_perception_layer 

//     # source /home/qd/turtlebot4_ws/install/setup.bash 
//     # colcon build --packages-select tb4_perception_integration

//     # source /home/qd/turtlebot4_ws/install/setup.bash 
//     # ros2 interface show tb4_perception_layer/msg/SemanticObstacle 
//     # ros2 interface show tb4_perception_layer/msg/SemanticObstacleArray

//     # source /home/qd/turtlebot4_ws/install/setup.bash 
//     # ros2 pkg executables tb4_perception_integration

#include "tb4_perception_layer/perception_layer.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(tb4_perception_layer::PerceptionLayer, nav2_costmap_2d::Layer)

namespace tb4_perception_layer
{

PerceptionLayer::PerceptionLayer() {}

void PerceptionLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("PerceptionLayer: failed to lock node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("semantic_topic", rclcpp::ParameterValue("/semantic_obstacles"));
  declareParameter("obstacle_timeout", rclcpp::ParameterValue(3.0));
  declareParameter("default_cost", rclcpp::ParameterValue(254));
  declareParameter("default_radius", rclcpp::ParameterValue(0.3));
  declareParameter("class_names", rclcpp::ParameterValue(std::vector<std::string>()));
  declareParameter("class_costs", rclcpp::ParameterValue(std::vector<int64_t>()));
  declareParameter("class_radii", rclcpp::ParameterValue(std::vector<double>()));

  bool enabled = true;
  node->get_parameter(name_ + ".enabled", enabled);
  enabled_ = enabled;

  node->get_parameter(name_ + ".semantic_topic", semantic_topic_);
  node->get_parameter(name_ + ".obstacle_timeout", obstacle_timeout_);

  int default_cost_int = 254;
  node->get_parameter(name_ + ".default_cost", default_cost_int);
  default_cost_ = static_cast<unsigned char>(
    std::clamp(default_cost_int, 0, 254));

  node->get_parameter(name_ + ".default_radius", default_radius_);

  std::vector<std::string> class_names;
  std::vector<int64_t> class_costs;
  std::vector<double> class_radii;
  node->get_parameter(name_ + ".class_names", class_names);
  node->get_parameter(name_ + ".class_costs", class_costs);
  node->get_parameter(name_ + ".class_radii", class_radii);

  size_t n = class_names.size();
  for (size_t i = 0; i < n; ++i) {
    if (i < class_costs.size()) {
      class_cost_map_[class_names[i]] = static_cast<unsigned char>(
        std::clamp(static_cast<int>(class_costs[i]), 0, 254));
    }
    if (i < class_radii.size()) {
      class_radius_map_[class_names[i]] = class_radii[i];
    }
  }

  sub_ = node->create_subscription<msg::SemanticObstacleArray>(
    semantic_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PerceptionLayer::obstacleCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node->get_logger(),
    "PerceptionLayer initialized — topic: %s, timeout: %.1fs, %zu class mappings",
    semantic_topic_.c_str(), obstacle_timeout_, n);

  current_ = true;
}

void PerceptionLayer::obstacleCallback(
  const msg::SemanticObstacleArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  obstacles_.clear();

  for (const auto & obs : msg->obstacles) {
    CachedObstacle cached;
    cached.class_id = obs.class_id;
    cached.x = obs.x;
    cached.y = obs.y;
    cached.radius = (obs.radius > 0.0) ? obs.radius : radiusForClass(obs.class_id);
    cached.cost = (obs.cost > 0.0)
      ? static_cast<unsigned char>(std::clamp(static_cast<int>(obs.cost), 0, 254))
      : costForClass(obs.class_id);
    cached.stamp = rclcpp::Time(obs.header.stamp);
    obstacles_.push_back(cached);
  }
}

void PerceptionLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_) {return;}

  std::lock_guard<std::mutex> lock(mutex_);
  auto node = node_.lock();
  if (!node) {return;}

  rclcpp::Time now = node->get_clock()->now();

  for (const auto & obs : obstacles_) {
    double age = (now - obs.stamp).seconds();
    if (age > obstacle_timeout_) {continue;}

    double r = obs.radius;
    *min_x = std::min(*min_x, obs.x - r);
    *min_y = std::min(*min_y, obs.y - r);
    *max_x = std::max(*max_x, obs.x + r);
    *max_y = std::max(*max_y, obs.y + r);
  }
}

void PerceptionLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j,
  int max_i, int max_j)
{
  if (!enabled_) {return;}

  std::lock_guard<std::mutex> lock(mutex_);
  auto node = node_.lock();
  if (!node) {return;}

  rclcpp::Time now = node->get_clock()->now();
  double resolution = master_grid.getResolution();

  for (const auto & obs : obstacles_) {
    double age = (now - obs.stamp).seconds();
    if (age > obstacle_timeout_) {continue;}

    // Convert world position to map cell
    unsigned int mx, my;
    if (!master_grid.worldToMap(obs.x, obs.y, mx, my)) {
      continue;  // obstacle is outside the costmap
    }

    double r = obs.radius;
    int cell_radius = static_cast<int>(std::ceil(r / resolution));

    int cx = static_cast<int>(mx);
    int cy = static_cast<int>(my);

    for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
      for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
        int px = cx + dx;
        int py = cy + dy;

        if (px < min_i || px >= max_i || py < min_j || py >= max_j) {
          continue;
        }

        // Check circular footprint
        double dist = std::hypot(dx * resolution, dy * resolution);
        if (dist > r) {continue;}

        unsigned char old_cost = master_grid.getCost(
          static_cast<unsigned int>(px), static_cast<unsigned int>(py));
        if (obs.cost > old_cost) {
          master_grid.setCost(
            static_cast<unsigned int>(px),
            static_cast<unsigned int>(py),
            obs.cost);
        }
      }
    }
  }
}

void PerceptionLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  obstacles_.clear();
  current_ = false;
}

unsigned char PerceptionLayer::costForClass(const std::string & class_id) const
{
  auto it = class_cost_map_.find(class_id);
  return (it != class_cost_map_.end()) ? it->second : default_cost_;
}

double PerceptionLayer::radiusForClass(const std::string & class_id) const
{
  auto it = class_radius_map_.find(class_id);
  return (it != class_radius_map_.end()) ? it->second : default_radius_;
}

}  // namespace tb4_perception_layer
