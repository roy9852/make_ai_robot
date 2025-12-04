#include <memory>
#include <vector>
#include <fstream>
#include <sstream>
#include <chrono>
#include <limits>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class SimulatorNode : public rclcpp::Node
{
public:
  SimulatorNode()
  : Node("simulator_node")
  {
    // Declare parameters
    this->declare_parameter<bool>("use_gazebo", false);
    this->declare_parameter<std::string>("map_file", "");
    this->declare_parameter<double>("resolution", 1.0);
    this->declare_parameter<double>("start_x", 1.0);
    this->declare_parameter<double>("start_y", 1.0);
    
    use_gazebo_ = this->get_parameter("use_gazebo").as_bool();
    map_file_ = this->get_parameter("map_file").as_string();
    resolution_ = this->get_parameter("resolution").as_double();
    
    // Initialize robot position
    current_pose_.header.frame_id = "map";
    current_pose_.pose.position.x = this->get_parameter("start_x").as_double();
    current_pose_.pose.position.y = this->get_parameter("start_y").as_double();
    current_pose_.pose.position.z = 0.0;
    current_pose_.pose.orientation.w = 1.0;
    
    current_path_index_ = 0;
    has_path_ = false;
    
    // Publishers - only create if NOT using Gazebo mode
    if (!use_gazebo_) {
      map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
      pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/go1_pose", 10);
      robot_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/robot_marker", 10);
      
      RCLCPP_INFO(this->get_logger(), "Running in VIRTUAL SIMULATOR mode");
    } else {
      RCLCPP_INFO(this->get_logger(), "Running in GAZEBO mode (no map/pose publishing)");
    }
    
    // Subscriber
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10,
      std::bind(&SimulatorNode::pathCallback, this, std::placeholders::_1));
    
    // Load and publish map only if NOT using Gazebo mode
    if (!use_gazebo_) {
      if (loadMap(map_file_)) {
        publishMap();
        RCLCPP_INFO(this->get_logger(), "Map loaded and published: %dx%d", 
          map_width_, map_height_);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map from: %s", map_file_.c_str());
      }
      
      // Timers
      pose_timer_ = this->create_wall_timer(
        100ms, std::bind(&SimulatorNode::publishCurrentPose, this));
      
      movement_timer_ = this->create_wall_timer(
        100ms, std::bind(&SimulatorNode::moveRobot, this));
      
      map_timer_ = this->create_wall_timer(
        1000ms, std::bind(&SimulatorNode::publishMap, this));
      
      RCLCPP_INFO(this->get_logger(), "Robot starting at (%.2f, %.2f)", 
        current_pose_.pose.position.x, current_pose_.pose.position.y);
    }
    
    RCLCPP_INFO(this->get_logger(), "Simulator Node initialized");
  }

private:
  bool loadMap(const std::string& filename)
  {
    std::ifstream file(filename);
    if (!file.is_open()) {
      return false;
    }
    
    std::string line;
    map_data_.clear();
    
    while (std::getline(file, line)) {
      std::vector<int> row;
      std::istringstream iss(line);
      int value;
      
      while (iss >> value) {
        row.push_back(value);
      }
      
      if (!row.empty()) {
        map_data_.push_back(row);
      }
    }
    
    file.close();
    
    if (map_data_.empty()) {
      return false;
    }
    
    map_height_ = map_data_.size();
    map_width_ = map_data_[0].size();
    
    return true;
  }
  
  void publishMap()
  {
    if (use_gazebo_ || map_data_.empty()) {
      return;
    }
    
    nav_msgs::msg::OccupancyGrid map_msg;
    map_msg.header.stamp = this->now();
    map_msg.header.frame_id = "map";
    
    map_msg.info.resolution = resolution_;
    map_msg.info.width = map_width_;
    map_msg.info.height = map_height_;
    map_msg.info.origin.position.x = 0.0;
    map_msg.info.origin.position.y = 0.0;
    map_msg.info.origin.position.z = 0.0;
    map_msg.info.origin.orientation.w = 1.0;
    
    map_msg.data.resize(map_width_ * map_height_);
    
    for (int y = 0; y < map_height_; ++y) {
      for (int x = 0; x < map_width_; ++x) {
        int index = y * map_width_ + x;
        // Convert: 0 (free) -> 0, 1 (obstacle) -> 100
        map_msg.data[index] = map_data_[y][x] == 0 ? 0 : 100;
      }
    }
    
    map_pub_->publish(map_msg);
  }
  
  void publishCurrentPose()
  {
    if (use_gazebo_) {
      return;
    }
    
    current_pose_.header.stamp = this->now();
    pose_pub_->publish(current_pose_);
    
    // Also publish robot marker for visualization
    publishRobotMarker();
  }
  
  void publishRobotMarker()
  {
    if (use_gazebo_) {
      return;
    }
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "robot";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose = current_pose_.pose;
    marker.pose.position.z = 0.2;
    
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.4;
    
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    robot_marker_pub_->publish(marker);
  }
  
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      return;
    }
    
    // Only log if path changed significantly
    bool log_update = !has_path_ || 
                      (current_path_.poses.size() != msg->poses.size());
    
    current_path_ = *msg;
    
    // Start from index 1 (skip index 0 which is current position)
    // The path_planner always adds current position as first waypoint
    current_path_index_ = (current_path_.poses.size() > 1) ? 1 : 0;
    has_path_ = true;
    
    if (log_update) {
      RCLCPP_INFO(this->get_logger(), "Path received: %zu waypoints (starting from index %zu)", 
        current_path_.poses.size(), current_path_index_);
    }
  }
  
  void moveRobot()
  {
    if (!has_path_ || current_path_.poses.empty()) {
      return;
    }
    
    // Check if reached end of path
    if (current_path_index_ >= current_path_.poses.size()) {
      if (has_path_) {
        RCLCPP_INFO(this->get_logger(), "✓ Goal reached!");
        has_path_ = false;
      }
      return;
    }
    
    // Get target waypoint (next position in path)
    auto target = current_path_.poses[current_path_index_];
    
    // Move directly to the next waypoint in path
    current_pose_.pose.position.x = target.pose.position.x;
    current_pose_.pose.position.y = target.pose.position.y;
    
    // Update orientation to face movement direction
    if (current_path_index_ + 1 < current_path_.poses.size()) {
      auto next_target = current_path_.poses[current_path_index_ + 1];
      double dx = next_target.pose.position.x - current_pose_.pose.position.x;
      double dy = next_target.pose.position.y - current_pose_.pose.position.y;
      double yaw = std::atan2(dy, dx);
      current_pose_.pose.orientation.z = std::sin(yaw / 2.0);
      current_pose_.pose.orientation.w = std::cos(yaw / 2.0);
    }
    
    RCLCPP_INFO(this->get_logger(), "Moved to waypoint %zu: (%.2f, %.2f)", 
      current_path_index_, current_pose_.pose.position.x, current_pose_.pose.position.y);
    
    // Move to next waypoint
    current_path_index_++;
  }
  
  // ROS objects
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  rclcpp::TimerBase::SharedPtr movement_timer_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  
  // Map data
  std::vector<std::vector<int>> map_data_;
  int map_width_;
  int map_height_;
  
  // Robot state
  geometry_msgs::msg::PoseStamped current_pose_;
  nav_msgs::msg::Path current_path_;
  size_t current_path_index_;
  bool has_path_;
  
  // Parameters
  bool use_gazebo_;
  std::string map_file_;
  double resolution_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulatorNode>());
  rclcpp::shutdown();
  return 0;
}

