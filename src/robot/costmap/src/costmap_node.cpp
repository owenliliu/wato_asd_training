#include <chrono>
#include <memory>
#include <vector>
#include <iostream> 
#include <cmath> // for sin, cos, floor
#include "costmap_node.hpp"

using std::vector;

// Type alias for readability
using Costmap = vector<vector<int>>;

CostmapNode::CostmapNode(double width_meters, double height_meters, double resolution) : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())), resolution_(resolution) {
  // Initialize the constructs and their parameters
  cost_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  // timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
  width_cells_ = static_cast<int>(width_meters / resolution_);
  height_cells_ = static_cast<int>(height_meters / resolution_);
  origin_x_ = - (width_cells_ * resolution_) / 2.0;
  origin_y_ = - (height_cells_ * resolution_) / 2.0;

    // Resize the grid with default value 0
    grid_.resize(height_cells_, std::vector<int>(width_cells_, 0));

}
 
// // Define the timer to publish a message every 500ms
// void CostmapNode::publishMessage() {
//   auto message = std_msgs::msg::String();
//   message.data = "Hello, ROS 2!";
//   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//   string_pub_->publish(message);
// }

 void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Step 1: Initialize costmap
    initializeCostmap();
 
    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            // Calculate grid coordinates
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }
 
    // Step 3: Inflate obstacles
    inflateObstacles(1, 100);
 
    // Step 4: Publish costmap
    publishCostmap();
}

void CostmapNode::initializeCostmap() {
    for (int y = 0; y < height_cells_; ++y) {
        for (int x = 0; x < width_cells_; ++x) {
            grid_[y][x] = 0;
        }
    }
}

void CostmapNode::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
    
    double x_world = range * std::cos(angle);
    double y_world = range * std::sin(angle);

    
    x_grid = static_cast<int>((x_world - origin_x_) / resolution_);
    y_grid = static_cast<int>((y_world - origin_y_) / resolution_);
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
    if (y_grid >= 0 && y_grid < height_cells_ &&
        x_grid >= 0 && x_grid < width_cells_) {
        grid_[y_grid][x_grid] = 100; 
    }
}

void CostmapNode::inflateObstacles(double inflation_radius, int max_cost) {
    int inflation_cells = static_cast<int>(std::ceil(inflation_radius / resolution_));

    std::vector<std::vector<int>> inflated_grid = grid_;

    for (int y = 0; y < height_cells_; ++y) {
        for (int x = 0; x < width_cells_; ++x) {
            if (grid_[y][x] == 100) { 
                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                        int ny = y + dy;
                        int nx = x + dx;

                
                        if (nx >= 0 && nx < width_cells_ && ny >= 0 && ny < height_cells_) {
                         
                            if (dx == 0 && dy == 0) continue;

                            double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                            if (distance > inflation_radius) continue;

                         
                            int cost = static_cast<int>(max_cost * (1.0 - distance / inflation_radius));
                         
                            if (cost > inflated_grid[ny][nx] && grid_[ny][nx] != 100) {
                                inflated_grid[ny][nx] = cost;
                            }
                        }
                    }
                }
            }
        }
    }

    grid_ = inflated_grid;
}

void CostmapNode::publishCostmap() {
    nav_msgs::msg::OccupancyGrid occupancy_msg;

    occupancy_msg.header.stamp = rclcpp::Clock().now();
    occupancy_msg.header.frame_id = "robot/chassis/lidar";

    occupancy_msg.info.resolution = resolution_;
    occupancy_msg.info.width = width_cells_;
    occupancy_msg.info.height = height_cells_;
    occupancy_msg.info.origin.position.x = origin_x_;
    occupancy_msg.info.origin.position.y = origin_y_;
    occupancy_msg.info.origin.position.z = 0.0;
    occupancy_msg.info.origin.orientation.w = 1.0;

    occupancy_msg.data.resize(width_cells_ * height_cells_);
    for (int y = 0; y < height_cells_; ++y) {
        for (int x = 0; x < width_cells_; ++x) {
            int index = y * width_cells_ + x;
            occupancy_msg.data[index] = static_cast<int8_t>(grid_[y][x]);
        }
    }


    cost_map_pub_->publish(occupancy_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>(50,50,0.1));
  rclcpp::shutdown();
  return 0;
}