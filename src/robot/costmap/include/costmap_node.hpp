#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
 
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode(double width_meters, double height_meters, double resolution);
    
    // Place callback function here
    void initializeCostmap();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles(double inflation_radius, int max_cost = 100);
    void publishCostmap();
  private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::vector<int>> grid_; // 2D costmap grid
    double resolution_;                  // meters per cell
    int width_cells_;                    // number of columns
    int height_cells_; 
    double origin_x_ = 0.0; // World x origin of the map (meters)
    double origin_y_ = 0.0; // World y origin of the map (meters)
};
 
#endif 