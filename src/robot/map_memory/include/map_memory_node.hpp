#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_memory_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    robot::MapMemoryCore map_memory_;

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    nav_msgs::msg::OccupancyGrid global_map_;
    bool has_global_map_ = false;
    
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    bool costmap_updated_ = false;
    bool should_update_map_ = false;
    const double distance_threshold_ = 1.5; // meters
    
    void initializeGlobalMap(const nav_msgs::msg::OccupancyGrid& costmap);
    void fuseCostmap(const nav_msgs::msg::OccupancyGrid& costmap);
};

#endif 
