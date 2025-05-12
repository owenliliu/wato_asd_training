#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();

private:
    // ROS subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // ROS publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State tracking
    enum class State { WAITING_FOR_GOAL, PLANNING, EXECUTING };
    State current_state_;

    // Current data
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::Pose robot_pose_;
    geometry_msgs::msg::Point current_goal_;
    bool has_map_ = false;
    bool has_goal_ = false;
    bool has_pose_ = false;

    // A* data structures
    struct Cell {
        int x;
        int y;
        bool operator==(const Cell& other) const {
            return x == other.x && y == other.y;
        }
    };

    struct CellHash {
        size_t operator()(const Cell& cell) const {
            return std::hash<int>()(cell.x) ^ std::hash<int>()(cell.y);
        }
    };

    // Callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    
    // Planning functions
    bool planPath(nav_msgs::msg::Path& path);
    bool isGoalReached() const;
    double heuristic(const Cell& a, const Cell& b) const;
    bool isValid(const Cell& cell) const;
    Cell worldToMap(double wx, double wy) const;
    std::pair<double, double> mapToWorld(int mx, int my) const;
};

#endif  // PLANNER_NODE_HPP_