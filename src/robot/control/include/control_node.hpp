#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <optional>

class ControlNode : public rclcpp::Node {
public:
    ControlNode();

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;

    struct Params {
        double lookahead_distance = 0.5;
        double goal_tolerance = 0.1;
        double max_linear_speed = 0.8;
        double max_angular_speed = 0.5;
    } params_;

    void controlLoop();
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() const;
    geometry_msgs::msg::Twist calculateControl(const geometry_msgs::msg::PoseStamped& target) const;
    double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) const;
    double distanceBetween(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) const;
    bool isGoalReached() const;
};

#endif