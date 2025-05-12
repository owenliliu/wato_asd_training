#include "control_node.hpp"

ControlNode::ControlNode() : Node("pure_pursuit_controller") {
    // Parameter declaration
    this->declare_parameter("lookahead_distance", params_.lookahead_distance);
    this->declare_parameter("goal_tolerance", params_.goal_tolerance);
    this->declare_parameter("max_linear_speed", params_.max_linear_speed);
    this->declare_parameter("max_angular_speed", params_.max_angular_speed);

    // Parameter retrieval
    params_.lookahead_distance = this->get_parameter("lookahead_distance").as_double();
    params_.goal_tolerance = this->get_parameter("goal_tolerance").as_double();
    params_.max_linear_speed = this->get_parameter("max_linear_speed").as_double();
    params_.max_angular_speed = this->get_parameter("max_angular_speed").as_double();

    // ROS setup
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
            current_path_ = msg;
        });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            current_odom_ = msg;
        });

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    control_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),  // 10Hz
        std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::controlLoop() {
    if (!current_path_ || !current_odom_ || current_path_->poses.empty()) {
        return;
    }

    if (isGoalReached()) {
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
        RCLCPP_INFO(get_logger(), "Goal reached!");
        return;
    }

    if (auto lookahead = findLookaheadPoint()) {
        cmd_vel_pub_->publish(calculateControl(*lookahead));
    }
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() const {
    const auto& robot_pos = current_odom_->pose.pose.position;
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();

    // Find closest point
    for (size_t i = 0; i < current_path_->poses.size(); ++i) {
        double dist = distanceBetween(robot_pos, current_path_->poses[i].pose.position);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    // Find first point beyond lookahead distance
    for (size_t i = closest_idx; i < current_path_->poses.size(); ++i) {
        double dist = distanceBetween(robot_pos, current_path_->poses[i].pose.position);
        if (dist >= params_.lookahead_distance) {
            return current_path_->poses[i];
        }
    }

    return current_path_->poses.back();
}

geometry_msgs::msg::Twist ControlNode::calculateControl(const geometry_msgs::msg::PoseStamped& target) const {
    geometry_msgs::msg::Twist cmd;
    const auto& robot_pos = current_odom_->pose.pose.position;
    double robot_yaw = quaternionToYaw(current_odom_->pose.pose.orientation);

    // Transform to robot frame
    double dx = target.pose.position.x - robot_pos.x;
    double dy = target.pose.position.y - robot_pos.y;
    double target_x = cos(robot_yaw) * dx + sin(robot_yaw) * dy;
    double target_y = -sin(robot_yaw) * dx + cos(robot_yaw) * dy;

    // Pure Pursuit control law
    double curvature = 2.0 * target_y / (target_x * target_x + target_y * target_y);
    
    cmd.linear.x = params_.max_linear_speed;
    cmd.angular.z = std::clamp(curvature * cmd.linear.x, 
                              -params_.max_angular_speed, 
                              params_.max_angular_speed);

    return cmd;
}

double ControlNode::quaternionToYaw(const geometry_msgs::msg::Quaternion& q) const {
    // Simplified conversion (accurate for 2D navigation)
    return atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
}

double ControlNode::distanceBetween(const geometry_msgs::msg::Point& p1, 
                                  const geometry_msgs::msg::Point& p2) const {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx*dx + dy*dy);
}

bool ControlNode::isGoalReached() const {
    if (current_path_->poses.empty()) return false;
    return distanceBetween(current_odom_->pose.pose.position,
                          current_path_->poses.back().pose.position) < params_.goal_tolerance;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}