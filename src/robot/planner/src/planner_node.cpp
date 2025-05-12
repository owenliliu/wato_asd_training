#include "planner_node.hpp"

PlannerNode::PlannerNode() 
    : Node("planner_node"),
      current_state_(State::WAITING_FOR_GOAL)
{
    // Initialize subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
        
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
        
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
    
    // Initialize publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/path", rclcpp::QoS(10).transient_local());
    
    // Initialize timer (500ms check interval)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    current_map_ = *msg;
    has_map_ = true;
    if (current_state_ == State::EXECUTING) {
        current_state_ = State::PLANNING;
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    current_goal_ = msg->point;
    has_goal_ = true;
    current_state_ = State::PLANNING;
    RCLCPP_INFO(this->get_logger(), "New goal received at (%.2f, %.2f)", 
               msg->point.x, msg->point.y);
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_pose_ = msg->pose.pose;
    has_pose_ = true;
}

void PlannerNode::timerCallback()
{
    switch (current_state_) {
        case State::WAITING_FOR_GOAL:
            break;
            
        case State::PLANNING:
            if (has_map_ && has_goal_ && has_pose_) {
                nav_msgs::msg::Path path;
                if (planPath(path)) {
                    path_pub_->publish(path);
                    current_state_ = State::EXECUTING;
                    RCLCPP_INFO(this->get_logger(), "New path published!");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Path planning failed!");
                    current_state_ = State::WAITING_FOR_GOAL;
                }
            }
            break;
            
        case State::EXECUTING:
            if (isGoalReached()) {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                current_state_ = State::WAITING_FOR_GOAL;
            }
            break;
    }
}

bool PlannerNode::planPath(nav_msgs::msg::Path& path) {
    if (!has_map_ || !has_goal_ || !has_pose_) {
        return false;
    }

    // Convert start and goal to map coordinates
    Cell start = worldToMap(robot_pose_.position.x, robot_pose_.position.y);
    Cell goal = worldToMap(current_goal_.x, current_goal_.y);

    // Validate cells
    if (!isValid(start) || !isValid(goal)) {
        RCLCPP_WARN(this->get_logger(), "Start or goal position is invalid!");
        return false;
    }

    // A* implementation
    struct Node {
        Cell cell;
        double g_score;
        double f_score;
        Cell came_from;
    };

    auto cmp = [](const Node& a, const Node& b) { return a.f_score > b.f_score; };
    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open_set(cmp);
    std::unordered_map<Cell, Node, CellHash> all_nodes;

    // Initialize start node
    Node start_node{start, 0, heuristic(start, goal), {-1, -1}};
    open_set.push(start_node);
    all_nodes[start] = start_node;

    // Possible movements (8-connected grid)
    const std::vector<Cell> directions = {
        {-1, -1}, {-1, 0}, {-1, 1},
        {0, -1},          {0, 1},
        {1, -1},  {1, 0}, {1, 1}
    };

    while (!open_set.empty()) {
        Node current = open_set.top();
        open_set.pop();

        // Check if we reached the goal
        if (current.cell == goal) {
            // Reconstruct path
            std::vector<Cell> path_cells;
            Cell node = current.cell;
            while (node.x != -1 && node.y != -1) {
                path_cells.push_back(node);
                node = all_nodes[node].came_from;
            }
            std::reverse(path_cells.begin(), path_cells.end());

            // Convert to Path message
            path.header.stamp = this->now();
            path.header.frame_id = "sim_world";
            path.poses.reserve(path_cells.size());

            for (const auto& cell : path_cells) {
                auto [wx, wy] = mapToWorld(cell.x, cell.y);
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = wx;
                pose.pose.position.y = wy;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }

            return true;
        }

        // Explore neighbors
        for (const auto& dir : directions) {
            Cell neighbor{current.cell.x + dir.x, current.cell.y + dir.y};

            if (!isValid(neighbor)) {
                continue;
            }

            // Diagonal cost is sqrt(2), straight is 1
            double move_cost = (dir.x != 0 && dir.y != 0) ? 1.414 : 1.0;
            double tentative_g = current.g_score + move_cost;

            if (all_nodes.find(neighbor) == all_nodes.end() || 
                tentative_g < all_nodes[neighbor].g_score) {
                Node neighbor_node{
                    neighbor,
                    tentative_g,
                    tentative_g + heuristic(neighbor, goal),
                    current.cell
                };
                open_set.push(neighbor_node);
                all_nodes[neighbor] = neighbor_node;
            }
        }
    }

    return false;
}

bool PlannerNode::isGoalReached() const {
    if (!has_pose_ || !has_goal_) return false;
    
    double dx = robot_pose_.position.x - current_goal_.x;
    double dy = robot_pose_.position.y - current_goal_.y;
    return std::sqrt(dx*dx + dy*dy) < 0.3;  // 0.3m threshold
}

double PlannerNode::heuristic(const Cell& a, const Cell& b) const {
    // Euclidean distance
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

bool PlannerNode::isValid(const Cell& cell) const {
    // Check bounds
    if (cell.x < 0 || cell.x >= current_map_.info.width ||
        cell.y < 0 || cell.y >= current_map_.info.height) {
        return false;
    }
    
    // Check if cell is occupied
    int index = cell.y * current_map_.info.width + cell.x;
    return current_map_.data[index] < 50;  // Consider <50 as free
}

PlannerNode::Cell PlannerNode::worldToMap(double wx, double wy) const {
    int mx = static_cast<int>((wx - current_map_.info.origin.position.x) / current_map_.info.resolution);
    int my = static_cast<int>((wy - current_map_.info.origin.position.y) / current_map_.info.resolution);
    return {mx, my};
}

std::pair<double, double> PlannerNode::mapToWorld(int mx, int my) const {
    double wx = current_map_.info.origin.position.x + (mx + 0.5) * current_map_.info.resolution;
    double wy = current_map_.info.origin.position.y + (my + 0.5) * current_map_.info.resolution;
    return {wx, wy};
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}