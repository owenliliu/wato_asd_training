#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {

    map_memory_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(10).transient_local());
    

    cost_map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10,
        std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
        
    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MapMemoryNode::timerCallback, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if (!has_global_map_) {
        initializeGlobalMap(*msg);
    } else {
        fuseCostmap(*msg);
    }
    costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    
    double distance = std::hypot(x - last_x_, y - last_y_);
    
    if (distance >= distance_threshold_) {
        last_x_ = x;
        last_y_ = y;
        should_update_map_ = true;
    }
}

void MapMemoryNode::timerCallback()
{
    if (should_update_map_ && costmap_updated_ && has_global_map_) {
        map_memory_pub_->publish(global_map_);
        should_update_map_ = false;
        costmap_updated_ = false;
    }
}

void MapMemoryNode::initializeGlobalMap(const nav_msgs::msg::OccupancyGrid& costmap) {
    global_map_ = costmap;
    global_map_.header.frame_id = "map";  // Fixed frame
    global_map_.info.origin.position.x = -10.0;  // Set fixed origin
    global_map_.info.origin.position.y = -10.0;
    has_global_map_ = true;
}

void MapMemoryNode::fuseCostmap(const nav_msgs::msg::OccupancyGrid& costmap)
{
    for (size_t i = 0; i < costmap.data.size(); ++i) {
        if (costmap.data[i] != -1) { // Only update known cells
            global_map_.data[i] = costmap.data[i];
        }
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
