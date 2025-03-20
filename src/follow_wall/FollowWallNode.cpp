#include "follow_wall/FollowWallNode.hpp"

#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace follow_wall
{

using namespace std::chrono_literals;
using std::placeholders::_1;

FollowWallNode::FollowWallNode()
: Node("follow_wall"), state_(GO_STRAIGHT) {
    
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&FollowWallNode::scan_callback, this, _1));
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    timer_ = create_wall_timer(20ms, std::bind(&FollowWallNode::control_cycle, this));
}

void FollowWallNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    front_distance_ = *std::min_element(msg->ranges.begin(), msg->ranges.begin() + 10);
    front_distance_ = std::min(front_distance_, *std::min_element(msg->ranges.end() - 10, msg->ranges.end()));
    left_distance_ = *std::min_element(msg->ranges.begin() + 80, msg->ranges.begin() + 100);
}

void FollowWallNode::control_cycle() {
    geometry_msgs::msg::Twist cmd;
    
    switch (state_) {
        case GO_STRAIGHT:
            cmd.linear.x = SPEED_LINEAR;
            if (front_distance_ < OBSTACLE_THRESHOLD) {
                go_state(AVOID_OBSTACLE);
            } else if (left_distance_ < MAX_WALL_DETECTION) {
                go_state(FOLLOW_WALL);
            }
            break;
        
        case FOLLOW_WALL:
            cmd.linear.x = SPEED_LINEAR;
            cmd.angular.z = -(left_distance_ - DISTANCE_TO_WALL) * WALL_FOLLOW_KP;
            if (front_distance_ < OBSTACLE_THRESHOLD) {
                go_state(AVOID_OBSTACLE);
            } else if (left_distance_ > MAX_WALL_DETECTION) {
                go_state(GO_STRAIGHT);
            }
            break;
        
        case AVOID_OBSTACLE:
            cmd.linear.x = 0.0;
            cmd.angular.z = SPEED_ANGULAR;
            if (front_distance_ > OBSTACLE_THRESHOLD) {
                go_state(FOLLOW_WALL);
            }
            break;
    }
    
    publisher_->publish(cmd);
}

void FollowWallNode::go_state(State new_state) {
    state_ = new_state;
}

}  // namespace follow_wall

