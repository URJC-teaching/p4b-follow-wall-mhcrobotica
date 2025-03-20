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
        "/scan_filtered", 10, std::bind(&FollowWallNode::scan_callback, this, _1));
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    timer_ = create_wall_timer(20ms, std::bind(&FollowWallNode::control_cycle, this));
}

void FollowWallNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int size = msg->ranges.size();
    
    // Detección frontal
    front_distance_ = *std::min_element(msg->ranges.begin(), msg->ranges.begin() + 30);
    front_distance_ = *std::min_element(msg->ranges.end() - 30, msg->ranges.end());
    
    // Detección lateral izquierda
    left_distance_ = *std::min_element(msg->ranges.begin() + size / 4 - 20, msg->ranges.begin() + size / 4 + 20);

    // Detección lateral derecha
    right_distance_ = *std::min_element(msg->ranges.begin() + 3 * size / 4 - 20, msg->ranges.begin() + 3 * size / 4 + 20);

    // Debug para verificar valores
    RCLCPP_INFO(this->get_logger(), "Front: %.2f, Left: %.2f, Right: %.2f", 
                front_distance_, left_distance_, right_distance_);
}


void FollowWallNode::control_cycle() {
    geometry_msgs::msg::Twist out_vel;
    
    switch (state_) {
        case GO_STRAIGHT:
            out_vel.linear.x = SPEED_LINEAR;
            if (front_distance_ < OBSTACLE_THRESHOLD) {
                go_state(AVOID_OBSTACLE);
            } else if (left_distance_ < MAX_WALL_DETECTION) {
                go_state(FOLLOW_WALL);
            }
            break;
        
        case FOLLOW_WALL:
            out_vel.linear.x = SPEED_LINEAR;
            out_vel.angular.z = -(left_distance_ - DISTANCE_TO_WALL) * WALL_FOLLOW_KP;
            if (front_distance_ < OBSTACLE_THRESHOLD) {
                go_state(AVOID_OBSTACLE);
            } else if (left_distance_ > MAX_WALL_DETECTION) {
                go_state(GO_STRAIGHT);
            }
            break;
        
        case AVOID_OBSTACLE:
            out_vel.linear.x = 0.0;
            out_vel.angular.z = SPEED_ANGULAR;
            if (left_distance_ > MAX_WALL_DETECTION && front_distance_ > OBSTACLE_THRESHOLD) {
                go_state(GO_STRAIGHT);
            } else if (left_distance_ < MAX_WALL_DETECTION) {
                go_state(FOLLOW_WALL);
            }
            break;
    }
    
    vel_pub_->publish(out_vel);
}

void FollowWallNode::go_state(State new_state) {
    state_ = new_state;
}

}  // namespace follow_wall

