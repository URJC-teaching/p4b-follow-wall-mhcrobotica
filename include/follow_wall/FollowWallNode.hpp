#ifndef FOLLOW_WALL_NODE_HPP
#define FOLLOW_WALL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace follow_wall
{

class FollowWallNode : public rclcpp::Node {
public:
    FollowWallNode();

private:
    enum State { FOLLOW_WALL, GO_STRAIGHT, AVOID_OBSTACLE };

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void control_cycle();
    void go_state(State new_state);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_;
    float front_distance_;
    float left_distance_;
    float right_distance_;


    static constexpr float SPEED_LINEAR = 0.3f;
    static constexpr float SPEED_ANGULAR = 0.6f;
    static constexpr float DISTANCE_TO_WALL = 1.0;
    static constexpr float OBSTACLE_THRESHOLD = 0.5;
    static constexpr float MAX_WALL_DETECTION = 2.0;
    static constexpr float WALL_FOLLOW_KP = 1.5;
};

}  // namespace follow_wall

#endif  // FOLLOW_WALL_NODE_HPP

