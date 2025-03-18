#include <memory>

#include "follow_wall/FollowWallNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto followWall = std::make_shared<follow_wall::FollowWallNode>();
  rclcpp::spin(followWall);

  rclcpp::shutdown();

  return 0;
}
