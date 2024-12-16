#include "rclcpp/rclcpp.hpp"
#include "sensor_coverage_planner/sensor_coverage_planner_ground.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node_handle = std::make_shared<rclcpp::Node>("uav_autonomous_exploration_node");//这个只是说节点的名字叫这个
  auto private_node_handle = node_handle->create_sub_node("uav_autonomous_exploration_node");//这样默认用这个nh弄出来的话题前面都会带/tare_planner_node头衔
  uav_exploration_ros_ns::uav_exploration_ros tare_planner(node_handle, private_node_handle);
  rclcpp::spin(node_handle);
  return 0;
}