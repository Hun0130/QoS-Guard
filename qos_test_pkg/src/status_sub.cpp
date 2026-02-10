/**
 * Topic D: /status Isolated Subscriber
 * No matching publisher - 구독 대상 없음
 * Mixed Priority: 코드와 XML이 충돌하는 값 가짐
 */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("status_sub");

  rclcpp::QoS qos(rclcpp::KeepLast(2));
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  auto sub = node->create_subscription<std_msgs::msg::String>(
      "/status", qos,
      [](const std_msgs::msg::String::SharedPtr) {});

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
