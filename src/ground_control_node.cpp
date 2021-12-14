/* **************************************************************************
 * The face that launched a thousand ships
 * **************************************************************************/
 
#include <chrono>
#include <cinttypes>
#include <memory>
#include "navigation_interfaces/srv/mission.hpp"
#include "rclcpp/rclcpp.hpp"

using Mission = navigation_interfaces::srv::Mission;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_client");
  auto client = node->create_client<Mission>("drone/mission");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
  }
  auto request = std::make_shared<Mission::Request>();
  request->drone_code = 42;
  request->mission_file = "FlySquare.xml";
  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    return 1;
  }
  auto result = result_future.get();
  // RCLCPP_INFO(
  //  node->get_logger(), "result of %" PRId64 " + %" PRId64 " = %" PRId64,
  //  request->a, request->b, result->sum);
  rclcpp::shutdown();
  return 0;
}
