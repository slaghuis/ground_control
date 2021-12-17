#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ground_control/srv/gpio_write.hpp"
#include "ground_control/srv/gpio_read.hpp"

#include "ground_control/state.hpp"

static const int DEFAULT_HOLDDOWN = 3;

class GroundControl : public rclcpp::Node
{
public:
  explicit GroundControl(const rclcpp::NodeOptions &);
  
  void TransitionTo(State *state);
  void execute_mission();
  void abort();
  bool is_working();
  
  // Utility functions
  bool set_led(int8_t number, bool high);
  bool read_switch(int8_t number);
private:
  
  // Global Variables
  State *state_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

