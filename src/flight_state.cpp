// Copyright 2021 Xeni Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* **************************************************************************
 * Set LED-2 to light up
 * Call the flight controller to take-off.
 * Just hang around for now.
 * **************************************************************************/
#include "ground_control/state.hpp"
#include "ground_control/ground_control_node.h"

using Mission = navigation_interfaces::srv::Mission;
using namespace std::chrono_literals;

void FlightState::execute_mission() 
{
  //  Do nothing.  Light one up
  node_->set_led(2, true);  // The red LED is on.
  
  // Sound an alarm??
  
  // Takeoff
  rclcpp::Client<Mission>::SharedPtr client = 
    node_->create_client<Mission>("drone/mission"); 
     
  auto request = std::make_shared<Mission::Request>();
  request->drone_code = 42;
  request->mission_file = "Square.xml";
  
  while (!client->wait_for_service(250ms)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      abort();
    }
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
     
  auto future = client->async_send_request(request);

  std::future_status status;
  do {
    status = future.wait_for(250ms);  // Not spinning here!  We are in a thread, and the spinning is taken cared of elsewhere.
    if (status == std::future_status::deferred) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Future deferred");
    } else if (status == std::future_status::timeout) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Future timeout");
    } else if (status == std::future_status::ready) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Future ready!");
    }
  } while (status != std::future_status::ready); 
     
 /* bool service_result = false; 
     
  if (status == std::future_status::ready)
  {
    service_result = future.get()->accepted; 
  }   
  
  // What do I do if service_result == FALSE?
 */ 
}


void FlightState::abort()
{
  // Do nothing
  node_->set_led(2, false);  // The amber LED is off.
  node_->TransitionTo(new PoweredState);
}
