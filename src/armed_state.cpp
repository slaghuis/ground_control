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

#include "ground_control/state.hpp"
#include "ground_control/ground_control_node.h"

void ArmedState::execute_mission() 
{
  // Read parameters
  int holddown;
  node_->get_parameter("holddown", holddown); 
  holddown_timer = std::make_shared<HolddownTimer>(holddown);
  
  node_->set_led(1, true);  // The amber LED is on.
  rclcpp::Rate loop_rate(2);
  // Test if the switch is on for 3 seconds before launch
  holddown_timer->test( node_->read_switch(1) );
  do {
    
    if ( ! node_->read_switch(0) ) {      
      // Switch 0 is toggled off.  Rollback 
      abort();
    } 
    loop_rate.sleep();     
  } while (holddown_timer->test( node_->read_switch(1) ));
  
  node_->set_led(1, false);  // The amber LED is off.
  node_->TransitionTo(new FlightState);
}


void ArmedState::abort()
{
  // Do nothing
  node_->set_led(1, false);  // The amber LED is off.
  node_->TransitionTo(new PoweredState);
}
