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
 * Set LED-0 to light up
 * Wait for Switch 1 to be closed
 * Switch to Armed mode
 * **************************************************************************/

#include "ground_control/state.hpp"
#include "ground_control/ground_control_node.h"

void PoweredState::execute_mission() 
{
  rclcpp::Rate loop_rate(1);
  
  node_->set_led(0, true);
  
  while( rclcpp::ok() ) {  // Effectively an endless loop
     
    if ( node_->read_switch(0) ) {      
      // Switch 1 is toggled hot, transition 
      node_->TransitionTo(new ArmedState);
    } 
    loop_rate.sleep();
  }
  
}

void PoweredState::abort()
{
  // Do nothing
}
