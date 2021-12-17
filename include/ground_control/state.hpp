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

#pragma once

#include <memory>  // std::shared_ptr

#include "rclcpp/rclcpp.hpp"
#include "ground_control/holddown_timer.hpp"
#include "navigation_interfaces/srv/mission.hpp"

//using GPIORead = ground_control::srv::GpioRead;

class GroundControl;

class State {
  protected:
    GroundControl * node_;
  
  public:  
    virtual ~State()
    {};
  
    void set_context(GroundControl * node) {
      this->node_ = node;
    }  
    virtual void execute_mission() = 0;
    virtual void abort() = 0;
};

// POWERED STATE /////////////////////////////////////////////////////////////////////////////////
class PoweredState : public State {
 public:
  void execute_mission() override;
  void abort() override;
};

// ARMED STATE /////////////////////////////////////////////////////////////////////////////////
class ArmedState : public State {
public:
  void execute_mission() override;
  void abort() override;
private:
  // Hoddown Timer
  std::shared_ptr<HolddownTimer> holddown_timer;
  
};

// FLIGHT STATE /////////////////////////////////////////////////////////////////////////////////
class FlightState : public State {
 public:
  void execute_mission() override;
  void abort() override;
};