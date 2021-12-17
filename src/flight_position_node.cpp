// Copyright (c) 2021 Xeni Robotics
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

/* *********************************************************************************
 * Transformation Listener for the drone postision in the map frame.
 * Publish the position as a well formed message string
 * *********************************************************************************/
#include <chrono>
#include <memory>
#include <string>      //std::string
#include <sstream>     //std::stringstream
#include <iomanip>     //std::setprecision

#include "rclcpp/rclcpp.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class PositionMessagePublisher : public rclcpp::Node
{
public:
  PositionMessagePublisher()
  : Node("position_publisher")
  {
  
    // Create a transform listener
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      
    publisher_ = this->create_publisher<std_msgs::msg::String>("ground_control/line_two", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&PositionMessagePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    double x, y, z, w;  

    auto message = std_msgs::msg::String();    
    
    if ( read_position(&x, &y, &z, &w) ) {
      std::stringstream ss(std::stringstream::in | std::stringstream::out);
      ss.precision(1);
      ss << "E" << std::fixed << x << " N" << std::fixed << y << " U" << std::fixed << z;
      message.data = ss.str();
    } else {
      message.data = "Position unavailable";
    }
    
    publisher_->publish(message);
  }
  
  bool read_position(double *x, double *y, double *z, double *w)
  {
    std::string from_frame = "base_link_ned"; //map_frame_.c_str();
    std::string to_frame = "map";
    
    geometry_msgs::msg::TransformStamped transformStamped;
    
    // Look up for the transformation between map and base_link frames
    // and save the last position in the 'map' frame
    try {
      rclcpp::Time now = this->get_clock()->now();
      transformStamped = tf_buffer_->lookupTransform(
        to_frame, from_frame,
        now, 100ms);
        *x = transformStamped.transform.translation.x;
        *y = transformStamped.transform.translation.y;
        *z = transformStamped.transform.translation.z;
    } catch (tf2::TransformException & ex) {
      RCLCPP_DEBUG(
        this->get_logger(), "Could not transform %s to %s: %s",
        to_frame.c_str(), from_frame.c_str(), ex.what());
      return false;  
    }
    
    // Orientation quaternion
    tf2::Quaternion q(
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z,
        transformStamped.transform.rotation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch; 
    m.getRPY(roll, pitch, *w);
   
    return true;
  }

  // PRIVATE VARIABLES ////////////////////////////////////////////////////////////////////////
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionMessagePublisher>());
  rclcpp::shutdown();
  return 0;
}