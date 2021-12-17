/* **************************************************************************
 * The face that launched a thousand ships
 * **************************************************************************/

#include "ground_control/ground_control_node.h"

// using GPIOWrite = ground_control::srv::GpioWrite;

using namespace std::chrono_literals;

GroundControl::GroundControl(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("ground_control", options)
  {
  
  this->declare_parameter<int>("holddown", DEFAULT_HOLDDOWN);
    
  publisher_ = this->create_publisher<std_msgs::msg::String>("ground_control/line_two", 10);

  this->TransitionTo(new PoweredState);   // Setup the state machine in powered state
}

void GroundControl::TransitionTo(State *state) {
  RCLCPP_INFO(this->get_logger(), "Context: Transition to %s", typeid(*state).name());
  if (this->state_ != nullptr)
    delete this->state_;
  this->state_ = state;
  this->state_->set_context(this);
  this->execute_mission();
}

void GroundControl::execute_mission() {
  this->state_->execute_mission();
}

void GroundControl::abort() {
  this->state_->abort();
}

bool GroundControl::set_led(int8_t number, bool high){
  
  rclcpp::Client<ground_control::srv::GpioWrite>::SharedPtr client = 
    create_client<ground_control::srv::GpioWrite>("ground_control/gpio_write"); 
     
  auto request = std::make_shared<ground_control::srv::GpioWrite::Request>();
  request->number = number;
  request->high = high;
  request->duration_ms = 0;
  
  while (!client->wait_for_service(250ms)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return false;
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
     
  bool service_result = false; 
     
  if (status == std::future_status::ready)
  {
    service_result = future.get()->result; 
  } 
     
  return service_result;   
}

bool GroundControl::read_switch(int8_t number) {
  
  rclcpp::Client<ground_control::srv::GpioRead>::SharedPtr client = 
    create_client<ground_control::srv::GpioRead>("ground_control/gpio_read"); 
     
  auto request = std::make_shared<ground_control::srv::GpioRead::Request>();
  request->number = number;
  
  while (!client->wait_for_service(250ms)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return false;
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
     
  bool service_result = false; 
  bool switch_state = false;   // An off swtch is usually safer than an hot switch
  
  if (status == std::future_status::ready)
  {
    service_result = future.get()->result;
    switch_state = future.get()->high;
  } 
    
  if (service_result) service_result = switch_state;
  
  return service_result;   
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundControl>());
  rclcpp::shutdown();
  return 0;
}
