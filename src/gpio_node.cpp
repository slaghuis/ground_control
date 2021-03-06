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
 * Published a ground_control/gpio_write service for ground_control::msg::GpioWrite 
 * messages and sets the the mentioned GPIO Hi or Low as directed by the message
 * content. This can be used to run a buzzer or to light a LED connected to the 
 * respected GPIO port on the Rasbberry PI
 *
 * Published a ground_control/gpio_read service for ground_control::msg::GpioRead 
 * messages and returns the mentioned GPIO Hi or Low status. 
 * This can be used to read the switch status connected to the respected GPIO port 
 * on the Rasbberry PI
 *
 * Code adopted from Tiny GPIO Access - https://abyz.me.uk/rpi/pigpio/examples.html
 * *********************************************************************************/
#include <memory>
#include <vector>       //std::vector
#include <algorithm>    //std::find
#include <sys/mman.h>   //MAP_FAILED
#include <unistd.h>

#include <stdio.h>      // open() to access /dev/gpio*
#include <fcntl.h>      // Flow control

#include "rclcpp/rclcpp.hpp"

#include "ground_control/srv/gpio_read.hpp"
#include "ground_control/srv/gpio_write.hpp"

#define GPSET0 7
#define GPSET1 8

#define GPCLR0 10
#define GPCLR1 11

#define GPLEV0 13
#define GPLEV1 14

#define GPPUD     37
#define GPPUDCLK0 38
#define GPPUDCLK1 39

#define PI_BANK (gpio>>5)
#define PI_BIT  (1<<(gpio&0x1F))

/* gpio modes. */

#define PI_INPUT  0
#define PI_OUTPUT 1
#define PI_ALT0   4
#define PI_ALT1   5
#define PI_ALT2   6
#define PI_ALT3   7
#define PI_ALT4   3
#define PI_ALT5   2

/* Values for pull-ups/downs off, pull-down and pull-up. */

#define PI_PUD_OFF  0
#define PI_PUD_DOWN 1
#define PI_PUD_UP   2

using namespace std::chrono_literals;
using GPIOWrite = ground_control::srv::GpioWrite;
using GPIORead = ground_control::srv::GpioRead;

class GpioService : public rclcpp::Node
{
public:
  GpioService()
  : Node("gpio_service")
  {
    
    this->declare_parameter("writeable_gpio_pins", std::vector<int64_t>{17, 27, 22}) ;
    this->declare_parameter("readable_gpio_pins", std::vector<int64_t>{5, 6}) ;
    
    one_off_timer_ = this->create_wall_timer(
      500ms, std::bind(&GpioService::init, this));      
  }

private:
  void init() 
  {
    // Only run this once.  Stop the timer that triggered this.
    this->one_off_timer_->cancel();
    
    // Initialise GPIO
    if (gpio_initialise() < 0) {
      // No need for this node if it cannot address my GPIO.  Shutdown.         
      rclcpp::shutdown();
      return;
    }
    
    RCLCPP_INFO(
        this->get_logger(),
        "Pi model = %d, Pi revision = %d\n", piModel, piRev);

    // Read ROS parameters
    rclcpp::Parameter write_pins = this->get_parameter("writeable_gpio_pins");
    writeable_pins = write_pins.as_integer_array();
    
    rclcpp::Parameter read_pins = this->get_parameter("readable_gpio_pins");
    readable_pins = read_pins.as_integer_array();
    
    // Setup the writeable GPIO pins
    for (auto it : writeable_pins) {
      gpio_set_mode(it, PI_OUTPUT);
    }

    // Setup the GPIO pins
    for (auto it : readable_pins) {
      gpio_set_mode(it, PI_INPUT);
    }

    // Start the services.  Open for business.
    write_servcie_ = this->create_service<GPIOWrite>("ground_control/gpio_write", 
      std::bind(&GpioService::handle_write_service,this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));    
    read_servcie_ = this->create_service<GPIORead>("ground_control/gpio_read", 
      std::bind(&GpioService::handle_read_service,this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));    

  }
   
  void handle_write_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<GPIOWrite::Request> request,
    const std::shared_ptr<GPIOWrite::Response> response)
  {
    (void)request_header;
    
    if (request->number < (signed char) writeable_pins.size() ) {
      RCLCPP_INFO(
        this->get_logger(),
        "Adjusting GPIO number %i", writeable_pins[request->number]);
        
      if (request->high) {  
        gpio_write(writeable_pins[request->number],1);
      } else {  
        gpio_write(writeable_pins[request->number],0);
      }
      
      response->result = true;
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Specified LED %i out of range [0..%i]", request->number, writeable_pins.size()-1 );
      response->result = false;
    }
  }

  void handle_read_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<GPIORead::Request> request,
    const std::shared_ptr<GPIORead::Response> response)
  {
    (void)request_header;
    
    if (request->number < (signed char) readable_pins.size() ) {
      RCLCPP_INFO(
        this->get_logger(),
        "Reading GPIO number %i", readable_pins[request->number]);
      response->high = (gpio_read(readable_pins[request->number]) == 1);      
      response->result = true;
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Specified SIWTCH %i out of range [0..%i]", request->number, readable_pins.size()-1 );
      response->result = false;
    }
  }

  // GPIO SPECIFIC ROUTINES //////////////////////////////////////////////////////////////////////
  
  unsigned gpio_hardware_revision(void)
  {
     static unsigned rev = 0;

     FILE * filp;
     char buf[512];
     char term;
     int chars=4; /* number of chars in revision string */

     if (rev) return rev;

     piModel = 0;

     filp = fopen ("/proc/cpuinfo", "r");

     if (filp != NULL)
     {
        while (fgets(buf, sizeof(buf), filp) != NULL)
        {
           if (piModel == 0)
           {
              if (!strncasecmp("model name", buf, 10))
              {
                 if (strstr (buf, "ARMv6") != NULL)
                 {
                    piModel = 1;
                    chars = 4;
                 }
                 else if (strstr (buf, "ARMv7") != NULL)
                 {
                    piModel = 2;
                    chars = 6;
                 }
                 else if (strstr (buf, "ARMv8") != NULL)
                 {
                    piModel = 2;
                    chars = 6;
                 }
              }
           }

           if (!strncasecmp("revision", buf, 8))
           {
              if (sscanf(buf+strlen(buf)-(chars+1),
                 "%x%c", &rev, &term) == 2)
              {
                 if (term != '\n') rev = 0;
              }
           }
        }

        fclose(filp);
     }
     return rev;
  }

  int gpio_initialise(void) 
  {
    int fd;

    piRev = gpio_hardware_revision(); /* sets piModel and piRev */

    fd = open("/dev/gpiomem", O_RDWR | O_SYNC) ;

    if (fd < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/gpiomem");
      return -1;
    }

    gpioReg = (uint32_t *)mmap(NULL, 0xB4, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);

    close(fd);

    if (gpioReg == MAP_FAILED)
    {
      RCLCPP_ERROR(this->get_logger(), "Bad, mmap failed");
      return -1;
    }
    return 0;
  }
  
  void gpio_set_mode(unsigned gpio, unsigned mode)
  {
    int reg, shift;

    reg   =  gpio/10;
    shift = (gpio%10) * 3;

    gpioReg[reg] = (gpioReg[reg] & ~(7<<shift)) | (mode<<shift);
  }
 
  void gpio_write(unsigned gpio, unsigned level)
  {
    if (level == 0) *(gpioReg + GPCLR0 + PI_BANK) = PI_BIT;
    else            *(gpioReg + GPSET0 + PI_BANK) = PI_BIT;
  }
  
  int gpio_read(unsigned gpio)
  {
    if ((*(gpioReg + GPLEV0 + PI_BANK) & PI_BIT) != 0) return 1;
    else                                               return 0;
  }
  
  // PRIVATE VARIABLES ///////////////////////////////////////////////////////////////////
   
  uint32_t  *gpioReg;   // = MAP_FAILED
 
  std::vector<int64_t> writeable_pins;
  std::vector<int64_t> readable_pins;
  
  rclcpp::Service<GPIOWrite>::SharedPtr write_servcie_;
  rclcpp::Service<GPIORead>::SharedPtr read_servcie_;
  rclcpp::TimerBase::SharedPtr one_off_timer_;
  
  unsigned piModel;
  unsigned piRev;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpioService>());
  rclcpp::shutdown();
  return 0;
}

