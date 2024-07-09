// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(char * argv[])
  : Node("minimal_publisher"), count_(0)
  {
    //sample code from documentation
    //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    drive_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));

    /*if (argc >= 1) {
      this->declare_parameter("argv0", argv[0]);
    } else {
      this->declare_parameter("argv0", "no argv0");
    }*/

    if (strcmp(argv[1], "v") == 0) {
      this->declare_parameter("v", argv[2]);
    } else {
      this->declare_parameter("v", "0");
    }
    if (strcmp(argv[3], "d") == 0) {
      this->declare_parameter("d", argv[4]);
    } else {
      this->declare_parameter("d", "0");
    }


  }

private:
  void timer_callback()
  {
    //sample code from ROS2 documentation
    /*auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);*/

    //for v and d
    std::string v_param = this->get_parameter("v").as_string();
    std::string d_param = this->get_parameter("d").as_string();
    //std::string argv0_param = this->get_parameter("argv0").as_string();

    RCLCPP_INFO(this->get_logger(), "v: %s!", v_param.c_str());
    RCLCPP_INFO(this->get_logger(), "d: %s!", d_param.c_str());
    //RCLCPP_INFO(this->get_logger(), "argv0: %s!", argv0_param.c_str());

    //drive publisher
    auto drive_message = std::make_shared<ackermann_msgs::msg::AckermannDriveStamped>();
    drive_message->drive.speed = stoi(v_param);
    drive_message->drive.steering_angle = stoi(d_param);
    drive_publisher->publish(drive_message);

    //std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("v", "1"), rclcpp::Parameter("d", "1")};
    //this->set_parameters(all_new_parameters);
    /*std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("d", "0")};
    this->set_parameters(all_new_parameters);*/

  }
  rclcpp::TimerBase::SharedPtr timer_;
  //sample code from documentation
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>(argv));
  rclcpp::shutdown();
  return 0;
}
