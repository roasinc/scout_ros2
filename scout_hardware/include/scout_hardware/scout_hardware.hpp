// Copyright (c) 2023, ROAS Inc.
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

#ifndef SCOUT_HARDWARE__SCOUT_HAREWARE_HPP_
#define SCOUT_HARDWARE__SCOUT_HAREWARE_HPP_

#include <string>
#include <vector>
#include <cmath>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "realtime_tools/realtime_publisher.h"

#include "can_msgs/msg/frame.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"

#include "scout_msgs/msg/battery_state.hpp"
#include "scout_msgs/msg/driver_state.hpp"
#include "scout_msgs/msg/fault_state.hpp"
#include "scout_msgs/msg/light.hpp"
#include "scout_msgs/msg/light_command.hpp"
#include "scout_msgs/msg/light_state.hpp"
#include "scout_msgs/msg/motor_state.hpp"
#include "scout_msgs/msg/robot_state.hpp"

#include "scout_hardware/visibility_control.h"

namespace scout_hardware
{
class ScoutHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ScoutHardware)

  SCOUT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  SCOUT_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  SCOUT_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  SCOUT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  SCOUT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  SCOUT_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  SCOUT_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /**
   * \brief Callback for reading from hardware interface
   */
  void receive();

  /**
   * \brief Parse the robot state message
   * \param data Robot state message converted to binary number
   */
  void robotState(uint8_t* data);

  /**
   * \brief Parse the motor state message
   * \param index Index of motor
   * \param data Motor state converted to binary number
   */
  void motorState(size_t index, uint8_t* data);

  /**
   * \brief Parse the driver state message
   * \param index Index of driver
   * \param data Driver state converted to binary number
   */
  void driverState(size_t index, uint8_t* data);

  /**
   * \brief Parse the light state message
   * \param data Light state message converted to binary number
   */
  void lightState(uint8_t* data);

  /**
   * \brief Parse the velocity message
   * \param data Velocity message converted to binary number
   */
  void velocity(uint8_t* data);

  /**
   * \brief Parse the position message
   * \param data Position message converted to binary number
   */
  void position(uint8_t* data);

  /**
   * \brief Parse the battery state message
   * \param data Battery state message converted to binary number
   */
  void batteryState(uint8_t* data);

  /**
   * \brief Publish state data
   */
  void publish();

  /**
   * \brief Front light command callback
   * \param msg Front light command message
   */
  void frontLightCmdCallback(const scout_msgs::msg::LightCommand::SharedPtr& msg);

  /**
   * \brief Rear light command callback
   * \param msg Rear light command message
   */
  void rearLightCmdCallback(const scout_msgs::msg::LightCommand::SharedPtr& msg);

  /// Robot hardware interface node
  std::shared_ptr<rclcpp::Node> node_;

private:
  /// Hardware interface
  std::vector<double> hw_commands_, hw_positions_, hw_velocities_, hw_efforts_;

  /// rostopic publisher
  rclcpp::Publisher<scout_msgs::msg::BatteryState>::SharedPtr pub_battery_state_;
  rclcpp::Publisher<scout_msgs::msg::DriverState>::SharedPtr pub_driver_state_;
  rclcpp::Publisher<scout_msgs::msg::LightState>::SharedPtr pub_light_state_;
  rclcpp::Publisher<scout_msgs::msg::MotorState>::SharedPtr pub_motor_state_;
  rclcpp::Publisher<scout_msgs::msg::RobotState>::SharedPtr pub_robot_state_;
  std::shared_ptr<realtime_tools::RealtimePublisher<scout_msgs::msg::BatteryState>> rp_battery_state_;
  std::shared_ptr<realtime_tools::RealtimePublisher<scout_msgs::msg::DriverState>> rp_driver_state_;
  std::shared_ptr<realtime_tools::RealtimePublisher<scout_msgs::msg::LightState>> rp_light_state_;
  std::shared_ptr<realtime_tools::RealtimePublisher<scout_msgs::msg::MotorState>> rp_motor_state_;
  std::shared_ptr<realtime_tools::RealtimePublisher<scout_msgs::msg::RobotState>> rp_robot_state_;

  /// rostopic subscriber
  rclcpp::Subscription<scout_msgs::msg::LightCommand>::SharedPtr sub_front_led_cmd_;
  rclcpp::Subscription<scout_msgs::msg::LightCommand>::SharedPtr sub_rear_led_cmd_;

  /// Socket CAN
  std::unique_ptr<drivers::socketcan::SocketCanSender> sender_;
  std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver_;
  std::unique_ptr<std::thread> receiver_thread_;
  std::string interface_;
  std::chrono::nanoseconds timeout_ns_, interval_ns_;

  /// Robot parameters
  double wheel_radius_, wheel_separation_;
};
}  // namespace scout_hardware

#endif  // SCOUT_HARDWARE__SCOUT_HAREWARE_HPP_