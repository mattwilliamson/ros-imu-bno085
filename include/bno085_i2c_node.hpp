// Copyright 2023 University of Leeds.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef BNO085_I2C_NODE_HPP_
#define BNO085_I2C_NODE_HPP_

#include "bno085_i2c_driver.hpp"

#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "watchdog/watchdog.hpp"

class BNO085I2CNode : public rclcpp::Node
{
public:
  explicit BNO085I2CNode(const rclcpp::NodeOptions & options);

private:
  void InitDiagnosticMsg();
  void PublishData(
    const builtin_interfaces::msg::Time msg_time,
    const BNO085I2CIMURecord & record);
  void PublishRawData(
    const builtin_interfaces::msg::Time msg_time,
    const BNO085I2CIMURecord & record);
  void PublishMagnetic(
    const builtin_interfaces::msg::Time msg_time,
    const BNO085I2CIMURecord & record);
  void PublishTemperature(
    const builtin_interfaces::msg::Time msg_time,
    const BNO085I2CIMURecord & record);
  void PublishStatus(const BNO085I2CIMURecord & record);
  void TimerCallback();

  int address;
  std::string device;
  std::string frame_id;
  int64_t rate;

  std::unique_ptr<BNO085I2CDriver> imu;
  std::string param_frame_id_;
  diagnostic_msgs::msg::DiagnosticStatus current_status;
  watchdog::Watchdog watchdog_;
  int publish_status_count_;

  // Publishers.
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr data_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_data_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_publisher_;

  // The timer thread that causes messages to be published regularly.
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // BNO085_I2C_NODE_HPP_
