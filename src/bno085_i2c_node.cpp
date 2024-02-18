/* bno085_i2c_node.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 * Author: Andy Blight
 *
 * Instantiates a BNO085 I2CDriver class, as well as
 * a Watchdog that causes this node to die if things aren't
 * working.
 */

#include "bno085_i2c_node.hpp"

#include <chrono>
#include <csignal>
#include <memory>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "bno085_i2c_driver.hpp"
#include "watchdog/watchdog.hpp"

static const char * kNodeName = "bno085_node";
static const char * kDataTopicName = "data";
static const char * kRawDataTopicName = "raw";
static const char * kMagneticTopicName = "mag";
static const char * kTemperatureTopicName = "temp";
static const char * kStatusTopicName = "status";

BNO085I2CNode::BNO085I2CNode(const rclcpp::NodeOptions & options)
: Node(kNodeName, options), publish_status_count_(0)
{
  // Declare parameters that may be set on this node
  this->declare_parameter<int>("address");
  this->declare_parameter<std::string>("device");
  this->declare_parameter<std::string>("frame_id");
  this->declare_parameter<int64_t>("rate");

  // Get parameters from yaml
  this->get_parameter_or<int>("address", address, BNO085_ADDRESS_A);
  this->get_parameter_or<std::string>("device", device, "/dev/i2c-1");
  this->get_parameter_or<std::string>("frame_id", frame_id, "imu");
  this->get_parameter_or<int64_t>("rate", rate, 100);

  RCLCPP_INFO(get_logger(),
              "Connecting to BNO08x IMU on i2c bus '%s' on address '%d' (%x)",
              device.c_str(), address, address);

  // Create and initialise driver instance.
  imu = std::make_unique<BNO085I2CDriver>(
    device,
    address);
  imu->init();

  // Create publishers.
  data_publisher_ = create_publisher<sensor_msgs::msg::Imu>(kDataTopicName, 10);
  raw_data_publisher_ = create_publisher<sensor_msgs::msg::Imu>(kRawDataTopicName, 10);
  magnetic_publisher_ = create_publisher<sensor_msgs::msg::MagneticField>(kMagneticTopicName, 10);
  temperature_publisher_ =
    create_publisher<sensor_msgs::msg::Temperature>(kTemperatureTopicName, 10);
  status_publisher_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(kStatusTopicName, 10);

  // Create timer to publish data.
  std::chrono::milliseconds param_rate_ms(rate);
  timer_ = create_wall_timer(param_rate_ms, std::bind(&BNO085I2CNode::TimerCallback, this));

  InitDiagnosticMsg();
}

void BNO085I2CNode::InitDiagnosticMsg()
{
  current_status.level = 0;
  current_status.name = "BNO085 IMU";
  current_status.hardware_id = "bno085_i2c";

  diagnostic_msgs::msg::KeyValue calib_stat;
  calib_stat.key = "Calibration status";
  calib_stat.value = "";
  current_status.values.push_back(calib_stat);

  diagnostic_msgs::msg::KeyValue selftest_result;
  selftest_result.key = "Self-test result";
  selftest_result.value = "";
  current_status.values.push_back(selftest_result);

  diagnostic_msgs::msg::KeyValue intr_stat;
  intr_stat.key = "Interrupt status";
  intr_stat.value = "";
  current_status.values.push_back(intr_stat);

  diagnostic_msgs::msg::KeyValue sys_clk_stat;
  sys_clk_stat.key = "System clock status";
  sys_clk_stat.value = "";
  current_status.values.push_back(sys_clk_stat);

  diagnostic_msgs::msg::KeyValue sys_stat;
  sys_stat.key = "System status";
  sys_stat.value = "";
  current_status.values.push_back(sys_stat);

  diagnostic_msgs::msg::KeyValue sys_err;
}

void BNO085I2CNode::PublishData(
  const builtin_interfaces::msg::Time msg_time,
  const BNO085I2CIMURecord & record)
{
  sensor_msgs::msg::Imu msg_data;
  msg_data.header.stamp = msg_time;
  msg_data.header.frame_id = param_frame_id_;

  double fused_orientation_norm =
    std::pow(
    std::pow(record.fused_orientation_w, 2) +
    std::pow(record.fused_orientation_x, 2) +
    std::pow(record.fused_orientation_y, 2) +
    std::pow(record.fused_orientation_z, 2),
    0.5);
  msg_data.orientation.w =
    static_cast<double>(record.fused_orientation_w) / fused_orientation_norm;
  msg_data.orientation.x =
    static_cast<double>(record.fused_orientation_x) / fused_orientation_norm;
  msg_data.orientation.y =
    static_cast<double>(record.fused_orientation_y) / fused_orientation_norm;
  msg_data.orientation.z =
    static_cast<double>(record.fused_orientation_z) / fused_orientation_norm;
  msg_data.linear_acceleration.x =
    static_cast<double>(record.fused_linear_acceleration_x) / 100.0;
  msg_data.linear_acceleration.y =
    static_cast<double>(record.fused_linear_acceleration_y) / 100.0;
  msg_data.linear_acceleration.z =
    static_cast<double>(record.fused_linear_acceleration_z) / 100.0;
  msg_data.angular_velocity.x =
    static_cast<double>(record.raw_angular_velocity_x) / 900.0;
  msg_data.angular_velocity.y =
    static_cast<double>(record.raw_angular_velocity_y) / 900.0;
  msg_data.angular_velocity.z =
    static_cast<double>(record.raw_angular_velocity_z) / 900.0;

  // Publish the message.
  data_publisher_->publish(msg_data);
}

void BNO085I2CNode::PublishRawData(
  const builtin_interfaces::msg::Time msg_time,
  const BNO085I2CIMURecord & record)
{
  sensor_msgs::msg::Imu msg_raw;
  msg_raw.header.stamp = msg_time;
  msg_raw.header.frame_id = param_frame_id_;
  msg_raw.linear_acceleration.x =
    static_cast<double>(record.raw_linear_acceleration_x) / 100.0;
  msg_raw.linear_acceleration.y =
    static_cast<double>(record.raw_linear_acceleration_y) / 100.0;
  msg_raw.linear_acceleration.z =
    static_cast<double>(record.raw_linear_acceleration_z) / 100.0;
  msg_raw.angular_velocity.x =
    static_cast<double>(record.raw_angular_velocity_x) / 900.0;
  msg_raw.angular_velocity.y =
    static_cast<double>(record.raw_angular_velocity_y) / 900.0;
  msg_raw.angular_velocity.z =
    static_cast<double>(record.raw_angular_velocity_z) / 900.0;

  // Publish the message.
  raw_data_publisher_->publish(msg_raw);
}

void BNO085I2CNode::PublishMagnetic(
  const builtin_interfaces::msg::Time msg_time,
  const BNO085I2CIMURecord & record)
{
  sensor_msgs::msg::MagneticField msg_mag;
  msg_mag.header.stamp = msg_time;
  msg_mag.header.frame_id = param_frame_id_;
  msg_mag.magnetic_field.x = static_cast<double>(record.raw_magnetic_field_x / 16.0);
  msg_mag.magnetic_field.y = static_cast<double>(record.raw_magnetic_field_y / 16.0);
  msg_mag.magnetic_field.z = static_cast<double>(record.raw_magnetic_field_z / 16.0);
  magnetic_publisher_->publish(msg_mag);
}

void BNO085I2CNode::PublishTemperature(
  const builtin_interfaces::msg::Time msg_time,
  const BNO085I2CIMURecord & record)
{
  sensor_msgs::msg::Temperature msg_temp;
  msg_temp.header.stamp = msg_time;
  msg_temp.header.frame_id = param_frame_id_;
  msg_temp.temperature = static_cast<double>(record.temperature);
  temperature_publisher_->publish(msg_temp);
}

void BNO085I2CNode::PublishStatus(const BNO085I2CIMURecord & record)
{
  current_status.values[DIAG_CALIB_STAT].value =
    std::to_string(record.calibration_status);
  current_status.values[DIAG_SELFTEST_RESULT].value =
    std::to_string(record.self_test_result);
  current_status.values[DIAG_INTR_STAT].value =
    std::to_string(record.interrupt_status);
  current_status.values[DIAG_SYS_CLK_STAT].value =
    std::to_string(record.system_clock_status);
  current_status.values[DIAG_SYS_STAT].value =
    std::to_string(record.system_status);
  current_status.values[DIAG_SYS_ERR].value =
    std::to_string(record.system_error_code);
  status_publisher_->publish(current_status);
}

void BNO085I2CNode::TimerCallback()
{
  // Get the data from the IMU.
  BNO085I2CIMURecord record;

  try {
    record = imu->read();
    // Update time for header.
    builtin_interfaces::msg::Time msg_time = now();

    // Publish all of the messages.
    PublishData(msg_time, record);
    PublishRawData(msg_time, record);
    PublishMagnetic(msg_time, record);
    PublishTemperature(msg_time, record);
    if (publish_status_count_ % 50 == 0) {
      PublishStatus(record);
      publish_status_count_ = 0;
    }
    ++publish_status_count_;
  } catch (const std::runtime_error & e) {
    RCLCPP_WARN(get_logger(), "%s: %s", __func__, e.what());
  }
  watchdog_.refresh();
}
