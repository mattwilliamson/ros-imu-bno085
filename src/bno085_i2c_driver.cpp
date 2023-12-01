/* bno085_i2c_driver.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 */

#include "bno085_i2c_driver.hpp"

#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

BNO085I2CDriver::BNO085I2CDriver(std::string device_, int address_)
{
  device = device_;
  address = address_;
}

bool BNO085I2CDriver::reset()
{
  int i = 0;

  _i2c_smbus_write_byte_data(
    file, BNO085_OPR_MODE_ADDR,
    BNO085_OPERATION_MODE_CONFIG);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  // reset
  _i2c_smbus_write_byte_data(file, BNO085_SYS_TRIGGER_ADDR, 0x20);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  // wait for chip to come back online
  while (_i2c_smbus_read_byte_data(file, BNO085_CHIP_ID_ADDR) != BNO085_ID) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (i++ > 500) {
      auto id_addr = _i2c_smbus_read_byte_data(file, BNO085_CHIP_ID_ADDR);
      std::stringstream buf;
      buf << "chip did not come back online within 5 seconds of reset - "
        "advertising addr 0x"
          << std::hex << id_addr;
      throw std::runtime_error(buf.str());
      return false;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // normal power mode
  _i2c_smbus_write_byte_data(
    file, BNO085_PWR_MODE_ADDR,
    BNO085_POWER_MODE_NORMAL);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  _i2c_smbus_write_byte_data(file, BNO085_PAGE_ID_ADDR, 0);
  _i2c_smbus_write_byte_data(file, BNO085_SYS_TRIGGER_ADDR, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  _i2c_smbus_write_byte_data(
    file, BNO085_OPR_MODE_ADDR,
    BNO085_OPERATION_MODE_NDOF);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  return true;
}

void BNO085I2CDriver::init()
{
  file = open(device.c_str(), O_RDWR);

  if (ioctl(file, I2C_SLAVE, address) < 0) {
    throw std::runtime_error("i2c device open failed");
  }

  if (_i2c_smbus_read_byte_data(file, BNO085_CHIP_ID_ADDR) != BNO085_ID) {
    throw std::runtime_error("incorrect chip ID");
  }

  std::cerr
    << "rev ids:"
    << " accel:" << _i2c_smbus_read_byte_data(file, BNO085_ACCEL_REV_ID_ADDR)
    << " mag:" << _i2c_smbus_read_byte_data(file, BNO085_MAG_REV_ID_ADDR)
    << " gyro:" << _i2c_smbus_read_byte_data(file, BNO085_GYRO_REV_ID_ADDR)
    << " sw:" << _i2c_smbus_read_word_data(file, BNO085_SW_REV_ID_LSB_ADDR)
    << " bl:" << _i2c_smbus_read_byte_data(file, BNO085_BL_REV_ID_ADDR)
    << std::endl;

  if (!reset()) {
    throw std::runtime_error("chip init failed");
  }
}

BNO085I2CIMURecord BNO085I2CDriver::read()
{
  BNO085I2CIMURecord record;

  // can only read a length of 0x20 at a time, so do it in 2 reads
  // BNO085_LINEAR_ACCEL_DATA_X_LSB_ADDR is the start of the data block
  // that aligns with the IMURecord struct.
  int result =
    _i2c_smbus_read_i2c_block_data(
    file, BNO085_ACCEL_DATA_X_LSB_ADDR, 0x20,
    reinterpret_cast<uint8_t *>(&record));
  if (result != 0x20) {
    throw std::runtime_error("read error");
  }
  result = _i2c_smbus_read_i2c_block_data(
    file, BNO085_ACCEL_DATA_X_LSB_ADDR + 0x20, 0x13,
    reinterpret_cast<uint8_t *>(&record + 0x20));
  if (result != 0x13) {
    throw std::runtime_error("read error");
  }

  return record;
}
