/*
 * test_driver.cpp
 *
 * License: BSD-3-Clause
 *
 * Copyright (c) 2015-2023 RT Corporation <support@rt-net.jp>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of RT Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <vector>
#include <gtest/gtest.h>
#include "fakeit.hpp"
#include "rt_usb_9axisimu_driver/rt_usb_9axisimu_driver.hpp"
#include "rt_usb_9axisimu_driver/rt_usb_9axisimu.hpp"

using fakeit::Mock;
using fakeit::When;
using rt_usb_9axisimu::SerialPort;

unsigned char int16_to_byte_low_8bit(int16_t val);
unsigned char int16_to_byte_high_8bit(int16_t val);
unsigned int create_dummy_bin_imu_data(unsigned char *buf, bool is_invalid);
unsigned int create_dummy_bin_imu_data(unsigned char *buf, bool is_invalid,
                                       int16_t *gyro, int16_t *acc, int16_t *mag, int16_t temp);
std::string double_to_string(double val);
unsigned int create_dummy_ascii_imu_data(unsigned char *buf, bool is_invalid);
unsigned int create_dummy_ascii_imu_data(unsigned char *buf, bool is_invalid,
                                         double *gyro, double *acc, double *mag, double temp);

class ReadBinaryTestParam {
public:
  int16_t gyro[3], acc[3], mag[3], temp;
  double ans_gyro[3], ans_acc[3], ans_mag[3], ans_temp;

  ReadBinaryTestParam (int case_num) {
    if (case_num == 0) set_data(0, 0.0, 0.0, 0.0, 21.0); // zero
    else if (case_num == 1) set_data(32767, 34.87147, 156.90161, 0.00492, 119.14299); // max
    else if (case_num == 2) set_data(-32768, -34.87253, -156.9064, -0.00492, -77.14598); // min
    else if (case_num == 3) set_data(32766, 34.870401, 156.896823, 0.004915, 119.139995); // boundary
    else if (case_num == 4) set_data(-32767, -34.871466, -156.901612, -0.004915, -77.14299); // boundary
    else if (case_num == 5) set_data(1, 0.001064, 0.004788, 0.0, 21.002995); // random
    else if (case_num == 6) set_data(-1, -0.001064, -0.004788, 0.0, 20.997005); // random
    else if (case_num == 7) set_data(999, 1.063161, 4.783615, 0.00015, 23.992183); // random
    else if (case_num == 8) set_data(-999, -1.063161, -4.783615, -0.00015, 18.007817); // random
    else if (case_num == 9) set_data(12345, 13.13786, 59.112839, 0.001852, 57.975469); // random
    else if (case_num == 10) set_data(-12345, -13.13786, -59.112839, -0.001852, -15.975469); // random
  }

private:
  void set_data (int16_t test_val,
                 double ans_gyro_val, double ans_acc_val, double ans_mag_val, double ans_temp_val) {
    gyro[0] = gyro[1] = gyro[2] = test_val;
    acc[0] = acc[1] = acc[2] = test_val;
    mag[0] = mag[1] = mag[2] = test_val;
    temp = test_val;
    ans_gyro[0] = ans_gyro[1] = ans_gyro[2] = ans_gyro_val;
    ans_acc[0] = ans_acc[1] = ans_acc[2] = ans_acc_val;
    ans_mag[0] = ans_mag[1] = ans_mag[2] = ans_mag_val;
    ans_temp = ans_temp_val;
  }
};

class ReadAsciiTestParam {
public:
  double gyro[3], acc[3], mag[3], temp;
  double ans_gyro[3], ans_acc[3], ans_mag[3], ans_temp;

  ReadAsciiTestParam (int case_num) {
    if(case_num == 0) set_data(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); // zero
    else if(case_num == 1) set_data(34.906585, 16.0, 4800.0, 85.0, 156.9064, 0.0048); // max
    else if(case_num == 2) set_data(-34.906585, -16.0, -4800.0, -40.0, -156.9064, -0.0048); // min
    else if(case_num == 3) set_data(0.00107, 0.0005, 0.14649, 0.0026, 0.00479, 0.00000015); // resolution
    else if(case_num == 4) set_data(-0.00107, -0.0005, -0.14649, -0.00122, -0.00479, -0.00000015); // resolution
    else if(case_num == 5) set_data(0.1, 0.1, 0.1, 0.1, 0.98067, 0.0000001); // random
    else if(case_num == 6) set_data(-0.1, -0.1, -0.1, -0.1, -0.98067, 0.0000001); // random
    else if(case_num == 7) set_data(1.0, 1.0, 1.0, 1.0, 9.80665, 0.000001); // random
    else if(case_num == 8) set_data(-1.0, -1.0, -1.0, -1.0, -9.80665, -0.000001); // random
    else if(case_num == 9) set_data(12.34567, 12.34567, 12.34567, 12.34567, 121.06966, 0.00001); // random
    else if(case_num == 10) set_data(-12.34567, -12.34567, -12.34567, -12.34567, -121.06966, -0.00001); // random
  }

private:
  void set_data (double gyro_val, double acc_val, double mag_val, double temp_val,
                 double ans_acc_val, double ans_mag_val) {
    gyro[0] = gyro[1] = gyro[2] = gyro_val;
    acc[0] = acc[1] = acc[2] = acc_val;
    mag[0] = mag[1] = mag[2] = mag_val;
    temp = temp_val;
    ans_gyro[0] = ans_gyro[1] = ans_gyro[2] = gyro_val;
    ans_acc[0] = ans_acc[1] = ans_acc[2] = ans_acc_val;
    ans_mag[0] = ans_mag[1] = ans_mag[2] = ans_mag_val;
    ans_temp = temp_val;
  }
};

Mock<SerialPort> create_serial_port_mock(void) {
  Mock<SerialPort> mock;
  When(Method(mock, setPort)).AlwaysReturn();
  When(Method(mock, openPort)).AlwaysReturn(true);
  When(Method(mock, openSerialPort)).AlwaysReturn(true);
  When(Method(mock, closeSerialPort)).AlwaysReturn();
  When(Method(mock, readFromDevice)).AlwaysReturn(0);
  When(Method(mock, writeToDevice)).AlwaysReturn(0);
  return mock;
}

unsigned char int16_to_byte_low_8bit(int16_t val) {
  return (val >> 0) & 0xFF;
}

unsigned char int16_to_byte_high_8bit(int16_t val) {
  return (val >> 8) & 0xFF;
}

unsigned int create_dummy_bin_imu_data(unsigned char *buf, bool is_invalid) {
  ReadBinaryTestParam data(0);
  return create_dummy_bin_imu_data(buf, is_invalid, data.gyro, data.acc, data.mag, data.temp);
}

unsigned int create_dummy_bin_imu_data(unsigned char *buf, bool is_invalid,
                                       int16_t *gyro, int16_t *acc, int16_t *mag, int16_t temp) {
  rt_usb_9axisimu::Consts consts;
  unsigned char dummy_bin_imu_data[consts.IMU_BIN_DATA_SIZE] = {0};
  dummy_bin_imu_data[consts.IMU_BIN_HEADER_FF0] = 0xff;
  dummy_bin_imu_data[consts.IMU_BIN_HEADER_FF1] = 0xff;
  if (is_invalid) {
    dummy_bin_imu_data[consts.IMU_BIN_HEADER_R] = 0x54; // T
    dummy_bin_imu_data[consts.IMU_BIN_HEADER_T] = 0x52; // R
  } else {
    dummy_bin_imu_data[consts.IMU_BIN_HEADER_R] = 0x52; // R
    dummy_bin_imu_data[consts.IMU_BIN_HEADER_T] = 0x54; // T
  }
  dummy_bin_imu_data[consts.IMU_BIN_HEADER_ID0] = 0x39;
  dummy_bin_imu_data[consts.IMU_BIN_HEADER_ID1] = 0x41;
  dummy_bin_imu_data[consts.IMU_BIN_FIRMWARE] = 0x12;
  dummy_bin_imu_data[consts.IMU_BIN_TIMESTAMP] = 0x00;
  dummy_bin_imu_data[consts.IMU_BIN_ACC_X_L] = int16_to_byte_low_8bit(acc[0]);
  dummy_bin_imu_data[consts.IMU_BIN_ACC_X_H] = int16_to_byte_high_8bit(acc[0]);
  dummy_bin_imu_data[consts.IMU_BIN_ACC_Y_L] = int16_to_byte_low_8bit(acc[1]);
  dummy_bin_imu_data[consts.IMU_BIN_ACC_Y_H] = int16_to_byte_high_8bit(acc[1]);
  dummy_bin_imu_data[consts.IMU_BIN_ACC_Z_L] = int16_to_byte_low_8bit(acc[2]);
  dummy_bin_imu_data[consts.IMU_BIN_ACC_Z_H] = int16_to_byte_high_8bit(acc[2]);
  dummy_bin_imu_data[consts.IMU_BIN_TEMP_L] = int16_to_byte_low_8bit(temp);
  dummy_bin_imu_data[consts.IMU_BIN_TEMP_H] = int16_to_byte_high_8bit(temp);
  dummy_bin_imu_data[consts.IMU_BIN_GYRO_X_L] = int16_to_byte_low_8bit(gyro[0]);
  dummy_bin_imu_data[consts.IMU_BIN_GYRO_X_H] = int16_to_byte_high_8bit(gyro[0]);
  dummy_bin_imu_data[consts.IMU_BIN_GYRO_Y_L] = int16_to_byte_low_8bit(gyro[1]);
  dummy_bin_imu_data[consts.IMU_BIN_GYRO_Y_H] = int16_to_byte_high_8bit(gyro[1]);
  dummy_bin_imu_data[consts.IMU_BIN_GYRO_Z_L] = int16_to_byte_low_8bit(gyro[2]);
  dummy_bin_imu_data[consts.IMU_BIN_GYRO_Z_H] = int16_to_byte_high_8bit(gyro[2]);
  dummy_bin_imu_data[consts.IMU_BIN_MAG_X_L] = int16_to_byte_low_8bit(mag[0]);
  dummy_bin_imu_data[consts.IMU_BIN_MAG_X_H] = int16_to_byte_high_8bit(mag[0]);
  dummy_bin_imu_data[consts.IMU_BIN_MAG_Y_L] = int16_to_byte_low_8bit(mag[1]);
  dummy_bin_imu_data[consts.IMU_BIN_MAG_Y_H] = int16_to_byte_high_8bit(mag[1]);
  dummy_bin_imu_data[consts.IMU_BIN_MAG_Z_L] = int16_to_byte_low_8bit(mag[2]);
  dummy_bin_imu_data[consts.IMU_BIN_MAG_Z_H] = int16_to_byte_high_8bit(mag[2]);
  for(int i = 0; i < consts.IMU_BIN_DATA_SIZE; i++) {
    buf[i] = dummy_bin_imu_data[i];
  }
  return consts.IMU_BIN_DATA_SIZE;
}

std::string double_to_string(double val) {
  std::string str = std::to_string(val);
  str.resize(8, '0');
  return str;
}

unsigned int create_dummy_ascii_imu_data(unsigned char *buf, bool is_invalid) {
  ReadAsciiTestParam data(0);
  return create_dummy_ascii_imu_data(buf, is_invalid, data.gyro, data.acc, data.mag, data.temp);
}

unsigned int create_dummy_ascii_imu_data(unsigned char *buf, bool is_invalid,
                                         double *gyro, double *acc, double *mag, double temp) {
  rt_usb_9axisimu::Consts consts;
  std::vector<const char*> dummy_ascii_imu_data(consts.IMU_ASCII_DATA_SIZE); 
  if (is_invalid) {
    dummy_ascii_imu_data[consts.IMU_ASCII_TIMESTAMP] = "0.0";
  } else {
    dummy_ascii_imu_data[consts.IMU_ASCII_TIMESTAMP] = "0";
  }
  dummy_ascii_imu_data[consts.IMU_ASCII_GYRO_X] = double_to_string(gyro[0]).c_str();
  dummy_ascii_imu_data[consts.IMU_ASCII_GYRO_Y] = double_to_string(gyro[1]).c_str();
  dummy_ascii_imu_data[consts.IMU_ASCII_GYRO_Z] = double_to_string(gyro[2]).c_str();
  dummy_ascii_imu_data[consts.IMU_ASCII_ACC_X] = double_to_string(acc[0]).c_str();
  dummy_ascii_imu_data[consts.IMU_ASCII_ACC_Y] = double_to_string(acc[1]).c_str();
  dummy_ascii_imu_data[consts.IMU_ASCII_ACC_Z] = double_to_string(acc[2]).c_str();
  dummy_ascii_imu_data[consts.IMU_ASCII_MAG_X] = double_to_string(mag[0]).c_str();
  dummy_ascii_imu_data[consts.IMU_ASCII_MAG_Y] = double_to_string(mag[1]).c_str();
  dummy_ascii_imu_data[consts.IMU_ASCII_MAG_Z] = double_to_string(mag[2]).c_str();
  dummy_ascii_imu_data[consts.IMU_ASCII_TEMP] = double_to_string(temp).c_str();
  const char split_char = ',';
  const char newline_char = '\n';
  buf[0] = (unsigned char)newline_char;
  unsigned int char_count = 1;
  for(int i = 0; i < consts.IMU_ASCII_DATA_SIZE; i++) {
    for(int j = 0; j < (int)strlen(dummy_ascii_imu_data.at(i)); j++) {
      buf[char_count] = (unsigned char)dummy_ascii_imu_data.at(i)[j];
      char_count++;
    }
    if(i != consts.IMU_ASCII_DATA_SIZE - 1) buf[char_count] = (unsigned char)split_char;
    else buf[char_count] = (unsigned char)newline_char;
    char_count++;
  }
  return char_count;
}

TEST(TestDriver, startCommunication)
{
  // Expect the startCommunication method to be called twice and return true then false
  auto mock = create_serial_port_mock();
  When(Method(mock, openSerialPort)).Return(true, false);  // Return true then false

  RtUsb9axisimuRosDriver driver(
    std::unique_ptr<SerialPort>(&mock.get()));

  EXPECT_TRUE(driver.startCommunication());
  EXPECT_FALSE(driver.startCommunication());
}

TEST(TestDriver, initialize_member_variables)
{
  // Expect member variables of the driver instance to be initialised
  auto mock = create_serial_port_mock();

  RtUsb9axisimuRosDriver driver(
    std::unique_ptr<SerialPort>(&mock.get()));

  EXPECT_FALSE(driver.hasBinaryDataFormat());
  EXPECT_FALSE(driver.hasAsciiDataFormat());
  EXPECT_FALSE(driver.hasRefreshedImuData());
}

TEST(TestDriver, checkDataFormat_Binary)
{
  // Expect to check correctly when read data in binary format
  auto mock = create_serial_port_mock();

  // 1st: invalid binary data ('R' and 'T' positions are reversed)
  // 2nd: correct binary data ('R' and 'T' are in the correct position)
  When(Method(mock, readFromDevice)).Do([](
    unsigned char* buf, unsigned int buf_size) {
    buf_size = create_dummy_bin_imu_data(buf, true);
    return buf_size;
  }).Do([](
    unsigned char* buf, unsigned int buf_size) {
    buf_size = create_dummy_bin_imu_data(buf, false);
    return buf_size;
  });

  RtUsb9axisimuRosDriver driver(
    std::unique_ptr<SerialPort>(&mock.get()));

  driver.checkDataFormat();

  EXPECT_TRUE(driver.hasBinaryDataFormat());
  EXPECT_FALSE(driver.hasAsciiDataFormat());
}

TEST(TestDriver, checkDataFormat_ASCII)
{
  // Expect to check correctly when read data in ASCII format
  auto mock = create_serial_port_mock();

  // 1st: invalid ascii data (timestamp is double)
  // 2nd: correct ascii data (timestamp is int)
  When(Method(mock, readFromDevice)).Do([](
    unsigned char* buf, unsigned int buf_size) {
    buf_size = create_dummy_ascii_imu_data(buf, true);
    return buf_size;
  }).Do([](
    unsigned char* buf, unsigned int buf_size) {
    buf_size = create_dummy_ascii_imu_data(buf, false);
    return buf_size;
  });

  RtUsb9axisimuRosDriver driver(
    std::unique_ptr<SerialPort>(&mock.get()));

  driver.checkDataFormat();

  EXPECT_TRUE(driver.hasAsciiDataFormat());
  EXPECT_FALSE(driver.hasBinaryDataFormat());
}

TEST(TestDriver, checkDataFormat_not_Binary_or_ASCII)
{
  // Expect to check correctly when read data in not Binary or ASCII format
  auto mock = create_serial_port_mock();

  // always invalid data (not binary or ascii)
  When(Method(mock, readFromDevice)).AlwaysDo([](
    unsigned char* buf, unsigned int buf_size) {
    unsigned char dummy_data_not_binary_or_ascii[] =
      "dummy_data_not_binary_or_ascii";
    for(int i = 0; i < (int)sizeof(dummy_data_not_binary_or_ascii); i++) {
      buf[i] = dummy_data_not_binary_or_ascii[i];
    }
    buf_size = strlen((char*)buf);
    return buf_size;
  });

  RtUsb9axisimuRosDriver driver(
    std::unique_ptr<SerialPort>(&mock.get()));

  driver.checkDataFormat();

  EXPECT_FALSE(driver.hasBinaryDataFormat());
  EXPECT_FALSE(driver.hasAsciiDataFormat());
}

TEST(TestDriver, readSensorData_Binary)
{
  // Expect to check the data is correctly updated when binary data is read
  auto mock = create_serial_port_mock();

  // 1st: correct binary data ('R' and 'T' are in the correct position)
  // 2nd: invalid binary data ('R' and 'T' positions are reversed)
  // 3rd: correct binary data ('R' and 'T' are in the correct position)
  When(Method(mock, readFromDevice)).Do([](
    unsigned char* buf, unsigned int buf_size) {
    buf_size = create_dummy_bin_imu_data(buf, false);
    return buf_size;
  }).Do([](
    unsigned char* buf, unsigned int buf_size) {
    buf_size = create_dummy_bin_imu_data(buf, true);
    return buf_size;
  }).Do([](
    unsigned char* buf, unsigned int buf_size) {
    buf_size = create_dummy_bin_imu_data(buf, false);
    return buf_size;
  });

  RtUsb9axisimuRosDriver driver(
    std::unique_ptr<SerialPort>(&mock.get()));

  driver.checkDataFormat(); // 1st
  driver.readSensorData(); // 2nd

  EXPECT_FALSE(driver.hasRefreshedImuData());

  driver.readSensorData(); // 3rd

  EXPECT_TRUE(driver.hasRefreshedImuData());
}

TEST(TestDriver, readSensorData_ASCII)
{
  // Expect to check the data is correctly updated when ascii data is read
  auto mock = create_serial_port_mock();

  // 1st: correct ascii data (timestamp is int)
  // 2nd: invalid ascii data (timestamp is double)
  // 3rd: correct ascii data (timestamp is int)
  When(Method(mock, readFromDevice)).Do([](
    unsigned char* buf, unsigned int buf_size) {
    buf_size = create_dummy_ascii_imu_data(buf, false);
    return buf_size;
  }).Do([](
    unsigned char* buf, unsigned int buf_size) {
    buf_size = create_dummy_ascii_imu_data(buf, true);
    return buf_size;
  }).Do([](
    unsigned char* buf, unsigned int buf_size) {
    buf_size = create_dummy_ascii_imu_data(buf, false);
    return buf_size;
  });

  RtUsb9axisimuRosDriver driver(
    std::unique_ptr<SerialPort>(&mock.get()));

  driver.checkDataFormat(); // 1st
  driver.readSensorData(); // 2nd

  EXPECT_FALSE(driver.hasRefreshedImuData());

  driver.readSensorData(); // 3rd

  EXPECT_TRUE(driver.hasRefreshedImuData());
}

class ReadBinaryTest : public testing::TestWithParam<ReadBinaryTestParam> {
};

TEST_P(ReadBinaryTest, read_binary_test) {
  // Expect to check the data is correctly converted when binary data is read
  auto mock = create_serial_port_mock();
  auto data = GetParam();

  RtUsb9axisimuRosDriver driver(
    std::unique_ptr<SerialPort>(&mock.get()));

  When(Method(mock, readFromDevice)).AlwaysDo([&](
    unsigned char* buf, unsigned int buf_size) {
    buf_size = create_dummy_bin_imu_data(buf, false, data.gyro, data.acc, data.mag, data.temp);
    return buf_size;
  });

  driver.checkDataFormat();
  driver.readSensorData();

  rclcpp::Time timestamp;
  auto imu_data_raw = driver.getImuRawDataUniquePtr(timestamp);
  auto imu_data_mag = driver.getImuMagUniquePtr(timestamp);
  auto imu_data_temperature = driver.getImuTemperatureUniquePtr();

  const double abs_error_acc = 1e-3;
  const double abs_error_gyro = 1e-3;
  const double abs_error_mag = 1e-5;
  const double abs_error_temp = 1e-3;
  EXPECT_NEAR(imu_data_raw->linear_acceleration.x, data.ans_acc[0], abs_error_acc);
  EXPECT_NEAR(imu_data_raw->linear_acceleration.y, data.ans_acc[1], abs_error_acc);
  EXPECT_NEAR(imu_data_raw->linear_acceleration.z, data.ans_acc[2], abs_error_acc);
  EXPECT_NEAR(imu_data_raw->angular_velocity.x, data.ans_gyro[0], abs_error_gyro);
  EXPECT_NEAR(imu_data_raw->angular_velocity.y, data.ans_gyro[1], abs_error_gyro);
  EXPECT_NEAR(imu_data_raw->angular_velocity.z, data.ans_gyro[2], abs_error_gyro);
  EXPECT_NEAR(imu_data_mag->magnetic_field.x, data.ans_mag[0], abs_error_mag);
  EXPECT_NEAR(imu_data_mag->magnetic_field.y, data.ans_mag[1], abs_error_mag);
  EXPECT_NEAR(imu_data_mag->magnetic_field.z, data.ans_mag[2], abs_error_mag);
  EXPECT_NEAR(imu_data_temperature->data, data.ans_temp, abs_error_temp);
}

INSTANTIATE_TEST_SUITE_P(
  TestDriver,
  ReadBinaryTest,
  ::testing::Values(
    ReadBinaryTestParam(0),
    ReadBinaryTestParam(1),
    ReadBinaryTestParam(2),
    ReadBinaryTestParam(3),
    ReadBinaryTestParam(4),
    ReadBinaryTestParam(5),
    ReadBinaryTestParam(6),
    ReadBinaryTestParam(7),
    ReadBinaryTestParam(8),
    ReadBinaryTestParam(9),
    ReadBinaryTestParam(10)
  )
);

class ReadAsciiTest : public testing::TestWithParam<ReadAsciiTestParam> {
};

TEST_P(ReadAsciiTest, read_ascii_test) {
  // Expect to check the data is correctly converted when ascii data is read
  auto mock = create_serial_port_mock();
  auto data = GetParam();

  RtUsb9axisimuRosDriver driver(
    std::unique_ptr<SerialPort>(&mock.get()));

  When(Method(mock, readFromDevice)).AlwaysDo([&](
    unsigned char* buf, unsigned int buf_size) {
    buf_size = create_dummy_ascii_imu_data(buf, false, data.gyro, data.acc, data.mag, data.temp);
    return buf_size;
  });

  driver.checkDataFormat();
  driver.readSensorData();

  rclcpp::Time timestamp;
  auto imu_data_raw = driver.getImuRawDataUniquePtr(timestamp);
  auto imu_data_mag = driver.getImuMagUniquePtr(timestamp);
  auto imu_data_temperature = driver.getImuTemperatureUniquePtr();

  const double abs_error_acc = 1e-3;
  const double abs_error_gyro = 1e-3;
  const double abs_error_mag = 1e-5;
  const double abs_error_temp = 1e-3;
  EXPECT_NEAR(imu_data_raw->linear_acceleration.x, data.ans_acc[0], abs_error_acc);
  EXPECT_NEAR(imu_data_raw->linear_acceleration.y, data.ans_acc[1], abs_error_acc);
  EXPECT_NEAR(imu_data_raw->linear_acceleration.z, data.ans_acc[2], abs_error_acc);
  EXPECT_NEAR(imu_data_raw->angular_velocity.x, data.ans_gyro[0], abs_error_gyro);
  EXPECT_NEAR(imu_data_raw->angular_velocity.y, data.ans_gyro[1], abs_error_gyro);
  EXPECT_NEAR(imu_data_raw->angular_velocity.z, data.ans_gyro[2], abs_error_gyro);
  EXPECT_NEAR(imu_data_mag->magnetic_field.x, data.ans_mag[0], abs_error_mag);
  EXPECT_NEAR(imu_data_mag->magnetic_field.y, data.ans_mag[1], abs_error_mag);
  EXPECT_NEAR(imu_data_mag->magnetic_field.z, data.ans_mag[2], abs_error_mag);
  EXPECT_NEAR(imu_data_temperature->data, data.ans_temp, abs_error_temp);
}

INSTANTIATE_TEST_SUITE_P(
  TestDriver,
  ReadAsciiTest,
  ::testing::Values(
    ReadAsciiTestParam(0),
    ReadAsciiTestParam(1),
    ReadAsciiTestParam(2),
    ReadAsciiTestParam(3),
    ReadAsciiTestParam(4),
    ReadAsciiTestParam(5),
    ReadAsciiTestParam(6),
    ReadAsciiTestParam(7),
    ReadAsciiTestParam(8),
    ReadAsciiTestParam(9),
    ReadAsciiTestParam(10)
  )
);