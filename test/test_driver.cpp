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
#include <iostream>

using fakeit::Mock;
using fakeit::When;
using rt_usb_9axisimu::SerialPort;

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

// Expect that short int is 2 bytes.
unsigned char short_int_to_byte(short int val, bool is_low) {
  if (is_low) return (val >> 0) & 0xFF;
  else return (val >> 8) & 0xFF;
}

unsigned int create_dummy_bin_imu_data(unsigned char *buf, bool is_invalid,
                                       short int *gyro, short int *acc, short int *mag, short int temp) {
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
  dummy_bin_imu_data[consts.IMU_BIN_ACC_X_L] = short_int_to_byte(acc[0], true);
  dummy_bin_imu_data[consts.IMU_BIN_ACC_X_H] = short_int_to_byte(acc[0], false);
  dummy_bin_imu_data[consts.IMU_BIN_ACC_Y_L] = short_int_to_byte(acc[1], true);
  dummy_bin_imu_data[consts.IMU_BIN_ACC_Y_H] = short_int_to_byte(acc[1], false);
  dummy_bin_imu_data[consts.IMU_BIN_ACC_Z_L] = short_int_to_byte(acc[2], true);
  dummy_bin_imu_data[consts.IMU_BIN_ACC_Z_H] = short_int_to_byte(acc[2], false);
  dummy_bin_imu_data[consts.IMU_BIN_TEMP_L] = short_int_to_byte(temp, true);
  dummy_bin_imu_data[consts.IMU_BIN_TEMP_H] = short_int_to_byte(temp, false);
  dummy_bin_imu_data[consts.IMU_BIN_GYRO_X_L] = short_int_to_byte(gyro[0], true);
  dummy_bin_imu_data[consts.IMU_BIN_GYRO_X_H] = short_int_to_byte(gyro[0], false);
  dummy_bin_imu_data[consts.IMU_BIN_GYRO_Y_L] = short_int_to_byte(gyro[1], true);
  dummy_bin_imu_data[consts.IMU_BIN_GYRO_Y_H] = short_int_to_byte(gyro[1], false);
  dummy_bin_imu_data[consts.IMU_BIN_GYRO_Z_L] = short_int_to_byte(gyro[2], true);
  dummy_bin_imu_data[consts.IMU_BIN_GYRO_Z_H] = short_int_to_byte(gyro[2], false);
  dummy_bin_imu_data[consts.IMU_BIN_MAG_X_L] = short_int_to_byte(mag[0], true);
  dummy_bin_imu_data[consts.IMU_BIN_MAG_X_H] = short_int_to_byte(mag[0], false);
  dummy_bin_imu_data[consts.IMU_BIN_MAG_Y_L] = short_int_to_byte(mag[1], true);
  dummy_bin_imu_data[consts.IMU_BIN_MAG_Y_H] = short_int_to_byte(mag[1], false);
  dummy_bin_imu_data[consts.IMU_BIN_MAG_Z_L] = short_int_to_byte(mag[2], true);
  dummy_bin_imu_data[consts.IMU_BIN_MAG_Z_H] = short_int_to_byte(mag[2], false);
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
    short int gyro[] = {0, 0, 0};
    short int acc[] = {0, 0, 0};
    short int mag[] = {0, 0, 0};
    short int temp = 0;
    buf_size = create_dummy_bin_imu_data(buf, true, gyro, acc, mag, temp);
    return buf_size;
  }).Do([](
    unsigned char* buf, unsigned int buf_size) {
    short int gyro[] = {0, 0, 0};
    short int acc[] = {0, 0, 0};
    short int mag[] = {0, 0, 0};
    short int temp = 0;
    buf_size = create_dummy_bin_imu_data(buf, false, gyro, acc, mag, temp);
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
    double gyro[] = {0.0, 0.0, 0.0};
    double acc[] = {0.0, 0.0, 0.0};
    double mag[] = {0.0, 0.0, 0.0};
    double temp = 0.0;
    buf_size = create_dummy_ascii_imu_data(buf, true, gyro, acc, mag, temp);
    return buf_size;
  }).Do([](
    unsigned char* buf, unsigned int buf_size) {
    double gyro[] = {0.0, 0.0, 0.0};
    double acc[] = {0.0, 0.0, 0.0};
    double mag[] = {0.0, 0.0, 0.0};
    double temp = 0.0;
    buf_size = create_dummy_ascii_imu_data(buf, false, gyro, acc, mag, temp);
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

  // 1st: invalid binary data ('R' and 'T' positions are reversed)
  // 2nd: correct binary data ('R' and 'T' are in the correct position)
  When(Method(mock, readFromDevice)).Do([](
    unsigned char* buf, unsigned int buf_size) {
    short int gyro[] = {0, 0, 0};
    short int acc[] = {0, 0, 0};
    short int mag[] = {0, 0, 0};
    short int temp = 0;
    buf_size = create_dummy_bin_imu_data(buf, true, gyro, acc, mag, temp);
    return buf_size;
  }).Do([](
    unsigned char* buf, unsigned int buf_size) {
    short int gyro[] = {0, 0, 0};
    short int acc[] = {0, 0, 0};
    short int mag[] = {0, 0, 0};
    short int temp = 0;
    buf_size = create_dummy_bin_imu_data(buf, false, gyro, acc, mag, temp);
    return buf_size;
  });

  RtUsb9axisimuRosDriver driver(
    std::unique_ptr<SerialPort>(&mock.get()));

  driver.setBinaryFormat();
  driver.readSensorData();

  EXPECT_FALSE(driver.hasRefreshedImuData());

  driver.readSensorData();

  EXPECT_TRUE(driver.hasRefreshedImuData());
}

TEST(TestDriver, readSensorData_ASCII)
{
  // Expect to check the data is correctly updated when ascii data is read
  auto mock = create_serial_port_mock();

  // 1st: invalid ascii data (timestamp is double)
  // 2nd: correct ascii data (timestamp is int)
  When(Method(mock, readFromDevice)).Do([](
    unsigned char* buf, unsigned int buf_size) {
    double gyro[] = {0.0, 0.0, 0.0};
    double acc[] = {0.0, 0.0, 0.0};
    double mag[] = {0.0, 0.0, 0.0};
    double temp = 0.0;
    buf_size = create_dummy_ascii_imu_data(buf, true, gyro, acc, mag, temp);
    return buf_size;
  }).Do([](
    unsigned char* buf, unsigned int buf_size) {
    double gyro[] = {0.0, 0.0, 0.0};
    double acc[] = {0.0, 0.0, 0.0};
    double mag[] = {0.0, 0.0, 0.0};
    double temp = 0.0;
    buf_size = create_dummy_ascii_imu_data(buf, false, gyro, acc, mag, temp);
    return buf_size;
  });

  RtUsb9axisimuRosDriver driver(
    std::unique_ptr<SerialPort>(&mock.get()));

  driver.setAsciiFormat();
  driver.readSensorData();

  EXPECT_FALSE(driver.hasRefreshedImuData());

  driver.readSensorData();

  EXPECT_TRUE(driver.hasRefreshedImuData());
}

TEST(TestDriver, check_convert_when_read_Binary) {
  // Expect to check the data is correctly converted when binary data is read
  auto mock = create_serial_port_mock();

  RtUsb9axisimuRosDriver driver(
    std::unique_ptr<SerialPort>(&mock.get()));

  driver.setBinaryFormat();

  rclcpp::Time timestamp;
  const double abs_error = 1e-9;

  // case 1: zero
  When(Method(mock, readFromDevice)).Do([](
    unsigned char* buf, unsigned int buf_size) {
    short int gyro[] = {0, 0, 0};
    short int acc[] = {0, 0, 0};
    short int mag[] = {0, 0, 0};
    short int temp = 0;
    buf_size = create_dummy_bin_imu_data(buf, true, gyro, acc, mag, temp);
    return buf_size;
  });

  auto imu_data_raw = driver.getImuRawDataUniquePtr(timestamp);
  auto imu_data_mag = driver.getImuMagUniquePtr(timestamp);
  auto imu_data_temperature = driver.getImuTemperatureUniquePtr();

  const double zero = 0.0;
  EXPECT_NEAR(imu_data_raw->linear_acceleration.x, zero, abs_error);
  EXPECT_NEAR(imu_data_raw->linear_acceleration.y, zero, abs_error);
  EXPECT_NEAR(imu_data_raw->linear_acceleration.z, zero, abs_error);
  EXPECT_NEAR(imu_data_raw->angular_velocity.x, zero, abs_error);
  EXPECT_NEAR(imu_data_raw->angular_velocity.y, zero, abs_error);
  EXPECT_NEAR(imu_data_raw->angular_velocity.z, zero, abs_error);
  EXPECT_NEAR(imu_data_mag->magnetic_field.x, zero, abs_error);
  EXPECT_NEAR(imu_data_mag->magnetic_field.y, zero, abs_error);
  EXPECT_NEAR(imu_data_mag->magnetic_field.z, zero, abs_error);
  EXPECT_NEAR(imu_data_temperature->data, zero, abs_error);
}

TEST(TestDriver, check_convert_when_read_ASCII) {
  // Expect to check the data is correctly converted when ascii data is read
  auto mock = create_serial_port_mock();

  RtUsb9axisimuRosDriver driver(
    std::unique_ptr<SerialPort>(&mock.get()));

  driver.setAsciiFormat();

  rclcpp::Time timestamp;
  const double abs_error = 1e-9;

  // case 1: zero
  When(Method(mock, readFromDevice)).Do([](
    unsigned char* buf, unsigned int buf_size) {
    double gyro[] = {0.0, 0.0, 0.0};
    double acc[] = {0.0, 0.0, 0.0};
    double mag[] = {0.0, 0.0, 0.0};
    double temp = 0.0;
    buf_size = create_dummy_ascii_imu_data(buf, false, gyro, acc, mag, temp);
    return buf_size;
  });

  auto imu_data_raw = driver.getImuRawDataUniquePtr(timestamp);
  auto imu_data_mag = driver.getImuMagUniquePtr(timestamp);
  auto imu_data_temperature = driver.getImuTemperatureUniquePtr();

  const double zero = 0.0;
  EXPECT_NEAR(imu_data_raw->linear_acceleration.x, zero, abs_error);
  EXPECT_NEAR(imu_data_raw->linear_acceleration.y, zero, abs_error);
  EXPECT_NEAR(imu_data_raw->linear_acceleration.z, zero, abs_error);
  EXPECT_NEAR(imu_data_raw->angular_velocity.x, zero, abs_error);
  EXPECT_NEAR(imu_data_raw->angular_velocity.y, zero, abs_error);
  EXPECT_NEAR(imu_data_raw->angular_velocity.z, zero, abs_error);
  EXPECT_NEAR(imu_data_mag->magnetic_field.x, zero, abs_error);
  EXPECT_NEAR(imu_data_mag->magnetic_field.y, zero, abs_error);
  EXPECT_NEAR(imu_data_mag->magnetic_field.z, zero, abs_error);
  EXPECT_NEAR(imu_data_temperature->data, zero, abs_error);
}