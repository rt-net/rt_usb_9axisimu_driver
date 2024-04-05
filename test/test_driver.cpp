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

#include <array>
#include <string>
#include <gtest/gtest.h>
#include "fakeit.hpp"
#include "rt_usb_9axisimu_driver/rt_usb_9axisimu_driver.hpp"
#include "rt_usb_9axisimu_driver/rt_usb_9axisimu.hpp"

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

  EXPECT_FALSE(driver.hasCompletedFormatCheck());
  EXPECT_FALSE(driver.hasBinaryDataFormat());
  EXPECT_FALSE(driver.hasAsciiDataFormat());
  EXPECT_FALSE(driver.hasRefreshedImuData());
}

TEST(TestDriver, checkDataFormat_Binary)
{
  // Expect to check correctly when read data in binary format
  auto mock = create_serial_port_mock();

  When(Method(mock, readFromDevice)).Do([](
    unsigned char* buf, unsigned int buf_size) {
    rt_usb_9axisimu::Consts consts;
    unsigned char dummy_bin_imu_data[consts.IMU_BIN_DATA_SIZE] = {0};
    dummy_bin_imu_data[consts.IMU_BIN_HEADER_R] = 0x52; // R
    dummy_bin_imu_data[consts.IMU_BIN_HEADER_T] = 0x54; // T
    for(int i = 0; i < consts.IMU_BIN_DATA_SIZE; i++) {
      buf[i] = dummy_bin_imu_data[i];
    }
    buf_size = consts.IMU_BIN_DATA_SIZE;
    return buf_size;
  });

  RtUsb9axisimuRosDriver driver(
    std::unique_ptr<SerialPort>(&mock.get()));

  driver.checkDataFormat();

  EXPECT_TRUE(driver.hasCompletedFormatCheck());
  EXPECT_TRUE(driver.hasBinaryDataFormat());
  EXPECT_FALSE(driver.hasAsciiDataFormat());
}

TEST(TestDriver, checkDataFormat_ASCII)
{
  // Expect to check correctly when read data in ASCII format
  auto mock = create_serial_port_mock();

  When(Method(mock, readFromDevice)).Do([](
    unsigned char* buf, unsigned int buf_size) {
    rt_usb_9axisimu::Consts consts;
    std::array<std::string, consts.IMU_ASCII_DATA_SIZE> dummy_ascii_imu_data; 
    dummy_ascii_imu_data[consts.IMU_ASCII_TIMESTAMP] = "0";
    dummy_ascii_imu_data[consts.IMU_ASCII_GYRO_X] = "0.000000";
    dummy_ascii_imu_data[consts.IMU_ASCII_GYRO_Y] = "0.000000";
    dummy_ascii_imu_data[consts.IMU_ASCII_GYRO_Z] = "0.000000";
    dummy_ascii_imu_data[consts.IMU_ASCII_ACC_X] = "0.000000";
    dummy_ascii_imu_data[consts.IMU_ASCII_ACC_Y] = "0.000000";
    dummy_ascii_imu_data[consts.IMU_ASCII_ACC_Z] = "0.000000";
    dummy_ascii_imu_data[consts.IMU_ASCII_MAG_X] = "0.000000";
    dummy_ascii_imu_data[consts.IMU_ASCII_MAG_Y] = "0.000000";
    dummy_ascii_imu_data[consts.IMU_ASCII_MAG_Z] = "0.000000";
    dummy_ascii_imu_data[consts.IMU_ASCII_TEMP] = "0.000000";
    std::string str = "";
    std::string split_char = ",";
    for(int i = 0; i < consts.IMU_ASCII_DATA_SIZE; i++) {
      str += dummy_ascii_imu_data[i];
      if(i != consts.IMU_ASCII_DATA_SIZE - 1) str += split_char;
    }
    for(int i = 0; i < int(str.length()); i++) {
      buf[i] = str[i];
    }
    buf_size = consts.IMU_BIN_DATA_SIZE;
    return buf_size;
  });

  RtUsb9axisimuRosDriver driver(
    std::unique_ptr<SerialPort>(&mock.get()));

  driver.checkDataFormat();
  driver.checkDataFormat();

  EXPECT_TRUE(driver.hasCompletedFormatCheck());
  EXPECT_TRUE(driver.hasAsciiDataFormat());
  EXPECT_FALSE(driver.hasBinaryDataFormat());
}

TEST(TestDriver, checkDataFormat_not_Binary_or_ASCII)
{
  // Expect to check correctly when read data in not Binary or ASCII format
  auto mock = create_serial_port_mock();

  When(Method(mock, readFromDevice)).Do([](
    unsigned char* buf, unsigned int buf_size) {
    rt_usb_9axisimu::Consts consts;
    unsigned char dummy_data_not_binary_or_ascii[] =
      "dummy_data_not_binary_or_ascii";
    for(int i = 0; i < int(sizeof(dummy_data_not_binary_or_ascii)); i++) {
      buf[i] = dummy_data_not_binary_or_ascii[i];
    }
    buf_size = consts.IMU_BIN_DATA_SIZE;
    return buf_size;
  });

  RtUsb9axisimuRosDriver driver(
    std::unique_ptr<SerialPort>(&mock.get()));

  driver.checkDataFormat();

  EXPECT_FALSE(driver.hasCompletedFormatCheck());
  EXPECT_FALSE(driver.hasBinaryDataFormat());
  EXPECT_FALSE(driver.hasAsciiDataFormat());
}
