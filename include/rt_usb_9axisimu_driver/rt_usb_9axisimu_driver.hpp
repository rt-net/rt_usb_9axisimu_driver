/*
 * rt_usb_9axisimu_driver.hpp
 *
 * License: BSD-3-Clause
 *
 * Copyright (c) 2015-2020 RT Corporation <support@rt-net.jp>
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

#ifndef RT_USB_9AXISIMU_DRIVER__RT_USB_9AXISIMU_DRIVER_HPP_
#define RT_USB_9AXISIMU_DRIVER__RT_USB_9AXISIMU_DRIVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rt_usb_9axisimu_driver/rt_usb_9axisimu.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/float64.hpp"

class RtUsb9axisimuRosDriver
{
public:
  explicit RtUsb9axisimuRosDriver(std::string serialport);
  RtUsb9axisimuRosDriver(std::unique_ptr<rt_usb_9axisimu::SerialPort> serial_port);
  ~RtUsb9axisimuRosDriver();

  enum ReadStatus
  {
    SUCCESS = 0,
    NEED_TO_CONTINUE,
    FAILURE
  };

  void setImuFrameIdName(std::string frame_id);
  void setImuPortName(std::string port);
  void setImuStdDev(double linear_acceleration, double angular_velocity, double magnetic_field);

  bool startCommunication();
  void stopCommunication(void);
  void checkDataFormat(const double timeout = 5.0);
  bool hasAsciiDataFormat(void);
  bool hasBinaryDataFormat(void);
  bool hasRefreshedImuData(void);

  std::unique_ptr<sensor_msgs::msg::Imu> getImuRawDataUniquePtr(const rclcpp::Time timestamp);
  std::unique_ptr<sensor_msgs::msg::MagneticField> getImuMagUniquePtr(const rclcpp::Time timestamp);
  std::unique_ptr<std_msgs::msg::Float64> getImuTemperatureUniquePtr(void);
  ReadStatus readSensorData();

private:
  std::unique_ptr<rt_usb_9axisimu::SerialPort> serial_port_;

  rt_usb_9axisimu::SensorData sensor_data_;

  std::string frame_id_;
  double linear_acceleration_stddev_;
  double angular_velocity_stddev_;
  double magnetic_field_stddev_;
  rt_usb_9axisimu::Consts consts;

  unsigned char bin_read_buffer_[rt_usb_9axisimu::Consts::READ_BUFFER_SIZE];
  unsigned char ascii_read_buffer_[rt_usb_9axisimu::Consts::READ_BUFFER_SIZE];
  unsigned int bin_read_buffer_idx_ = 0;
  unsigned int ascii_read_buffer_idx_ = 0;

  enum DataFormat
  {
    NONE = 0,
    NOT_BINARY,
    BINARY,
    NOT_ASCII,
    ASCII,
    INCORRECT
  };
  DataFormat data_format_;
  bool has_refreshed_imu_data_;

  // Method to combine two separate one-byte data into one two-byte data
  int16_t combineByteData(unsigned char data_h, unsigned char data_l);
  // Method to extract binary sensor data from communication buffer
  rt_usb_9axisimu::ImuData<int16_t> extractBinarySensorData(unsigned char * imu_data_buf);
  bool isBinarySensorData(unsigned char * imu_data_buf, unsigned int data_size);
  ReadStatus readBinaryData(void);
  bool isAsciiSensorData(unsigned char * imu_data_buf, unsigned int data_size);
  bool isValidAsciiSensorData(std::vector<std::string> imu_data_vector_buf);
  ReadStatus readAsciiData(void);
};

#endif  // RT_USB_9AXISIMU_DRIVER__RT_USB_9AXISIMU_DRIVER_HPP_
