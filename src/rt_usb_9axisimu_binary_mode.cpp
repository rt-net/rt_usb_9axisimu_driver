/*
 * rt_usb_9axisimu_binary_mode.cpp
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

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/Float64.h"
#include "rt_usb_9axisimu_driver/rt_usb_9axisimu_binary_mode.hpp"

bool RtUsb9axisimuBinaryModeRosDriver::readBinaryData(void)
{
  unsigned char imu_data_buf[256];
  rt_usb_9axisimu::ImuData<signed short> imu_rawdata;

  int data_size_of_buf = readFromDevice(imu_data_buf, consts.IMU_DATA_SIZE);

  if (data_size_of_buf < consts.IMU_DATA_SIZE)
  {
    if (data_size_of_buf <= 0)
    {
      return false;  // Raise communication error
    }
    return false;
  }

  if (isBinarySensorData(imu_data_buf) == false)
  {
    return false;
  }

  imu_rawdata = extractBinarySensorData(imu_data_buf);  // Extract sensor data

  sensor_data_.setImuRawData(imu_rawdata);  // Update raw data
  sensor_data_.convertRawDataUnit();        // Convert raw data to physical quantity

  return true;
}

bool RtUsb9axisimuBinaryModeRosDriver::readAsciiData(void)
{
  ROS_ERROR("readAsciiData is not implemented.");
}

RtUsb9axisimuBinaryModeRosDriver::RtUsb9axisimuBinaryModeRosDriver(std::string port = "")
  : rt_usb_9axisimu::SerialPort(port.c_str())
{
  // nh_priv_("~");
  // publisher for streaming
  imu_data_raw_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
  imu_mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
  imu_temperature_pub_ = nh_.advertise<std_msgs::Float64>("imu/temperature", 1);

  format_check_has_completed_ = false;
  data_format_ = DataFormat::NONE;
}

RtUsb9axisimuBinaryModeRosDriver::~RtUsb9axisimuBinaryModeRosDriver()
{
}

void RtUsb9axisimuBinaryModeRosDriver::setImuFrameIdName(std::string frame_id)
{
  frame_id_ = frame_id;
}

void RtUsb9axisimuBinaryModeRosDriver::setImuPortName(std::string serialport)
{
  rt_usb_9axisimu::SerialPort(serialport.c_str());
}

void RtUsb9axisimuBinaryModeRosDriver::setImuStdDev(double linear_acceleration, double angular_velocity,
                                                    double magnetic_field)
{
  linear_acceleration_stddev_ = linear_acceleration;
  angular_velocity_stddev_ = angular_velocity;
  magnetic_field_stddev_ = magnetic_field;
}

bool RtUsb9axisimuBinaryModeRosDriver::startCommunication()
{
  // returns serial port open status
  return openSerialPort();
}

void RtUsb9axisimuBinaryModeRosDriver::stopCommunication(void)
{
  closeSerialPort();
}

void RtUsb9axisimuBinaryModeRosDriver::checkDataFormat(void)
{
  if (data_format_ == DataFormat::NONE)
  {
    unsigned char data_buf[256];
    int data_size_of_buf = readFromDevice(data_buf, consts.IMU_DATA_SIZE);
    if (data_size_of_buf == consts.IMU_DATA_SIZE)
    {
      if (isBinarySensorData(data_buf))
      {
        data_format_ = DataFormat::BINARY;
        format_check_has_completed_ = true;
      }
      else
      {
        data_format_ = DataFormat::NOT_BINARY;
      }
    }
  }
  else if (data_format_ == DataFormat::NOT_BINARY)
  {
    data_format_ = DataFormat::ASCII;
    format_check_has_completed_ = true;
  }
}

bool RtUsb9axisimuBinaryModeRosDriver::formatCheckHasCompleted(void)
{
  return format_check_has_completed_;
}

bool RtUsb9axisimuBinaryModeRosDriver::hasCorrectDataFormat(void)
{
  bool output = true;
  if (data_format_ == DataFormat::INCORRECT || data_format_ == DataFormat::NOT_ASCII ||
      data_format_ == DataFormat::NOT_BINARY)
  {
    output = false;
  }
  return output;
}

bool RtUsb9axisimuBinaryModeRosDriver::hasAsciiDataFormat(void)
{
  return data_format_ == DataFormat::ASCII;
}

bool RtUsb9axisimuBinaryModeRosDriver::hasBinaryDataFormat(void)
{
  return data_format_ == DataFormat::BINARY;
}

// Method to combine two separate one-byte data into one two-byte data
signed short RtUsb9axisimuBinaryModeRosDriver::combineByteData(unsigned char data_h, unsigned char data_l)
{
  signed short short_data = 0;

  short_data = data_h;
  short_data = short_data << 8;
  short_data |= data_l;

  return short_data;
}

// Method to extract binary sensor data from communication buffer
rt_usb_9axisimu::ImuData<signed short>
RtUsb9axisimuBinaryModeRosDriver::extractBinarySensorData(unsigned char* imu_data_buf)
{
  rt_usb_9axisimu::ImuData<signed short> imu_rawdata;

  imu_rawdata.firmware_ver = imu_data_buf[consts.IMU_FIRMWARE];
  imu_rawdata.timestamp = imu_data_buf[consts.IMU_TIMESTAMP];
  imu_rawdata.temperature = combineByteData(imu_data_buf[consts.IMU_TEMP_H], imu_data_buf[consts.IMU_TEMP_L]);
  imu_rawdata.ax = combineByteData(imu_data_buf[consts.IMU_ACC_X_H], imu_data_buf[consts.IMU_ACC_X_L]);
  imu_rawdata.ay = combineByteData(imu_data_buf[consts.IMU_ACC_Y_H], imu_data_buf[consts.IMU_ACC_Y_L]);
  imu_rawdata.az = combineByteData(imu_data_buf[consts.IMU_ACC_Z_H], imu_data_buf[consts.IMU_ACC_Z_L]);
  imu_rawdata.gx = combineByteData(imu_data_buf[consts.IMU_GYRO_X_H], imu_data_buf[consts.IMU_GYRO_X_L]);
  imu_rawdata.gy = combineByteData(imu_data_buf[consts.IMU_GYRO_Y_H], imu_data_buf[consts.IMU_GYRO_Y_L]);
  imu_rawdata.gz = combineByteData(imu_data_buf[consts.IMU_GYRO_Z_H], imu_data_buf[consts.IMU_GYRO_Z_L]);
  imu_rawdata.mx = combineByteData(imu_data_buf[consts.IMU_MAG_X_H], imu_data_buf[consts.IMU_MAG_X_L]);
  imu_rawdata.my = combineByteData(imu_data_buf[consts.IMU_MAG_Y_H], imu_data_buf[consts.IMU_MAG_Y_L]);
  imu_rawdata.mz = combineByteData(imu_data_buf[consts.IMU_MAG_Z_H], imu_data_buf[consts.IMU_MAG_Z_L]);

  return imu_rawdata;
}

bool RtUsb9axisimuBinaryModeRosDriver::isBinarySensorData(unsigned char* imu_data_buf)
{
  bool is_binary_sensor_data = false;
  if (imu_data_buf[consts.IMU_HEADER_R] == 'R' && imu_data_buf[consts.IMU_HEADER_T] == 'T')
  {
    is_binary_sensor_data = true;
  }
  return is_binary_sensor_data;
}

bool RtUsb9axisimuBinaryModeRosDriver::publishSensorData()
{
  rt_usb_9axisimu::ImuData<double> imu;
  sensor_msgs::Imu imu_data_raw_msg;
  sensor_msgs::MagneticField imu_magnetic_msg;
  std_msgs::Float64 imu_temperature_msg;

  imu = sensor_data_.getImuData();  // Get phisical quantity

  // Calculate linear_acceleration_covariance diagonal elements
  double linear_acceleration_cov = linear_acceleration_stddev_ * linear_acceleration_stddev_;
  // Calculate angular_velocity_covariance diagonal elements
  double angular_velocity_cov = angular_velocity_stddev_ * angular_velocity_stddev_;
  // Calculate magnetic_field_covariance diagonal elements
  double magnetic_field_cov = magnetic_field_stddev_ * magnetic_field_stddev_;

  // imu_data_raw_msg has no orientation values
  imu_data_raw_msg.orientation_covariance[0] = -1;

  imu_data_raw_msg.linear_acceleration_covariance[0] = imu_data_raw_msg.linear_acceleration_covariance[4] =
      imu_data_raw_msg.linear_acceleration_covariance[8] = linear_acceleration_cov;

  imu_data_raw_msg.angular_velocity_covariance[0] = imu_data_raw_msg.angular_velocity_covariance[4] =
      imu_data_raw_msg.angular_velocity_covariance[8] = angular_velocity_cov;

  imu_magnetic_msg.magnetic_field_covariance[0] = imu_magnetic_msg.magnetic_field_covariance[4] =
      imu_magnetic_msg.magnetic_field_covariance[8] = magnetic_field_cov;

  ros::Time now = ros::Time::now();

  imu_data_raw_msg.header.stamp = imu_magnetic_msg.header.stamp = now;

  imu_data_raw_msg.header.frame_id = imu_magnetic_msg.header.frame_id = frame_id_;

  // original data used the g unit, convert to m/s^2
  imu_data_raw_msg.linear_acceleration.x = imu.ax * consts.CONVERTOR_G2A;
  imu_data_raw_msg.linear_acceleration.y = imu.ay * consts.CONVERTOR_G2A;
  imu_data_raw_msg.linear_acceleration.z = imu.az * consts.CONVERTOR_G2A;

  // original data used the degree/s unit, convert to radian/s
  imu_data_raw_msg.angular_velocity.x = imu.gx * consts.CONVERTOR_D2R;
  imu_data_raw_msg.angular_velocity.y = imu.gy * consts.CONVERTOR_D2R;
  imu_data_raw_msg.angular_velocity.z = imu.gz * consts.CONVERTOR_D2R;

  // original data used the uTesla unit, convert to Tesla
  imu_magnetic_msg.magnetic_field.x = imu.mx / consts.CONVERTOR_UT2T;
  imu_magnetic_msg.magnetic_field.y = imu.my / consts.CONVERTOR_UT2T;
  imu_magnetic_msg.magnetic_field.z = imu.mz / consts.CONVERTOR_UT2T;

  // original data used the celsius unit
  imu_temperature_msg.data = imu.temperature;

  // publish the IMU data
  imu_data_raw_pub_.publish(imu_data_raw_msg);
  imu_mag_pub_.publish(imu_magnetic_msg);
  imu_temperature_pub_.publish(imu_temperature_msg);

  return true;
}

// Method to receive IMU data, convert those units to SI, and publish to ROS
// topic
bool RtUsb9axisimuBinaryModeRosDriver::readSensorData()
{
  bool result = false;
  if (data_format_ == DataFormat::BINARY)
  {
    result = readBinaryData();
  }
  else if (data_format_ == DataFormat::ASCII)
  {
    result = readAsciiData();
  }

  return result;
}
