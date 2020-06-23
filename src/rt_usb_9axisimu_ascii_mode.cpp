/*
 * rt_usb_9axisimu_ascii_mode.cpp
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

#include "rt_usb_9axisimu_driver/rt_usb_9axisimu_ascii_mode.hpp"

RtUsb9axisimuAsciiModeRosDriver::RtUsb9axisimuAsciiModeRosDriver(std::string port = "")
  : rt_usb_9axisimu::SerialPort(port.c_str())
{
  // publisher for streaming
  imu_data_raw_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
  imu_mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
  imu_temperature_pub_ = nh_.advertise<std_msgs::Float64>("imu/temperature", 1);
  imu_data_are_refreshed_ = false;
}

RtUsb9axisimuAsciiModeRosDriver::~RtUsb9axisimuAsciiModeRosDriver()
{
}

void RtUsb9axisimuAsciiModeRosDriver::setImuFrameIdName(std::string frame_id)
{
  frame_id_ = frame_id;
}

void RtUsb9axisimuAsciiModeRosDriver::setImuPortName(std::string serialport)
{
  rt_usb_9axisimu::SerialPort(serialport.c_str());
}

void RtUsb9axisimuAsciiModeRosDriver::setImuStdDev(double linear_acceleration, double angular_velocity,
                                                   double magnetic_field)
{
  linear_acceleration_stddev_ = linear_acceleration;
  angular_velocity_stddev_ = angular_velocity;
  magnetic_field_stddev_ = magnetic_field;
}

bool RtUsb9axisimuAsciiModeRosDriver::startCommunication()
{
  // returns serial port open status
  return openSerialPort();
}

// Method to combine two separate one-byte data into one two-byte data
signed short RtUsb9axisimuAsciiModeRosDriver::combineByteData(unsigned char data_h, unsigned char data_l)
{
  signed short short_data = 0;

  short_data = data_h;
  short_data = short_data << 8;
  short_data |= data_l;

  return short_data;
}

// Method to extract binary sensor data from communication buffer
rt_usb_9axisimu::ImuData<signed short>
RtUsb9axisimuAsciiModeRosDriver::extractBinarySensorData(unsigned char* imu_data_buf)
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

bool RtUsb9axisimuAsciiModeRosDriver::isAsciiSensorData(unsigned char* imu_data_buf)
{
  bool is_ascii_sensor_data = false;
  if (imu_data_buf[consts.IMU_HEADER_R] == 'R' && imu_data_buf[consts.IMU_HEADER_T] == 'T')
  {
    is_ascii_sensor_data = true;
  }
  return is_ascii_sensor_data;
}

bool RtUsb9axisimuAsciiModeRosDriver::publishSensorData()
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

  // original data used the radian/s unit
  imu_data_raw_msg.angular_velocity.x = imu.gx;
  imu_data_raw_msg.angular_velocity.y = imu.gy;
  imu_data_raw_msg.angular_velocity.z = imu.gz;

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
bool RtUsb9axisimuAsciiModeRosDriver::readSensorData()
{
  static std::vector<std::string> imu_data_vector_buf;

  unsigned char imu_data_buf[256];
  rt_usb_9axisimu::ImuData<double> imu_data;
  std::string imu_data_oneline_buf;

  imu_data_are_refreshed_ = false;
  imu_data_oneline_buf.clear();

  int data_size_of_buf = readFromDevice(imu_data_buf, sizeof(imu_data_buf));

  if (data_size_of_buf <= 0)
  {
    return false;  // Raize communication error
  }

  // if (isAsciiSensorData(imu_data_buf) == false)
  // {
  //   return false;
  // }

  for (int char_count = 0; char_count < data_size_of_buf; char_count++)
  {
    if (imu_data_buf[char_count] == ',' || imu_data_buf[char_count] == '\n')
    {
      imu_data_vector_buf.push_back(imu_data_oneline_buf);
      imu_data_oneline_buf.clear();
    }
    else
    {
      imu_data_oneline_buf += imu_data_buf[char_count];
    }

    if (imu_data_buf[char_count] == '\n' && imu_data_vector_buf.size() == 11 &&
        imu_data_vector_buf[0].find(".") == std::string::npos)
    {
      imu_data.gx = stof(imu_data_vector_buf[1]);
      imu_data.gy = stof(imu_data_vector_buf[2]);
      imu_data.gz = stof(imu_data_vector_buf[3]);
      imu_data.ax = stof(imu_data_vector_buf[4]);
      imu_data.ay = stof(imu_data_vector_buf[5]);
      imu_data.az = stof(imu_data_vector_buf[6]);
      imu_data.mx = stof(imu_data_vector_buf[7]);
      imu_data.my = stof(imu_data_vector_buf[8]);
      imu_data.mz = stof(imu_data_vector_buf[9]);
      imu_data.temperature = stof(imu_data_vector_buf[10]);

      imu_data_vector_buf.clear();
      sensor_data_.setImuData(imu_data);
      imu_data_are_refreshed_ = true;
    }
    else if (imu_data_vector_buf.size() > 11)
    {
      // TODO ERROR HANDLING
      imu_data_vector_buf.clear();
    }
  }

  return true;
}

bool RtUsb9axisimuAsciiModeRosDriver::imuDataAreRefreshed()
{
  return imu_data_are_refreshed_;
}

int main(int argc, char** argv)
{
  // init ROS node
  ros::init(argc, argv, "rt_usb_9axisimu_driver");

  std::string imu_port = std::string("/dev/ttyACM0");
  ros::param::get("~port", imu_port);
  std::string imu_frame_id = std::string("imu_link");
  ros::param::get("~frame_id", imu_frame_id);
  rt_usb_9axisimu::Consts imu_consts;
  double imu_stddev_linear_acceleration = imu_consts.DEFAULT_LINEAR_ACCELERATION_STDDEV;
  ros::param::get("~linear_acceleration_stddev", imu_stddev_linear_acceleration);
  double imu_stddev_angular_velocity = imu_consts.DEFAULT_ANGULAR_VELOCITY_STDDEV;
  ros::param::get("~angular_velocity_stddev", imu_stddev_angular_velocity);
  double imu_stddev_magnetic_field = imu_consts.DEFAULT_MAGNETIC_FIELD_STDDEV;
  ros::param::get("~magnetic_field_stddev", imu_stddev_magnetic_field);

  RtUsb9axisimuAsciiModeRosDriver sensor(imu_port);
  sensor.setImuFrameIdName(imu_frame_id);
  sensor.setImuStdDev(imu_stddev_linear_acceleration, imu_stddev_angular_velocity, imu_stddev_magnetic_field);

  if (sensor.startCommunication())
  {
    ROS_INFO("RT imu driver initialization OK.\n");
    while (ros::ok())
    {
      if (sensor.readSensorData())
      {
        if (sensor.imuDataAreRefreshed())
        {
          sensor.publishSensorData();
        }
      }
      else
      {
        ROS_ERROR("GetSensorData() returns false, please check your devices.\n");
      }
    }
  }
  else
  {
    ROS_ERROR("Error opening sensor device, please re-check your devices.\n");
  }

  ROS_INFO("Shutting down RT imu driver complete.\n");

  return 0;
}
