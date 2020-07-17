
/*
 * rt_usb_9axisimu_driver_node.cpp
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

#include <string>

#include "ros/ros.h"
#include "rt_usb_9axisimu_driver/rt_usb_9axisimu_driver.hpp"

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

  RtUsb9axisimuRosDriver driver(imu_port);
  driver.setImuFrameIdName(imu_frame_id);
  driver.setImuStdDev(imu_stddev_linear_acceleration, imu_stddev_angular_velocity, imu_stddev_magnetic_field);

  if (driver.startCommunication())
  {
    while (ros::ok() && driver.hasCompletedFormatCheck() == false)
    {
      driver.checkDataFormat();
    }

    if (ros::ok() && driver.hasCompletedFormatCheck())
    {
      ROS_INFO("Format check has completed.");
      if (driver.hasAsciiDataFormat())
      {
        ROS_INFO("Data format is ascii.");
      }
      else if (driver.hasBinaryDataFormat())
      {
        ROS_INFO("Data format is binary.");
      }
      else
      {
        ROS_ERROR("Data format is neither binary nor ascii.");
      }
    }

    if (driver.hasAsciiDataFormat() || driver.hasBinaryDataFormat())
    {
      while (ros::ok())
      {
        if (driver.readSensorData())
        {
          if (driver.hasRefreshedImuData())
          {
            driver.publishImuData();
          }
        }
        else
        {
          ROS_ERROR("readSensorData() returns false, please check your devices.\n");
        }
      }
    }
    driver.stopCommunication();
  }
  else
  {
    ROS_ERROR("Error opening sensor device, please re-check your devices.\n");
  }

  ROS_INFO("Shutting down RT imu driver complete.\n");

  return 0;
}
