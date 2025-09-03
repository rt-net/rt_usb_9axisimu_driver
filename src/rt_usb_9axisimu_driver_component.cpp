/*
 * rt_usb_9axisimu_driver_component.cpp
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

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rt_usb_9axisimu_driver/rt_usb_9axisimu_driver_component.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


constexpr auto param_frame_id = "frame_id";
constexpr auto param_port = "port";
constexpr auto param_liner_acceleration_stddev = "linear_acceleration_stddev";
constexpr auto param_angular_velocity_stddev = "angular_velocity_stddev";
constexpr auto param_magnetic_field_stddev = "magnetic_field_stddev";
namespace rt_usb_9axisimu_driver
{

Driver::Driver(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("rt_usb_9axisimu_driver", options)
{
  driver_ = std::make_unique<RtUsb9axisimuRosDriver>("/dev/ttyACM0");

  this->declare_parameter(param_frame_id, "imu_link");
  this->declare_parameter(param_port, "/dev/ttyACM0");
  this->declare_parameter(param_liner_acceleration_stddev, 0.023145);
  this->declare_parameter(param_angular_velocity_stddev, 0.0010621);
  this->declare_parameter(param_magnetic_field_stddev, 0.00000080786);
}

void Driver::on_publish_timer()
{
  // Keep reading to get the latest data
  auto read_result = RtUsb9axisimuRosDriver::ReadStatus::NEED_TO_CONTINUE;
  while (read_result != RtUsb9axisimuRosDriver::ReadStatus::SUCCESS) {
    read_result = driver_->readSensorData();
    if (read_result == RtUsb9axisimuRosDriver::ReadStatus::NEED_TO_CONTINUE) {
      continue;
    } else if (read_result == RtUsb9axisimuRosDriver::ReadStatus::SUCCESS) {
      rclcpp::Time timestamp = this->now();
      imu_data_raw_pub_->publish(std::move(driver_->getImuRawDataUniquePtr(timestamp)));
      imu_mag_pub_->publish(std::move(driver_->getImuMagUniquePtr(timestamp)));
      imu_temperature_pub_->publish(std::move(driver_->getImuTemperatureUniquePtr()));
      break;
    } else if (read_result == RtUsb9axisimuRosDriver::ReadStatus::FAILURE) {
      RCLCPP_ERROR(this->get_logger(), "readSensorData() returns FAILURE, please check your devices.");
      return;
    }
  }
}

CallbackReturn Driver::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_configure() is called.");

  driver_->setImuFrameIdName(this->get_parameter(param_frame_id).get_value<std::string>());
  driver_->setImuPortName(this->get_parameter(param_port).get_value<std::string>());
  driver_->setImuStdDev(
    this->get_parameter(param_liner_acceleration_stddev).get_value<double>(),
    this->get_parameter(param_angular_velocity_stddev).get_value<double>(),
    this->get_parameter(param_magnetic_field_stddev).get_value<double>());

  if (!driver_->startCommunication()) {
    RCLCPP_ERROR(this->get_logger(), "Error opening sensor device, please re-check your devices.");
    return CallbackReturn::FAILURE;
  }

  driver_->checkDataFormat();

  if (driver_->hasAsciiDataFormat()) {
    RCLCPP_INFO(this->get_logger(), "Data format is ascii.");
  } else if (driver_->hasBinaryDataFormat()) {
    RCLCPP_INFO(this->get_logger(), "Data format is binary.");
  } else {
    RCLCPP_WARN(this->get_logger(), "Data format is neither binary nor ascii.");
    driver_->stopCommunication();
    return CallbackReturn::FAILURE;
  }

  imu_data_raw_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1);
  imu_mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 1);
  imu_temperature_pub_ = create_publisher<std_msgs::msg::Float64>("imu/temperature", 1);
  publish_timer_ = create_wall_timer(10ms, std::bind(&Driver::on_publish_timer, this));
  // Don't actually start publishing data until activated
  publish_timer_->cancel();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_activate() is called.");

  if (driver_->readSensorData() == RtUsb9axisimuRosDriver::ReadStatus::FAILURE) {
    RCLCPP_ERROR(this->get_logger(), "readSensorData() returns FAILURE, please check your devices.");
    return CallbackReturn::ERROR;
  }
  imu_data_raw_pub_->on_activate();
  imu_mag_pub_->on_activate();
  imu_temperature_pub_->on_activate();
  publish_timer_->reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");

  imu_data_raw_pub_->on_deactivate();
  imu_mag_pub_->on_deactivate();
  imu_temperature_pub_->on_deactivate();
  publish_timer_->cancel();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");

  driver_->stopCommunication();
  imu_data_raw_pub_.reset();
  imu_mag_pub_.reset();
  imu_temperature_pub_.reset();
  publish_timer_->cancel();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_shutdown() is called.");

  driver_->stopCommunication();
  imu_data_raw_pub_.reset();
  imu_mag_pub_.reset();
  imu_temperature_pub_.reset();
  publish_timer_->cancel();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_error() is called.");

  driver_->stopCommunication();
  imu_data_raw_pub_.reset();
  imu_mag_pub_.reset();
  imu_temperature_pub_.reset();
  publish_timer_->cancel();

  return CallbackReturn::SUCCESS;
}


}  // namespace rt_usb_9axisimu_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rt_usb_9axisimu_driver::Driver)
