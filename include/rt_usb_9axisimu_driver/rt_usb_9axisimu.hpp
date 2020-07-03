/*
 * rt_usb_9axisimu.hpp
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

#ifndef RT_USB_9AXISIMU_H_
#define RT_USB_9AXISIMU_H_

#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <sstream>

/**********************************************************************************************************
 *
 * Constants
 *
 **********************************************************************************************************/

namespace rt_usb_9axisimu
{
class Consts
{
public:
  enum IMU_BINARY_DATA_TABLE
  {
    IMU_BIN_HEADER_FF0 = 0,
    IMU_BIN_HEADER_FF1 = 1,
    IMU_BIN_HEADER_R = 2,
    IMU_BIN_HEADER_T = 3,
    IMU_BIN_HEADER_ID0 = 4,
    IMU_BIN_HEADER_ID1 = 5,
    IMU_BIN_FIRMWARE = 6,
    IMU_BIN_TIMESTAMP = 7,
    IMU_BIN_ACC_X_L = 8,
    IMU_BIN_ACC_X_H = 9,
    IMU_BIN_ACC_Y_L = 10,
    IMU_BIN_ACC_Y_H = 11,
    IMU_BIN_ACC_Z_L = 12,
    IMU_BIN_ACC_Z_H = 13,
    IMU_BIN_TEMP_L = 14,
    IMU_BIN_TEMP_H = 15,
    IMU_BIN_GYRO_X_L = 16,
    IMU_BIN_GYRO_X_H = 17,
    IMU_BIN_GYRO_Y_L = 18,
    IMU_BIN_GYRO_Y_H = 19,
    IMU_BIN_GYRO_Z_L = 20,
    IMU_BIN_GYRO_Z_H = 21,
    IMU_BIN_MAG_X_L = 22,
    IMU_BIN_MAG_X_H = 23,
    IMU_BIN_MAG_Y_L = 24,
    IMU_BIN_MAG_Y_H = 25,
    IMU_BIN_MAG_Z_L = 26,
    IMU_BIN_MAG_Z_H = 27,
    IMU_BIN_DATA_SIZE = 28,
  };

  enum IMU_ASCII_DATA_TABLE
  {
    IMU_ASCII_TIMESTAMP = 0,
    IMU_ASCII_GYRO_X,
    IMU_ASCII_GYRO_Y,
    IMU_ASCII_GYRO_Z,
    IMU_ASCII_ACC_X,
    IMU_ASCII_ACC_Y,
    IMU_ASCII_ACC_Z,
    IMU_ASCII_MAG_X,
    IMU_ASCII_MAG_Y,
    IMU_ASCII_MAG_Z,
    IMU_ASCII_TEMP,
    IMU_ASCII_DATA_SIZE
  };

  // Convertor
  const double CONVERTOR_RAW2G;
  const double CONVERTOR_RAW2DPS;
  double CONVERTOR_RAW2UT;
  double CONVERTOR_RAW2C_1;
  double CONVERTOR_RAW2C_2;
  const double CONVERTOR_G2A;
  const double CONVERTOR_D2R;
  const double CONVERTOR_UT2T;
  const double DEFAULT_LINEAR_ACCELERATION_STDDEV;
  const double DEFAULT_ANGULAR_VELOCITY_STDDEV;
  const double DEFAULT_MAGNETIC_FIELD_STDDEV;

  Consts()
    : CONVERTOR_RAW2G(2048)                         // for linear_acceleration (raw data to [g])
    , CONVERTOR_RAW2DPS(16.4)                       // for angular_velocity (raw data to [degree/s])
    , CONVERTOR_RAW2UT(0.3)                         // for magnetic_field (raw data to [uT])
    , CONVERTOR_RAW2C_1(340)                        // for temperature (raw data to celsius)
    , CONVERTOR_RAW2C_2(35)                         // for temperature (raw data to celsius)
    , CONVERTOR_G2A(9.80665)                        // for linear_acceleration (g to m/s^2)
    , CONVERTOR_D2R(M_PI / 180.0)                   // for angular_velocity (degree to radian)
    , CONVERTOR_UT2T(1000000)                       // for magnetic_field (uT to Tesla)
    , DEFAULT_LINEAR_ACCELERATION_STDDEV(0.023145)  // Default of square root of the
                                                    // linear_acceleration_covariance diagonal elements in
                                                    // m/s^2.
    , DEFAULT_ANGULAR_VELOCITY_STDDEV(0.0010621)    // Default of square root of the
                                                    // angular_velocity_covariance diagonal elements in
                                                    // rad/s.
    , DEFAULT_MAGNETIC_FIELD_STDDEV(0.00000080786)  // Default of square root of the
                                                    // magnetic_field_covariance diagonal elements in
                                                    // Tesla.
  {
  }

  ~Consts()
  {
  }

  // Method to adjust convertors to firmware version
  void ChangeConvertor(const int firmware_ver)
  {
    if (firmware_ver == 5)
    {
      CONVERTOR_RAW2UT = 0.3;
      CONVERTOR_RAW2C_1 = 340;
      CONVERTOR_RAW2C_2 = 35;
    }
    else if (firmware_ver >= 6)
    {
      CONVERTOR_RAW2UT = 0.15;
      CONVERTOR_RAW2C_1 = 333.87;
      CONVERTOR_RAW2C_2 = 21;
    }
  }
};

/**********************************************************************************************************
 *
 * Serial port abstraction
 *
 **********************************************************************************************************/

class SerialPort
{
private:
  std::string port_name_;  // ex) "/dev/ttyACM0"
  struct termios old_settings_;
  int port_fd_;

public:
  SerialPort(const char* port = "") : port_name_(port), port_fd_(-1)
  {
  }

  ~SerialPort()
  {
    closeSerialPort();
  }

  bool openPort(const char* port)
  {
    port_name_ = port;
    return openSerialPort();
  }

  bool openSerialPort()
  {
    int fd = 0;

    if (port_fd_ > 0)
    {
      return true;
    }

    fd = open(port_name_.c_str(), O_RDWR | O_NOCTTY);  // Open serial port
    if (fd < 0)
    {
      return false;  // Port open error
    }

    struct termios settings;

    tcgetattr(fd, &old_settings_);

    cfsetispeed(&settings, B57600);
    cfmakeraw(&settings);

    tcsetattr(fd, TCSANOW, &settings);

    port_fd_ = fd;

    return (fd > 0);
  }

  void closeSerialPort()
  {
    if (port_fd_ > 0)
    {
      tcsetattr(port_fd_, TCSANOW, &old_settings_);
      close(port_fd_);  // Close serial port
      port_fd_ = -1;
    }
  }

  int readFromDevice(unsigned char* buf, unsigned int buf_len)
  {
    if (port_fd_ < 0)
    {
      return -1;
    }

    return read(port_fd_, buf, buf_len);
  }

  int writeToDevice(unsigned char* data, unsigned int data_len)
  {
    if (port_fd_ < 0)
    {
      return -1;
    }

    return write(port_fd_, data, data_len);
  }
};

/**************************************************************************
 *
 * IMU
 *
 **************************************************************************/

// Class to store either raw integer imu data or converted physical quantity
template <typename Type>
class ImuData
{
public:
  int firmware_ver;
  int timestamp;
  Type ax, ay, az, gx, gy, gz, mx, my, mz, temperature;

  ImuData()
  {
    reset();
  }

  ~ImuData()
  {
  }

  inline void reset()
  {
    firmware_ver = 5;
    timestamp = -1;
    ax = ay = az = gx = gy = gz = mx = my = mz = temperature = 0;
  }
};

class SensorData
{
private:
  ImuData<int16_t> imu_raw_data_;
  ImuData<double> imu_;
  Consts consts_;

public:
  SensorData()
  {
    reset();
  }

  ~SensorData()
  {
  }

  void reset()
  {
    imu_raw_data_.reset();
    imu_.reset();
  }

  void setImuRawData(ImuData<int16_t>& i)
  {
    imu_raw_data_ = i;
  }

  void setImuData(ImuData<double>& i)
  {
    imu_ = i;
  }

  // Method to convert raw integer imu_ data to physical quantity
  void convertRawDataUnit()
  {
    // Adjust convertors to firmware version
    consts_.ChangeConvertor(imu_raw_data_.firmware_ver);

    imu_.firmware_ver = imu_raw_data_.firmware_ver;
    imu_.timestamp = imu_raw_data_.timestamp;

    // Convert raw data to [g]
    imu_.ax = imu_raw_data_.ax / consts_.CONVERTOR_RAW2G;
    imu_.ay = imu_raw_data_.ay / consts_.CONVERTOR_RAW2G;
    imu_.az = imu_raw_data_.az / consts_.CONVERTOR_RAW2G;

    // Convert raw data to [degree/s]
    imu_.gx = imu_raw_data_.gx / consts_.CONVERTOR_RAW2DPS;
    imu_.gy = imu_raw_data_.gy / consts_.CONVERTOR_RAW2DPS;
    imu_.gz = imu_raw_data_.gz / consts_.CONVERTOR_RAW2DPS;

    // Convert raw data to [uT]
    imu_.mx = imu_raw_data_.mx * consts_.CONVERTOR_RAW2UT;
    imu_.my = imu_raw_data_.my * consts_.CONVERTOR_RAW2UT;
    imu_.mz = imu_raw_data_.mz * consts_.CONVERTOR_RAW2UT;

    // Convert raw data to celsius
    imu_.temperature = imu_raw_data_.temperature / consts_.CONVERTOR_RAW2C_1 + consts_.CONVERTOR_RAW2C_2;
  }

  ImuData<double> getImuData()
  {
    return imu_;
  }
};
};  // namespace rt_usb_9axisimu

#endif