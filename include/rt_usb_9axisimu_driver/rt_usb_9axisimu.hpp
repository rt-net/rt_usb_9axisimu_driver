//------------------------------------------------------------------------------
// Copyright (c) 2015 RT Corporation
// All rights reserved.

// License: BSD

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of RT Corporation nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//------------------------------------------------------------------------------
#include <math.h>

#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <sstream>

/**********************************************************************************************************
 *
 * Constants
 *
 **********************************************************************************************************/

namespace RtUsbImu 
{       
  class Consts 
  {
    public:
      //IMU Data table constants
      enum{
        IMU_HEADER_FF0 = 0,
        IMU_HEADER_FF1 = 1,
        IMU_HEADER_R = 2,
        IMU_HEADER_T = 3,
        IMU_HEADER_ID0 = 4,
        IMU_HEADER_ID1 = 5,
        IMU_FIRMWARE = 6,
        IMU_TIMESTAMP = 7,
        IMU_ACC_X_L = 8,
        IMU_ACC_X_H = 9,
        IMU_ACC_Y_L = 10,
        IMU_ACC_Y_H = 11,
        IMU_ACC_Z_L = 12,
        IMU_ACC_Z_H = 13,
        IMU_TEMP_L = 14,
        IMU_TEMP_H = 15,
        IMU_GYRO_X_L = 16, 
        IMU_GYRO_X_H = 17,
        IMU_GYRO_Y_L = 18,
        IMU_GYRO_Y_H = 19,
        IMU_GYRO_Z_L = 20,
        IMU_GYRO_Z_H = 21,
        IMU_MAG_X_L = 22,
        IMU_MAG_X_H = 23,
        IMU_MAG_Y_L = 24,
        IMU_MAG_Y_H = 25,
        IMU_MAG_Z_L = 26,
        IMU_MAG_Z_H = 27,
        IMU_DATA_SIZE = 28,
      };
      //Convertor
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
      : CONVERTOR_RAW2G(2048)      // for linear_acceleration (raw data to [g])
      , CONVERTOR_RAW2DPS(16.4)    // for angular_velocity (raw data to [degree/s])
      , CONVERTOR_RAW2UT(0.3)      // for magnetic_field (raw data to [uT])
      , CONVERTOR_RAW2C_1(340)     // for temperature (raw data to celsius)
      , CONVERTOR_RAW2C_2(35)      // for temperature (raw data to celsius)
      , CONVERTOR_G2A(9.80665)     // for linear_acceleration (g to m/s^2)
      , CONVERTOR_D2R(M_PI/180.0)  // for angular_velocity (degree to radian)
      , CONVERTOR_UT2T(1000000)    // for magnetic_field (uT to Tesla)
      , DEFAULT_LINEAR_ACCELERATION_STDDEV(0.023145)    // Default of square root of the linear_acceleration_covariance diagonal elements in m/s^2. 
      , DEFAULT_ANGULAR_VELOCITY_STDDEV(0.0010621)      // Default of square root of the angular_velocity_covariance diagonal elements in rad/s.
      , DEFAULT_MAGNETIC_FIELD_STDDEV(0.00000080786)    // Default of square root of the magnetic_field_covariance diagonal elements in Tesla.
      {}

      ~Consts(){
      }

      //Method to adjust convertors to firmware version
      void ChangeConvertor(const int firmware_ver) {
        if(firmware_ver == 5) {
          CONVERTOR_RAW2UT = 0.3;
          CONVERTOR_RAW2C_1 = 340;
          CONVERTOR_RAW2C_2 = 35;
        }
        else if(firmware_ver >= 6) {
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
      std::string port_name; // ex) "/dev/ttyACM0"
      struct termios oldSettings;
      int port_fd;

    public:
      SerialPort(const char* port = "") : port_name(port), port_fd(-1){
      }
      
      ~SerialPort() {
        Close();
      }

      bool Open(const char* port) {
        port_name = port;
        return Open();
      }

      bool Open() {
        int fd = 0;

        if(port_fd > 0) {
          return true;
        }

        fd = open(port_name.c_str(), O_RDWR | O_NOCTTY);  //Open serial port
        if(fd < 0) {
          return false;  //Port open error
        }

        struct termios settings;

        tcgetattr(fd, &oldSettings);

        cfsetispeed(&settings, B57600);
        cfmakeraw(&settings);

        tcsetattr(fd, TCSANOW, &settings);

        port_fd = fd;

        return (fd > 0);

      }

      void Close() {
        if(port_fd > 0) {
          tcsetattr(port_fd, TCSANOW, &oldSettings);
          close(port_fd);  //Close serial port
          port_fd = -1;
        }
      }

      int Read(unsigned char* buf, unsigned int buf_len) {
        if(port_fd < 0) {
          return -1;
        }

        return  read(port_fd, buf, buf_len);
      }

      int Write(unsigned char* data, unsigned int data_len) {
        if(port_fd < 0) {
          return -1;
        }

        return write(port_fd, data, data_len);
      }
  };
  
  /**************************************************************************
   *
   * IMU
   *
   **************************************************************************/

  //Class to store either raw integer imu data or converted physical quantity
  template <typename Type> class ImuData 
  {
    public:
      int firmware_ver;
      int timestamp;
      Type ax, ay, az, gx, gy, gz, mx, my, mz, temperature;

      ImuData() {
        Reset();
      }
      
      ~ImuData(){
      }

      inline void Reset() {
        firmware_ver = 5;
        timestamp = -1;
        ax = ay = az = gx = gy = gz = mx = my = mz = temperature = 0;
      }
  };

  class SensorData
  {
    private:
      ImuData<signed short> imu_rawdata;
      ImuData<double> imu;
      Consts consts;

    public:
      SensorData() {
        Reset();
      }

      ~SensorData(){
      }

      void Reset() {
        imu_rawdata.Reset();
        imu.Reset();
      }

      void UpdateImuRaw(ImuData<signed short>& i) {
        imu_rawdata = i;
      }

      //Method to convert raw integer imu data to physical quantity
      void ConvertRawdata() {  
        //Adjust convertors to firmware version
        consts.ChangeConvertor(imu_rawdata.firmware_ver);

        imu.firmware_ver = imu_rawdata.firmware_ver;
        imu.timestamp = imu_rawdata.timestamp;

        // Convert raw data to [g]
        imu.ax = imu_rawdata.ax/consts.CONVERTOR_RAW2G;
        imu.ay = imu_rawdata.ay/consts.CONVERTOR_RAW2G;
        imu.az = imu_rawdata.az/consts.CONVERTOR_RAW2G;

        // Convert raw data to [degree/s]
        imu.gx = imu_rawdata.gx/consts.CONVERTOR_RAW2DPS;
        imu.gy = imu_rawdata.gy/consts.CONVERTOR_RAW2DPS;
        imu.gz = imu_rawdata.gz/consts.CONVERTOR_RAW2DPS;

        // Convert raw data to [uT]
        imu.mx = imu_rawdata.mx*consts.CONVERTOR_RAW2UT;
        imu.my = imu_rawdata.my*consts.CONVERTOR_RAW2UT;
        imu.mz = imu_rawdata.mz*consts.CONVERTOR_RAW2UT;

        // Convert raw data to celsius
        imu.temperature = imu_rawdata.temperature/consts.CONVERTOR_RAW2C_1 + consts.CONVERTOR_RAW2C_2;
      }
      
      ImuData<double> OutputImuData() {
        return imu;
      }
  };
};
