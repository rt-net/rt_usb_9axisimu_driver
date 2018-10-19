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
#include "rt_usb_9axisimu_driver/rt_usb_9axisimu.hpp"

#include "ros/ros.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/Float64.h"

#include <sstream>

using namespace RtUsbImu;

class RtUsb9axisimuDriverForROS : public SerialPort
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;

    ros::Publisher imu_data_raw_pub_;
    ros::Publisher imu_mag_pub_;
    ros::Publisher imu_temperature_pub_;

    SensorData sensor_data_;

    std::string frame_id_;
    double linear_acceleration_stddev_;
    double angular_velocity_stddev_;
    double magnetic_field_stddev_;
    Consts consts;

  public:
    RtUsb9axisimuDriverForROS(std::string port="") : SerialPort(port.c_str()), nh_priv_("~")
    {
      // dependent on user device
      nh_priv_.setParam("port", port);
      // default frame id
      nh_priv_.param("frame_id", frame_id_, std::string("imu_link"));
      // defaults obtained experimentally from device
      nh_priv_.param("linear_acceleration_stddev", linear_acceleration_stddev_, consts.DEFAULT_LINEAR_ACCELERATION_STDDEV);
      nh_priv_.param("angular_velocity_stddev", angular_velocity_stddev_, consts.DEFAULT_ANGULAR_VELOCITY_STDDEV);
      nh_priv_.param("magnetic_field_stddev", magnetic_field_stddev_, consts.DEFAULT_MAGNETIC_FIELD_STDDEV);
      // publisher for streaming
      imu_data_raw_pub_   = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
      imu_mag_pub_        = nh_.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
      imu_temperature_pub_= nh_.advertise<std_msgs::Float64>("imu/temperature", 1);
    }

    ~RtUsb9axisimuDriverForROS(){
    }

    bool Start(){
      if(Open() == false)
        return false;
      return true;
    }

    //Method to combine two separate one-byte data into one two-byte data 
    signed short CombineByteData(unsigned char data_h, unsigned char data_l){
      signed short short_data = 0;

      short_data = data_h;
      short_data = short_data << 8;
      short_data |= data_l;

      return short_data;
    }

    //Method to extract sensor data from communication buffer 
    ImuData<signed short> ExtractSensorData(unsigned char* imu_data_buf){
      ImuData<signed short> imu_rawdata; 

      imu_rawdata.firmware_ver = imu_data_buf[consts.IMU_FIRMWARE];
      imu_rawdata.timestamp = imu_data_buf[consts.IMU_TIMESTAMP];
      imu_rawdata.temperature = CombineByteData(imu_data_buf[consts.IMU_TEMP_H], imu_data_buf[consts.IMU_TEMP_L]);
      imu_rawdata.ax = CombineByteData(imu_data_buf[consts.IMU_ACC_X_H], imu_data_buf[consts.IMU_ACC_X_L]);
      imu_rawdata.ay = CombineByteData(imu_data_buf[consts.IMU_ACC_Y_H], imu_data_buf[consts.IMU_ACC_Y_L]);
      imu_rawdata.az = CombineByteData(imu_data_buf[consts.IMU_ACC_Z_H], imu_data_buf[consts.IMU_ACC_Z_L]);
      imu_rawdata.gx = CombineByteData(imu_data_buf[consts.IMU_GYRO_X_H], imu_data_buf[consts.IMU_GYRO_X_L]);
      imu_rawdata.gy = CombineByteData(imu_data_buf[consts.IMU_GYRO_Y_H], imu_data_buf[consts.IMU_GYRO_Y_L]);
      imu_rawdata.gz = CombineByteData(imu_data_buf[consts.IMU_GYRO_Z_H], imu_data_buf[consts.IMU_GYRO_Z_L]);
      imu_rawdata.mx = CombineByteData(imu_data_buf[consts.IMU_MAG_X_H], imu_data_buf[consts.IMU_MAG_X_L]);
      imu_rawdata.my = CombineByteData(imu_data_buf[consts.IMU_MAG_Y_H], imu_data_buf[consts.IMU_MAG_Y_L]);
      imu_rawdata.mz = CombineByteData(imu_data_buf[consts.IMU_MAG_Z_H], imu_data_buf[consts.IMU_MAG_Z_L]);

      return imu_rawdata;
    }

    bool isCorrectData(unsigned char* imu_data_buf){
      if(imu_data_buf[consts.IMU_HEADER_R] == 'R' && 
              imu_data_buf[consts.IMU_HEADER_T] == 'T'){
        return true;
      }else{
        return false;
      }
    }

    //Method to receive IMU data, convert those units to SI, and publish to ROS topic 
    bool GetAndPublishData(){
      unsigned char imu_data_buf[256];
      ImuData<signed short> imu_rawdata; 
      ImuData<double> imu; 
      bool change_stddev = true;
      while(ros::ok()){

        int readDataSize = Read(imu_data_buf, consts.IMU_DATA_SIZE);
        if(readDataSize < consts.IMU_DATA_SIZE){
          if(readDataSize <= 0){
            return false;  //Communication error
          }
          continue;
        }

        if(isCorrectData(imu_data_buf) == false){
          continue;
        }
        imu_rawdata = ExtractSensorData(imu_data_buf);  //Extract sensor data

        consts.ChangeConvertor(imu_rawdata.firmware_ver);  //Adjust convertors to firmware version
        if(change_stddev){
          //Update standard deviations
          nh_priv_.param("linear_acceleration_stddev", linear_acceleration_stddev_, consts.DEFAULT_LINEAR_ACCELERATION_STDDEV);
          nh_priv_.param("angular_velocity_stddev", angular_velocity_stddev_, consts.DEFAULT_ANGULAR_VELOCITY_STDDEV);
          nh_priv_.param("magnetic_field_stddev", magnetic_field_stddev_, consts.DEFAULT_MAGNETIC_FIELD_STDDEV);
          change_stddev = false;
        }

        sensor_data_.UpdateImuRaw(imu_rawdata);       //Update raw data
        sensor_data_.ConvertRawdata();                 //Convert raw data to physical quantity
        imu = sensor_data_.OutputImuData();           //Get phisical quantity

        sensor_msgs::Imu imu_data_raw_msg;
        sensor_msgs::MagneticField imu_magnetic_msg;
        std_msgs::Float64 imu_temperature_msg;

        //Calculate linear_acceleration_covariance diagonal elements
        double linear_acceleration_cov = linear_acceleration_stddev_ * linear_acceleration_stddev_;
        //Calculate angular_velocity_covariance diagonal elements
        double angular_velocity_cov    = angular_velocity_stddev_ * angular_velocity_stddev_;
        //Calculate magnetic_field_covariance diagonal elements
        double magnetic_field_cov      = magnetic_field_stddev_ * magnetic_field_stddev_;

        //imu_data_raw_msg has no orientation values
        imu_data_raw_msg.orientation_covariance[0] = -1;

        imu_data_raw_msg.linear_acceleration_covariance[0] =
          imu_data_raw_msg.linear_acceleration_covariance[4] =
          imu_data_raw_msg.linear_acceleration_covariance[8] = linear_acceleration_cov;

        imu_data_raw_msg.angular_velocity_covariance[0] =
          imu_data_raw_msg.angular_velocity_covariance[4] =
          imu_data_raw_msg.angular_velocity_covariance[8] = angular_velocity_cov;

        imu_magnetic_msg.magnetic_field_covariance[0] =
          imu_magnetic_msg.magnetic_field_covariance[4] =
          imu_magnetic_msg.magnetic_field_covariance[8] = magnetic_field_cov;

        ros::Time now = ros::Time::now();

        imu_data_raw_msg.header.stamp =
          imu_magnetic_msg.header.stamp = now;

        imu_data_raw_msg.header.frame_id =
          imu_magnetic_msg.header.frame_id = frame_id_;

        // original data used the g unit, convert to m/s^2
        imu_data_raw_msg.linear_acceleration.x = imu.ax * consts.CONVERTOR_G2A;
        imu_data_raw_msg.linear_acceleration.y = imu.ay * consts.CONVERTOR_G2A;
        imu_data_raw_msg.linear_acceleration.z = imu.az * consts.CONVERTOR_G2A;

        // original data used the degree/s unit, convert to radian/s
        imu_data_raw_msg.angular_velocity.x = imu.gx * consts.CONVERTOR_D2R;
        imu_data_raw_msg.angular_velocity.y = imu.gy * consts.CONVERTOR_D2R;
        imu_data_raw_msg.angular_velocity.z = imu.gz * consts.CONVERTOR_D2R;

        // original data used the uTesla unit, convert to Tesla
        imu_magnetic_msg.magnetic_field.x =  imu.mx / consts.CONVERTOR_UT2T;
        imu_magnetic_msg.magnetic_field.y = imu.my / consts.CONVERTOR_UT2T;
        imu_magnetic_msg.magnetic_field.z = imu.mz / consts.CONVERTOR_UT2T;

        // original data used the celsius unit
        imu_temperature_msg.data = imu.temperature;

        // publish the IMU data
        imu_data_raw_pub_.publish(imu_data_raw_msg);
        imu_mag_pub_.publish(imu_magnetic_msg);
        imu_temperature_pub_.publish(imu_temperature_msg);
      }

      return true;
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rt_usb_9axisimu_driver");

  std::string port = std::string("/dev/ttyACM0");

  ros::param::get("~port", port);

  RtUsb9axisimuDriverForROS sensor(port);

  if(sensor.Start() == false)
    ROS_ERROR("Start() returns false, please check your devices.\n");
  else
  {
    ROS_INFO("RT imu driver initialization OK.\n");
    if(sensor.GetAndPublishData() == false)
      ROS_ERROR("GetAndPublishData() returns false, please check your devices.\n");
  }
  
  ROS_INFO("Shutting down RT imu driver complete.\n");

  return 0;
}
