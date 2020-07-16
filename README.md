[日本語](README.md) | English

# rt_usb_9axisimu_driver

[![industrial_ci](https://github.com/rt-net/rt_usb_9axisimu_driver/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/rt_usb_9axisimu_driver/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

rt_usb_9axisimu_driverは株式会社アールティが販売しているUSB出力9軸IMUセンサモジュール用のROSパッケージです。  
株式会社アールティによって開発、メンテナンスがなされています。
- License: [The 3-Clause BSD License](https://github.com/rt-net/rt_usb_9axisimu_driver/blob/master/LICENSE)
- Source: https://github.com/rt-net/rt_usb_9axisimu_driver.git (branch: master)

現在、以下のROSのディストリビューションに対応しております。
- Kinetic
- Melodic

## 1. 概要

このパッケージは、[USB出力9軸IMUセンサモジュール](https://www.rt-net.jp/products/9axisimu2/)をROSから使用するためのドライバを提供するものです。

### 1.1 座標軸について

USB出力9軸IMUセンサモジュールは、センサとしてInvenSense社のMPU9250を使用しております。  
このセンサの磁気センサの座標系はNED座標系(x-north, y-east, z-down)ですが、
モジュール内のマイコン(LPC1343)においてENU座標系(x-east, y-north, z-up)に変換され、
ジャイロセンサおよび加速度センサの座標系と揃えられております。  
これはROSで使われる座標系のルールにも適合しています。詳しくは、[REP-0103](http://www.ros.org/reps/rep-0103.html#axis-orientation)をご覧ください。

### 1.2 ファームウェア開発について

USB出力9軸IMUセンサモジュールはオープンハード・オープンソースのため、モジュール内のマイコンのファームウェアの変更が可能です。  
このROSパッケージはデフォルトのファームウェアにのみ対応しております。ファームウェアを変更された場合、正常な動作ができなくなる恐れがございますので、ご了承ください。

### 1.3 ver2.0でのご利用について

2018年10月現在、販売されているUSB出力9軸IMUセンサモジュールはver2.0となります。  
このバージョンのデフォルトのファームウェアには、ASCII出力とBinary出力の２つのデータ出力形式があります。  
センサ出荷時点ではASCII出力に設定されています。出力形式の切り替え方法は、以下のリポジトリにあるマニュアルをご参照ください。  
https://github.com/rt-net/RT-USB-9AXIS-00

## 2. インストール

現在、Debianパッケージの配布は行っておりません。ソースからインストールを行ってください。  
catkinワークスペースを~/catkin_wsとすると、以下のような手順になります。

```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/rt-net/rt_usb_9axisimu_driver.git
$ cd ~/catkin_ws
$ catkin build
```

## 3. 使用方法
2のようにインストールを行った場合、ドライバを起動しようとする端末(Terminal)で、
```sh
$ source ~/catkin_ws/devel/setup.bash
```
と入力してください。その後、

```sh
$ roscore

# 別の端末で
$ rosrun rt_usb_9axisimu_driver rt_usb_9axisimu_driver
```

または、

```sh
$ roslaunch rt_usb_9axisimu_driver rt_usb_9axisimu_driver.launch
```

と入力することにより、ドライバを起動できます。

### [ERROR] Error opening sensor device, please re-check your devices. が発生する場合

ポートの権限を変更してください。

```sh
$ sudo chmod 666 /dev/ttyACM0
```

## 4. ノード
### 4.1 rt_usb_9axisimu_driver

rt_usb_9axisimu_driverはUSB出力9軸IMUセンサモジュールの出力を受信し、角速度と並進加速度・磁束密度をパブリッシュします。

#### 4.1.1 パブリッシュされるトピック

- /imu/data_raw([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))
    - 並進加速度と角速度の生データ

- /imu/mag([sensor_msgs/MagneticField](http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html))
    - 磁束密度の生データ

- /imu/temperature([std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float64.html))
    - センサの温度データ

#### 4.1.2 パラメータ

- ~frame_id (string, default: imu_link)
    - IMUデータのヘッダーにセットされるフレーム名

- ~port (string, default: /dev/ttyACM0)
    - モジュールが接続されているポート名

- ~linear_acceleration_stddev (double, default: 0.023145)
    - 並進加速度の共分散行列の対角成分の平方根(m/s^2)

- ~angular_velocity_stddev (double, default: 0.0010621)
    - 角速度の共分散行列の対角成分の平方根(rad/s)

- ~magnetic_field_stddev (double, default: 0.00000080786)
    - 磁束密度の共分散行列の対角成分の平方根(T)

