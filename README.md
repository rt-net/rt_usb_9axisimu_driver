[日本語](README.md) | English

# rt_usb_9axisimu_driver

ROS 2 package for USB Output 9-axis IMU sensor module.

[![industrial_ci](https://github.com/rt-net/rt_usb_9axisimu_driver/workflows/industrial_ci/badge.svg?branch=dashing-devel)](https://github.com/rt-net/rt_usb_9axisimu_driver/actions?query=workflow%3Aindustrial_ci+branch%3Adashing-devel)

![usb-9axisimu](https://rt-net.github.io/images/usb-9axisimu/usb-9axisimu.png)

## Installation

```sh
$ cd ~/ros2_ws/src
# Clone package & checkout ROS 2 branch
$ git clone -b dashing-devel https://github.com/rt-net/rt_usb_9axisimu_driver

# Install dependencies
$ rosdep install -r -y -i --from-paths .

# Build & Install
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source ~/ros2_ws/install/setup.bash
```

## QuickStart

```sh
# Terminal 1
$ source ~/ros2_ws/install/setup.bash
$ ros2 run rt_usb_9axisimu_driver rt_usb_9axisimu_driver
```

```sh
# Terminal 2
$ source ~/ros2_ws/install/setup.bash
$ ros2 lifecycle set rt_usb_9axisimu_driver configure
$ ros2 lifecycle set rt_usb_9axisimu_driver activate
# Echo topics (Press Ctrl+C for exit)
$ ros2 topic echo /imu/data_raw
$ ros2 topic echo /imu/mag
$ ros2 topic echo /imu/temperature
```

## 1. 概要

rt_usb_9axisimu_driverは株式会社アールティが販売している
[USB出力9軸IMUセンサモジュール](https://www.rt-net.jp/products/9axisimu2/)
のROS 2パッケージです。  

株式会社アールティによって開発、メンテナンスがなされています。

- License: [The 3-Clause BSD License](https://github.com/rt-net/rt_usb_9axisimu_driver/blob/master/LICENSE)

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

2020年8月現在、販売されているUSB出力9軸IMUセンサモジュールはver2.0となります。  
このバージョンのデフォルトのファームウェアには、ASCII出力とBinary出力の２つのデータ出力形式があります。  
センサ出荷時点ではASCII出力に設定されています。出力形式の切り替え方法は、以下のリポジトリにあるマニュアルをご参照ください。  
https://github.com/rt-net/RT-USB-9AXIS-00

### [ERROR] Error opening sensor device, please re-check your devices. が発生する場合

ポートの権限を変更してください。

```sh
$ sudo chmod 666 /dev/ttyACM0
```

## 2. ノード
### 2.1 rt_usb_9axisimu_driver

rt_usb_9axisimu_driverはUSB出力9軸IMUセンサモジュールの出力を受信し、角速度と並進加速度・磁束密度をパブリッシュします。

#### 2.1.1 パブリッシュされるトピック

- /imu/data_raw([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))
    - 並進加速度と角速度の生データ

- /imu/mag([sensor_msgs/MagneticField](http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html))
    - 磁束密度の生データ

- /imu/temperature([std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float64.html))
    - センサの温度データ

#### 2.1.2 パラメータ

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


## 3. ROS 2 特有の使い方
### 3.1 Lifecycle

[Lifecycle](https://design.ros2.org/articles/node_lifecycle.html)
機能を使うことで、ノード実行中にUSBの抜き差しや、
トピックのパブリッシュを稼働/停止できます。

各状態での動作内容は次のとおりです。

#### 3.1.1 Unconfigured state

- USBポート(`~port`)にアクセスしません
- トピックをパブリッシュしません

#### 3.1.2 Configuring transition

- パラメータを反映します
- USBポート(`~port`)をオープンし9軸IMUセンサと通信します
  - 9軸IMUセンサの認識に失敗したら`Unconfigured state`に遷移します

#### 3.1.3 Inactive state

- トピックをパブリッシュしません

#### 3.1.4 Activating transition

- 9軸IMUセンサと定期通信を開始します
  - 9軸IMUセンサとの通信に失敗したら`Unconfigured state`に遷移します
- トピックのパブリッシュを開始します

#### 3.1.5 Active state

- トピックをパブリッシュします
- 9軸IMUセンサとの通信に失敗しても**状態遷移しません**

#### 3.1.6 Deactivating transition

- 9軸IMUセンサとの定期通信を停止します
- トピックのパブリッシュを停止します

#### 3.1.7 CleaningUp transition

- USBポート(`~port`)をクローズし9軸IMUセンサと通信を終了します

#### 3.1.8 Example

```sh
# Terminal 1
$ source ~/ros2_ws/install/setup.bash
$ ros2 run rt_usb_9axisimu_driver rt_usb_9axisimu_driver
```

```sh
# Terminal 2
$ source ~/ros2_ws/install/setup.bash

# User can plug-in/out the IMU module at unconfigure state.

$ ros2 lifecycle set rt_usb_9axisimu_driver configure
$ ros2 lifecycle set rt_usb_9axisimu_driver activate
# The node start publishing the topics.

# Stop publishing
$ ros2 lifecycle set rt_usb_9axisimu_driver deactivate
$ ros2 lifecycle set rt_usb_9axisimu_driver cleanup

# User can plug-in/out the IMU module at unconfigure state.
# User can set parameters of the node.
$ ros2 param set /rt_usb_9axisimu_driver frame_id "imu2-link"
$ ros2 param set /rt_usb_9axisimu_driver port "/dev/ttyACM1"

$ ros2 lifecycle set rt_usb_9axisimu_driver configure
$ ros2 lifecycle set rt_usb_9axisimu_driver activate
# The node start publishing the topics.
```

### 3.2 Component

rt_usb_9axisimu_driver::Driverは
[Component](https://index.ros.org/doc/ros2/Tutorials/Composition/)
として実装されているため、共有ライブラリとして実行できます。

#### 3.2.1 Example

```sh
# Terminal 1
$ source ~/ros2_ws/install/setup.bash
$ ros2 run rclcpp_components component_container
```

```sh
# Terminal 2
$ source ~/ros2_ws/install/setup.bash
$ ros2 component load /ComponentManager rt_usb_9axisimu_driver rt_usb_9axisimu_driver::Driver

$ ros2 lifecycle set rt_usb_9axisimu_driver configure
$ ros2 lifecycle set rt_usb_9axisimu_driver activate
# The node start publishing the topics.
```