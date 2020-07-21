^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rt_usb_9axisimu_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Update README.md (`#23 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/23>`_)
* Rename files (`#24 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/24>`_)
  Co-authored-by: Daisuke Sato <daisuke.sato@rt-net.jp>
* Refactor (`#22 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/22>`_)
  Co-authored-by: Daisuke Sato <daisuke.sato@rt-net.jp>
* Merge binary mode and ascii mode into one node (`#19 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/19>`_)
  Co-authored-by: Daisuke Sato <daisuke.sato@rt-net.jp>
* Support ASCII Mode (`#18 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/18>`_)
  * 社内のコーディングルールに合わせて変数名等変更し、ファイルを分割
  * package.xmlをformat2にアップデート
  * frame_idを指定可能にした
  * ROS_INFOやROS_ERRORを表示する
  * ライセンスファイルをアップデート
  * 使用しないrospyをCMakeListsから削除
  * ヘッダファイルを整理
  * ASCII出力時用のテスト用ノードを追加
  * Fix publish rate of imu data
  * Fix imu values
  * Delete debug message
  * Change stof to std::stof
  * Add compile option for c++11
  Co-authored-by: Daisuke Sato <daisuke.sato@rt-net.jp>
* Refactor (`#17 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/17>`_)
  * 社内のコーディングルールに合わせて変数名等変更し、ファイルを分割
  * package.xmlをformat2にアップデート
  * frame_idを指定可能にした
  * ROS_INFOやROS_ERRORを表示する
  * ライセンスファイルをアップデート
  * 使用しないrospyをCMakeListsから削除
  * ヘッダファイルを整理
  * Fix typo
  * Remove ChangeConvertor() from readSensorData()
  * Update README.md
  Co-authored-by: ShotaAk <s.aoki@rt-net.jp>
* docs: Copy contents from wiki page (`#16 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/16>`_)
  * docs: Copy contents from wiki page
  * docs: Refactor document
* Migrate to GitHub Actions (`#14 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/14>`_)
* Merge pull request `#12 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/12>`_ from rt-net/update_packagexml
  Update package.xml
* Merge pull request `#11 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/11>`_ from rt-net/`#7 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/7>`__support_melodic
  Support Melodic
* Merge branch 'master' into `#7 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/7>`__support_melodic
* Merge pull request `#9 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/9>`_ from rt-net/`#8 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/8>`__fix_travis
  Fix TravisCI settings
* Update package.xml
* Update TravisCI settings to require test for Melodic
* Update industrial_ci settings
* Update TravisCI status badge
* Fix TravisCI settings
* Contributors: Daisuke Sato, Shota Aoki, Tiryoh

0.2.0 (2019-12-21)
------------------
* Close node & port at device plugout
* Fix wrong data output
* Fix serial port attributes
* Add serial port settings after port open()
* Add required attribute to launch file
* Fix CMake Warnings
* Add a port argument to launch file
* Merge pull request `#5 <https://github.com/Tiryoh/rt_usb_9axisimu_driver/issues/5>`_ from pazeshun/secure-read-bytes
  Continue reading until all data comes
* Continue reading until all data comes
* Don't use travis-python
* Fix typo in stddev
* Merge pull request `#2 <https://github.com/Tiryoh/rt_usb_9axisimu_driver/issues/2>`_ from pazeshun/add-travis-test
  Add travis test
* Add build status to README.md
* Add .travis.yml
* Contributors: RT Corp, Shota Aoki, ShotaAk, Shun Hasegawa, pazeshun

0.1.2 (2016-03-10)
-----------

0.1.1 (2015-09-30)
-----------
* update to manage different firmware versions

0.1.0 (2015-09-26)
-----------
* first release for ROS indigo
