^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rt_usb_9axisimu_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2021-05-13)
------------------
* Contributors: Daisuke Sato, Shota Aoki

Bug fix
^^^^^^^^^^^^^^^
* Fix launch file install path (`#36 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/36>`_)

Documentation updates
^^^^^^^^^^^^^^^^^^^^^
* Support ROS 1 Noetic (`#37 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/37>`_)
* Update README to add Foxy support (ROS 1 branch) (`#32 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/32>`_)
* Add build status to README (`#28 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/28>`_)

1.0.0 (2020-07-21)
------------------
* Contributors: Daisuke Sato, Shota Aoki

Feature updates
^^^^^^^^^^^^^^^
* Rename files (`#24 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/24>`_)
* Refactor (`#22 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/22>`_)
* Merge binary mode and ascii mode into one node (`#19 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/19>`_)
* Support ASCII Mode (`#18 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/18>`_)
* Refactor (`#17 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/17>`_)

Documentation updates
^^^^^^^^^^^^^^^^^^^^^
* Update README.md (`#23 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/23>`_)
* docs: Copy contents from wiki page (`#16 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/16>`_)


CI updates
^^^^^^^^^^^^^^
* Migrate to GitHub Actions (`#14 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/14>`_)
* Merge pull request `#11 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/11>`_ from rt-net/#7_support_melodic

  * Merge branch 'master' into #7_support_melodic
  * Update TravisCI settings to require test for Melodic
* Merge pull request `#9 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/9>`_ from rt-net/#8_fix_travis

  * Update industrial_ci settings
  * Update TravisCI status badge
  * Fix TravisCI settings

Minor updates for ROS Package information
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
* Merge pull request `#12 <https://github.com/rt-net/rt_usb_9axisimu_driver/issues/12>`_ from rt-net/update_packagexml

  * Update package.xml


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
------------------

0.1.1 (2015-09-30)
------------------
* update to manage different firmware versions

0.1.0 (2015-09-26)
------------------
* first release for ROS indigo
