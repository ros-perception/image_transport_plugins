^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package compressed_depth_image_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.4 (2025-02-10)
------------------

4.0.3 (2024-11-25)
------------------

4.0.2 (2024-07-31)
------------------
* Added common linters to compressed depth image transport (`#168 <https://github.com/ros-perception/image_transport_plugins/issues/168>`_)
* Contributors: Alejandro Hernández Cordero

4.0.1 (2024-07-22)
------------------
* Removed warning (`#164 <https://github.com/ros-perception/image_transport_plugins/issues/164>`_)
* Contributors: Alejandro Hernández Cordero

4.0.0 (2024-04-13)
------------------
* Added RVL Codec support to compressed_depth_image_transport (`#159 <https://github.com/ros-perception/image_transport_plugins/issues/159>`_)
* Contributors: Kenji Brameld, anilsripadarao, ijnek

3.2.0 (2023-05-08)
------------------
* Deprecated the following parameter names in favor of transport scoped ones. The remapping is listed below:
  * `image.png_level` to `image.compressedDepth.png_level`
  * `image.depth_max` to `image.compressedDepth.depth_max`
  * `image.quality` to `image.compressedDepth.depth_quantization`
  The deprecated parameters emit a warning if explicitly set, but this warning will be removed in future distros.
  (`#145 <https://github.com/ros-perception/image_transport_plugins/issues/145>`_)
* Contributors: Bartosz Meglicki, Kenji Brameld, Marcel Zeilinger

3.0.0 (2023-04-18)
------------------
* Replace deprecated cv_bridge.h with cv_bridge.hpp (`#118 <https://github.com/ros-perception/image_transport_plugins/issues/118>`_)
* Update maintainer (`#112 <https://github.com/ros-perception/image_transport_plugins/issues/112>`_)
* Contributors: Kenji Brameld

2.6.0 (2022-08-16)
------------------
* Fix advertiseImpl() in compressed_depth_publisher and subscribeImpl() in compressed_depth_subscriber. (`#106 <https://github.com/ros-perception/image_transport_plugins/issues/106>`_)
* Contributors: Ivan Santiago Paunovic

2.5.0 (2022-04-18)
------------------

2.3.2 (2022-02-18)
------------------
* Fix copyright year 20012 -> 2012 (`#79 <https://github.com/ros-perception/image_transport_plugins/issues/79>`_)
* JPEG only supports 8 bits images (`#73 <https://github.com/ros-perception/image_transport_plugins/issues/73>`_)
* Contributors: Ivan Santiago Paunovic, Michael Carroll

2.3.1 (2021-07-13)
------------------

2.3.0 (2020-05-28)
------------------
* Use non-deprecated image_transport headers (`#59 <https://github.com/ros-perception/image_transport_plugins/issues/59>`_)
* Contributors: Michael Carroll

2.2.1 (2019-10-23)
------------------

2.2.0 (2019-09-27)
------------------

2.1.0 (2019-08-23)
------------------
* Merge pull request `#33 <https://github.com/ros-perception/image_transport_plugins/issues/33>`_ from klintan/ros2
  [ROS2] Fixed portability warning for compressed depth plugin.
* fixed portability warning for name
* Contributors: Andreas Klintberg, David Gossow

2.0.0 (2018-12-13)
------------------
* Pointer api updates (`#31 <https://github.com/ros-perception/image_transport_plugins/issues/31>`_)
* Bring ros2-devel back into ros2 mainline. (`#29 <https://github.com/ros-perception/image_transport_plugins/issues/29>`_)
* Update compressed_image_transport to ros2 (`#26 <https://github.com/ros-perception/image_transport_plugins/issues/26>`_)
* Contributors: Michael Carroll, Jose Luis Rivero

1.9.5 (2016-10-03)
------------------
* disable -Werr
* Contributors: Vincent Rabaud

1.9.4 (2016-10-02)
------------------
* address gcc6 build error and tune
  With gcc6, compiling fails with `stdlib.h: No such file or directory`,
  as including '-isystem /usr/include' breaks with gcc6, cf.,
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70129.
  This commit addresses this issue for this package in the same way
  it was addressed in various other ROS packages. A list of related
  commits and pull requests is at:
  https://github.com/ros/rosdistro/issues/12783
  Signed-off-by: Lukas Bulwahn <lukas.bulwahn@oss.bmw-carit.de>
* Fix a missing return statement and add -Wall -Werror.
* Contributors: Lukas Bulwahn, Mac Mason

1.9.3 (2016-01-17)
------------------
* Refactor the codec into its own .h and .cpp.
* remove useless tf dependencies
* Contributors: Mac Mason, Vincent Rabaud

1.9.2 (2015-04-25)
------------------
* use compression parameters for both depths
  fixes `#12 <https://github.com/ros-perception/image_transport_plugins/issues/12>`_
* get code to compile with OpenCV3
* Contributors: Vincent Rabaud

1.9.1 (2014-07-18)
------------------

1.9.0 (2014-05-16)
------------------

1.8.21 (2013-06-27)
-------------------
* maintainer: david gossow
* Contributors: David Gossow

1.8.20 (2013-03-18)
-------------------
* 1.8.19 -> 1.8.20
* Contributors: Julius Kammerl

1.8.19 (2013-02-24)
-------------------
* 1.8.18 -> 1.8.19
* Contributors: Julius Kammerl

1.8.18 (2013-02-07 17:59)
-------------------------
* 1.8.17 -> 1.8.18
* fixing input format checks (enabling rgba, bgra) + minor fixes
* Contributors: Julius Kammerl

1.8.17 (2013-01-18)
-------------------
* 1.8.16 -> 1.8.17
* Contributors: Julius Kammerl

1.8.16 (2013-01-17)
-------------------
* 1.8.15 -> 1.8.16
* use the pluginlib script to remove some runtime warnings
* Contributors: Julius Kammerl, Vincent Rabaud

1.8.15 (2012-12-28 20:11)
-------------------------

1.8.14 (2012-12-28 20:02)
-------------------------

1.8.13 (2012-12-28 19:06)
-------------------------
* fix the bad exports
* make sure the plugins are visible by image_transport
* added license headers to various cpp and h files
* Contributors: Aaron Blasdel, Vincent Rabaud

1.8.12 (2012-12-19 19:30)
-------------------------
* fix downstream stuff in cmake
* Contributors: Dirk Thomas

1.8.11 (2012-12-19 17:17)
-------------------------
* fix cmake order
* Contributors: Dirk Thomas

1.8.10 (2012-12-19 17:03)
-------------------------
* fix dyn reconf
* Contributors: Dirk Thomas

1.8.9 (2012-12-19 00:26)
------------------------
* switching to verion 1.8.9
* Contributors: Julius Kammerl

1.8.8 (2012-12-17)
------------------
* adding build_deb on message_generation & mrun_deb on message_runtime
* Updated package.xml for new buildtool_depend tag for catkin requirement
* Contributors: Julius Kammerl, mirzashah

1.8.7 (2012-12-10 15:29)
------------------------
* adding missing tf build dependency
* Contributors: Julius Kammerl

1.8.6 (2012-12-10 15:08)
------------------------
* switching to version 1.8.6
* Contributors: Julius Kammerl

1.8.5 (2012-12-09)
------------------
* adding missing build debs
* added class_loader_hide_library_symbols macros to CMakeList
* switching to 1.8.5
* Contributors: Julius Kammerl

1.8.4 (2012-11-30)
------------------
* switching to version 1.8.4
* adding plugin.xml exports for pluginlib
* catkinizing theora_image_transport
* catkinizing compressed_depth_image_transport
* github migration from code.ros.org (r40053)
* Contributors: Julius Kammerl
