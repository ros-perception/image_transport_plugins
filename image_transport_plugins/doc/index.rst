Image Transport Plugins
=======================

``image_transport_plugins`` is a metapackage that installs a set of plugins for `image_transport <https://docs.ros.org/en/rolling/p/image_transport/>`_.

The plugins publish and subscribe to ``sensor_msgs/msg/Image`` topics in representations other than the raw pixel data.
For example, a video codec gives a much lower bandwidth to view a stream of images off-robot, and JPEG or PNG are a good choice for the low frame rate transport of high-definition images.

Nodes that use ``image_transport`` do not need to be modified, or even compiled again, to use the plugins.
When a plugin is installed, every ``image_transport`` publisher advertises an additional topic for its transport, and every ``image_transport`` subscriber can select it by name.

.. toctree::
   :maxdepth: 2

   self
   Package information </standards>

1. Plugins
----------

.. list-table::
   :header-rows: 1
   :widths: 30 15 12 43

   * - Package
     - Transport
     - Lossless
     - Description
   * - `compressed_image_transport <https://docs.ros.org/en/rolling/p/compressed_image_transport/>`_
     - ``compressed``
     - PNG and TIFF
     - Compresses every image as JPEG, PNG or TIFF. The general purpose choice for color and grayscale images.
   * - `compressed_depth_image_transport <https://docs.ros.org/en/rolling/p/compressed_depth_image_transport/>`_
     - ``compressedDepth``
     - 16-bit images
     - Compresses depth images (16-bit integer or 32-bit floating point) with PNG or RVL.
   * - `theora_image_transport <https://docs.ros.org/en/rolling/p/theora_image_transport/>`_
     - ``theora``
     - No
     - Encodes the images as a Theora video stream. The lowest bandwidth, but subscribers must receive the stream from its beginning.
   * - `zstd_image_transport <https://docs.ros.org/en/rolling/p/zstd_image_transport/>`_
     - ``zstd``
     - Yes
     - Compresses the data of the image with a general purpose algorithm. Works with any type of image.

See the documentation of each package for its parameters and tutorials.

2. Usage
--------

Install the metapackage, or only the plugins that you need:

.. code-block:: console

   $ sudo apt install ros-$ROS_DISTRO-image-transport-plugins

List the transports that are available:

.. code-block:: console

   $ ros2 run image_transport list_transports
   Declared transports:
   image_transport/compressed
   image_transport/compressedDepth
   image_transport/raw
   image_transport/theora
   image_transport/zstd
   ...

A node that publishes the ``camera/image_raw`` topic with ``image_transport`` now advertises one topic for every transport:

.. code-block:: console

   $ ros2 topic list -t
   /camera/image_raw [sensor_msgs/msg/Image]
   /camera/image_raw/compressed [sensor_msgs/msg/CompressedImage]
   /camera/image_raw/compressedDepth [sensor_msgs/msg/CompressedImage]
   /camera/image_raw/theora [theora_image_transport/msg/Packet]
   /camera/image_raw/zstd [sensor_msgs/msg/CompressedImage]

The images are only encoded for the transports that have subscribers.

Every plugin declares its parameters in the node that uses it, named after the topic and the transport:

.. code-block:: console

   $ ros2 param set /my_camera camera.image_raw.compressed.jpeg_quality 80

To limit the transports that a publisher advertises, set its ``<base_topic>.enable_pub_plugins`` parameter to the list of plugins to load, for example ``["image_transport/compressed"]``.

To subscribe to one of the transports, select it by name in a node that subscribes with ``image_transport``, or convert it to raw images with the ``republish`` node:

.. code-block:: console

   $ ros2 run image_transport republish --ros-args \
       -p in_transport:=compressed -p out_transport:=raw \
       -r in/compressed:=/camera/image_raw/compressed \
       -r out:=/camera/image_decoded
