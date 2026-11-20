Publishing and subscribing to compressed depth images (C++)
-----------------------------------------------------------

Overview
~~~~~~~~

This tutorial demonstrates how to send depth images with the ``compressedDepth`` transport.
It creates a node that publishes a depth image with ``image_transport``, a node that subscribes to the compressed topic, and it shows how to change the compression parameters.

The publisher does not contain any code that is specific to this package: ``image_transport`` loads the plugin at runtime.

Prerequisites
~~~~~~~~~~~~~

This tutorial assumes you have a working knowledge of ROS 2 and of `image_transport <https://docs.ros.org/en/rolling/p/image_transport/>`_.

If you have not done so already `create a workspace <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_.

1. Write the publisher
~~~~~~~~~~~~~~~~~~~~~~

Let's assume you have already created an empty ROS 2 package for C++ named ``depth_tutorial``.
Create a new file ``src/depth_publisher.cpp`` inside your package, with the following content:

.. code-block:: C++

  #include <chrono>
  #include <cstdint>
  #include <cstring>
  #include <memory>

  #include "image_transport/image_transport.hpp"
  #include "rclcpp/rclcpp.hpp"
  #include "sensor_msgs/image_encodings.hpp"
  #include "sensor_msgs/msg/image.hpp"

  class DepthPublisher : public rclcpp::Node
  {
  public:
    DepthPublisher()
    : Node("depth_publisher")
    {
      publisher_ = image_transport::create_publisher(
        *this, "camera/depth/image_raw", rclcpp::SystemDefaultsQoS());

      timer_ = create_wall_timer(
        std::chrono::milliseconds(100), [this]() {publish_frame();});
    }

  private:
    void publish_frame()
    {
      // A synthetic depth image: a ramp from 0.5 m to 6.89 m, in millimeters
      sensor_msgs::msg::Image image;
      image.header.stamp = now();
      image.header.frame_id = "camera_depth_optical_frame";
      image.width = 640;
      image.height = 480;
      image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      image.is_bigendian = false;
      image.step = image.width * sizeof(uint16_t);
      image.data.resize(image.step * image.height);

      for (uint32_t y = 0; y < image.height; ++y) {
        for (uint32_t x = 0; x < image.width; ++x) {
          const uint16_t depth_mm = static_cast<uint16_t>(500 + 10 * x);
          std::memcpy(&image.data[y * image.step + x * sizeof(uint16_t)], &depth_mm, sizeof(depth_mm));
        }
      }

      publisher_.publish(image);
    }

    image_transport::Publisher publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
  };

  int main(int argc, char ** argv)
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthPublisher>());
    rclcpp::shutdown();
    return 0;
  }

To keep the example short, the node does not read a real camera.
It publishes a synthetic image in which the depth grows from left to right.

.. code-block:: C++

      publisher_ = image_transport::create_publisher(
        *this, "camera/depth/image_raw", rclcpp::SystemDefaultsQoS());

The image is advertised with ``image_transport`` instead of a regular ROS 2 publisher.
``image_transport`` advertises the ``camera/depth/image_raw`` topic, and one additional topic for each of the transport plugins that are installed.
For this package it is ``camera/depth/image_raw/compressedDepth``.

.. code-block:: C++

      image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;

The encoding of the image is the important part.
The ``compressedDepth`` transport only works with single-channel 16-bit images, like ``16UC1`` (depth in millimeters), and single-channel 32-bit floating point images, like ``32FC1`` (depth in meters).

2. Write the subscriber
~~~~~~~~~~~~~~~~~~~~~~~

Create a new file ``src/depth_subscriber.cpp`` inside your package, with the following content:

.. code-block:: C++

  #include <cstdint>
  #include <cstring>
  #include <memory>

  #include "image_transport/image_transport.hpp"
  #include "rclcpp/rclcpp.hpp"
  #include "sensor_msgs/msg/image.hpp"

  class DepthSubscriber : public rclcpp::Node
  {
  public:
    DepthSubscriber()
    : Node("depth_subscriber")
    {
      subscriber_ = image_transport::create_subscription(
        *this, "camera/depth/image_raw",
        [this](const sensor_msgs::msg::Image::ConstSharedPtr & image) {on_image(image);},
        "compressedDepth", rclcpp::SystemDefaultsQoS());

      RCLCPP_INFO(get_logger(), "Subscribed to %s", subscriber_.getTopic().c_str());
    }

  private:
    void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & image)
    {
      // Depth of the pixel in the center of the image
      const size_t offset =
        (image->height / 2) * image->step + (image->width / 2) * sizeof(uint16_t);
      uint16_t depth_mm;
      std::memcpy(&depth_mm, &image->data[offset], sizeof(depth_mm));

      RCLCPP_INFO(
        get_logger(), "%ux%u %s image, depth at the center: %.3f m",
        image->width, image->height, image->encoding.c_str(), depth_mm / 1000.0);
    }

    image_transport::Subscriber subscriber_;
  };

  int main(int argc, char ** argv)
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthSubscriber>());
    rclcpp::shutdown();
    return 0;
  }

.. code-block:: C++

      subscriber_ = image_transport::create_subscription(
        *this, "camera/depth/image_raw",
        [this](const sensor_msgs::msg::Image::ConstSharedPtr & image) {on_image(image);},
        "compressedDepth", rclcpp::SystemDefaultsQoS());

The node subscribes to the base topic, ``camera/depth/image_raw``, and it selects the transport by its name, ``compressedDepth``.
``image_transport`` then subscribes to ``camera/depth/image_raw/compressedDepth`` and decompresses the messages, so the callback receives a regular ``sensor_msgs/msg/Image``.

.. note::

   Instead of hard-coding the name of the transport, a node can use ``image_transport::TransportHints`` to read it from its ``image_transport`` parameter.

3. Update package.xml and CMakeLists.txt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Add the dependencies to your ``package.xml``.
The nodes are not linked against ``compressed_depth_image_transport``, which is only needed at runtime:

.. code-block:: xml

  <depend>image_transport</depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <exec_depend>compressed_depth_image_transport</exec_depend>

Add the executables to your ``CMakeLists.txt``:

.. code-block:: cmake

  find_package(ament_cmake REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)

  add_executable(depth_publisher src/depth_publisher.cpp)
  target_link_libraries(depth_publisher PRIVATE
    image_transport::image_transport
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS})

  add_executable(depth_subscriber src/depth_subscriber.cpp)
  target_link_libraries(depth_subscriber PRIVATE
    image_transport::image_transport
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS})

  install(TARGETS depth_publisher depth_subscriber
    DESTINATION lib/${PROJECT_NAME})

  ament_package()

4. Build and run
~~~~~~~~~~~~~~~~

Build your package from the root of your workspace and source it:

.. code-block:: console

  $ colcon build --packages-select depth_tutorial
  $ source install/setup.bash

Check that the transport is available:

.. code-block:: console

  $ ros2 run image_transport list_transports
  Declared transports:
  image_transport/compressedDepth
  image_transport/raw
  ...

Run the publisher:

.. code-block:: console

  $ ros2 run depth_tutorial depth_publisher

In another terminal, check the topics:

.. code-block:: console

  $ ros2 topic list -t
  /camera/depth/image_raw [sensor_msgs/msg/Image]
  /camera/depth/image_raw/compressedDepth [sensor_msgs/msg/CompressedImage]
  ...

And run the subscriber:

.. code-block:: console

  $ ros2 run depth_tutorial depth_subscriber
  [INFO] [depth_subscriber]: Subscribed to /camera/depth/image_raw/compressedDepth
  [INFO] [depth_subscriber]: 640x480 16UC1 image, depth at the center: 3.700 m
  [INFO] [depth_subscriber]: 640x480 16UC1 image, depth at the center: 3.700 m

The time stamps of the log messages are omitted here.

You can compare the bandwidth of both topics.
The synthetic image of this tutorial compresses much better than a real depth image:

.. code-block:: console

  $ ros2 topic bw /camera/depth/image_raw
  6.27 MB/s from 50 messages
          Message size mean: 0.61 MB min: 0.61 MB max: 0.61 MB
  $ ros2 topic bw /camera/depth/image_raw/compressedDepth
  37.96 KB/s from 40 messages
          Message size mean: 3.72 KB min: 3.72 KB max: 3.72 KB

5. Change the compression parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The parameters of the transport are declared in the node that publishes the images:

.. code-block:: console

  $ ros2 param list /depth_publisher
    camera.depth.image_raw.compressedDepth.depth_max
    camera.depth.image_raw.compressedDepth.depth_quantization
    camera.depth.image_raw.compressedDepth.format
    camera.depth.image_raw.compressedDepth.png_level
    ...

They can be changed while the node is running.
For example, to use the RVL format instead of PNG:

.. code-block:: console

  $ ros2 param set /depth_publisher camera.depth.image_raw.compressedDepth.format rvl
  Set parameter successful
  $ ros2 topic echo --once /camera/depth/image_raw/compressedDepth --field format
  16UC1; compressedDepth rvl
  ---

The subscriber detects the format of every message, so it does not need to be restarted.

The ``depth_max`` parameter removes the measurements that are too far away.
With a maximum depth of 3 meters, the pixel in the center of the image (3.7 m) becomes invalid, which is a zero in a ``16UC1`` image:

.. code-block:: console

  $ ros2 param set /depth_publisher camera.depth.image_raw.compressedDepth.depth_max 3.0
  Set parameter successful
  $ ros2 run depth_tutorial depth_subscriber
  [INFO] [depth_subscriber]: Subscribed to /camera/depth/image_raw/compressedDepth
  [INFO] [depth_subscriber]: 640x480 16UC1 image, depth at the center: 0.000 m

The parameters can also be set when the node starts:

.. code-block:: console

  $ ros2 run depth_tutorial depth_publisher --ros-args -p camera.depth.image_raw.compressedDepth.format:=rvl

6. Decompress the images without writing code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Nodes that do not use ``image_transport`` (or tools like ``ros2 topic echo``) cannot read the compressed topic.
The ``republish`` node of ``image_transport`` converts between transports.
To decompress the images into a new ``/camera/depth/image_decompressed`` topic:

.. code-block:: console

  $ ros2 run image_transport republish --ros-args \
      -p in_transport:=compressedDepth -p out_transport:=raw \
      -r in/compressedDepth:=/camera/depth/image_raw/compressedDepth \
      -r out:=/camera/depth/image_decompressed
