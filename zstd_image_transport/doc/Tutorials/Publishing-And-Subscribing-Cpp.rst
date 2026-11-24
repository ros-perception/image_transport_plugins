Publishing and subscribing to zstd images (C++)
-----------------------------------------------

Overview
~~~~~~~~

This tutorial demonstrates how to send images with the ``zstd`` transport.
It creates a node that publishes an image with ``image_transport``, a node that subscribes to the ``zstd`` topic, and it shows how to change the parameters of the transport.

The publisher does not contain any code that is specific to this package: ``image_transport`` loads the plugin at runtime.

Prerequisites
~~~~~~~~~~~~~

This tutorial assumes you have a working knowledge of ROS 2 and of `image_transport <https://docs.ros.org/en/rolling/p/image_transport/>`_.

If you have not done so already `create a workspace <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_.

1. Write the publisher
~~~~~~~~~~~~~~~~~~~~~~

Let's assume you have already created an empty ROS 2 package for C++ named ``image_tutorial``.
Create a new file ``src/image_publisher.cpp`` inside your package, with the following content:

.. code-block:: C++

  #include <chrono>
  #include <cstdint>
  #include <memory>

  #include "image_transport/image_transport.hpp"
  #include "rclcpp/rclcpp.hpp"
  #include "sensor_msgs/image_encodings.hpp"
  #include "sensor_msgs/msg/image.hpp"

  class ImagePublisher : public rclcpp::Node
  {
  public:
    ImagePublisher()
    : Node("image_publisher")
    {
      publisher_ = image_transport::create_publisher(
        *this, "camera/image_raw", rclcpp::QoS(10));

      timer_ = create_wall_timer(
        std::chrono::milliseconds(100), [this]() {publish_frame();});
    }

  private:
    void publish_frame()
    {
      // A synthetic color image: a gradient that moves with every frame
      sensor_msgs::msg::Image image;
      image.header.stamp = now();
      image.header.frame_id = "camera_optical_frame";
      image.width = 640;
      image.height = 480;
      image.encoding = sensor_msgs::image_encodings::BGR8;
      image.is_bigendian = false;
      image.step = image.width * 3;
      image.data.resize(image.step * image.height);

      for (uint32_t y = 0; y < image.height; ++y) {
        for (uint32_t x = 0; x < image.width; ++x) {
          uint8_t * pixel = &image.data[y * image.step + x * 3];
          pixel[0] = static_cast<uint8_t>(x + frame_count_);  // blue
          pixel[1] = static_cast<uint8_t>(y);                 // green
          pixel[2] = static_cast<uint8_t>((x + y) / 2);       // red
        }
      }
      ++frame_count_;

      publisher_.publish(image);
    }

    uint32_t frame_count_ {0};
    image_transport::Publisher publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
  };

  int main(int argc, char ** argv)
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
  }

To keep the example short, the node does not read a real camera.
It publishes a synthetic color image that changes with every frame.

.. code-block:: C++

      publisher_ = image_transport::create_publisher(
        *this, "camera/image_raw", rclcpp::QoS(10));

The image is advertised with ``image_transport`` instead of a regular ROS 2 publisher.
``image_transport`` advertises the ``camera/image_raw`` topic, and one additional topic for each of the transport plugins that are installed.
For this package it is ``camera/image_raw/zstd``.

2. Write the subscriber
~~~~~~~~~~~~~~~~~~~~~~~

Create a new file ``src/image_subscriber.cpp`` inside your package, with the following content:

.. code-block:: C++

  #include <memory>

  #include "image_transport/image_transport.hpp"
  #include "rclcpp/rclcpp.hpp"
  #include "sensor_msgs/msg/image.hpp"

  class ImageSubscriber : public rclcpp::Node
  {
  public:
    ImageSubscriber()
    : Node("image_subscriber")
    {
      subscriber_ = image_transport::create_subscription(
        *this, "camera/image_raw",
        [this](const sensor_msgs::msg::Image::ConstSharedPtr & image) {on_image(image);},
        "zstd", rclcpp::QoS(10));

      RCLCPP_INFO(get_logger(), "Subscribed to %s", subscriber_.getTopic().c_str());
    }

  private:
    void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & image)
    {
      // Blue, green and red values of the pixel in the center of the image
      const uint8_t * pixel =
        &image->data[(image->height / 2) * image->step + (image->width / 2) * 3];

      RCLCPP_INFO(
        get_logger(), "%ux%u %s image, pixel at the center: [%u, %u, %u]",
        image->width, image->height, image->encoding.c_str(), pixel[0], pixel[1], pixel[2]);
    }

    image_transport::Subscriber subscriber_;
  };

  int main(int argc, char ** argv)
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
  }

.. code-block:: C++

      subscriber_ = image_transport::create_subscription(
        *this, "camera/image_raw",
        [this](const sensor_msgs::msg::Image::ConstSharedPtr & image) {on_image(image);},
        "zstd", rclcpp::QoS(10));

The node subscribes to the base topic, ``camera/image_raw``, and it selects the transport by its name, ``zstd``.
``image_transport`` then subscribes to ``camera/image_raw/zstd`` and decodes the messages, so the callback receives a regular ``sensor_msgs/msg/Image``.

.. note::

   Instead of hard-coding the name of the transport, a node can use ``image_transport::TransportHints`` to read it from its ``image_transport`` parameter.

3. Update package.xml and CMakeLists.txt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Add the dependencies to your ``package.xml``.
The nodes are not linked against ``zstd_image_transport``, which is only needed at runtime:

.. code-block:: xml

  <depend>image_transport</depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <exec_depend>zstd_image_transport</exec_depend>

Add the executables to your ``CMakeLists.txt``:

.. code-block:: cmake

  find_package(ament_cmake REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)

  add_executable(image_publisher src/image_publisher.cpp)
  target_link_libraries(image_publisher PRIVATE
    image_transport::image_transport
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS})

  add_executable(image_subscriber src/image_subscriber.cpp)
  target_link_libraries(image_subscriber PRIVATE
    image_transport::image_transport
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS})

  install(TARGETS image_publisher image_subscriber
    DESTINATION lib/${PROJECT_NAME})

  ament_package()

4. Build and run
~~~~~~~~~~~~~~~~

Build your package from the root of your workspace and source it:

.. code-block:: console

  $ colcon build --packages-select image_tutorial
  $ source install/setup.bash

Check that the transport is available:

.. code-block:: console

  $ ros2 run image_transport list_transports
  Declared transports:
  image_transport/zstd
  image_transport/raw
  ...

Run the publisher:

.. code-block:: console

  $ ros2 run image_tutorial image_publisher

In another terminal, run the subscriber:

.. code-block:: console

  $ ros2 run image_tutorial image_subscriber
  [INFO] [image_subscriber]: Subscribed to /camera/image_raw/zstd
  [INFO] [image_subscriber]: 640x480 bgr8 image, pixel at the center: [166, 240, 24]
  [INFO] [image_subscriber]: 640x480 bgr8 image, pixel at the center: [167, 240, 24]

The compression is lossless: the blue value of the pixel grows by one with every frame, exactly as it is published.

The time stamps of the log messages are omitted here.

In another terminal, check the topics:

.. code-block:: console

  $ ros2 topic list -t
  /camera/image_raw [sensor_msgs/msg/Image]
  /camera/image_raw/zstd [sensor_msgs/msg/CompressedImage]
  ...

You can compare the bandwidth of both topics.
The numbers depend on the content of the image, and the synthetic image of this tutorial is not representative of a real camera:

.. code-block:: console

  $ ros2 topic bw /camera/image_raw
  9.39 MB/s from 50 messages
          Message size mean: 0.92 MB min: 0.92 MB max: 0.92 MB
  $ ros2 topic bw /camera/image_raw/zstd
  6.86 MB/s from 60 messages
          Message size mean: 0.68 MB min: 0.68 MB max: 0.68 MB

5. Change the compression level
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The parameter of the transport is declared in the node that publishes the images:

.. code-block:: console

  $ ros2 param list /image_publisher
    camera.image_raw.zstd.zstd_level
    ...

It can be changed while the node is running.
A higher level needs more CPU time and it produces smaller messages, although for this image the difference is small:

.. code-block:: console

  $ ros2 param set /image_publisher camera.image_raw.zstd.zstd_level 9
  Set parameter successful
  $ ros2 topic bw /camera/image_raw/zstd
  6.84 MB/s from 60 messages
          Message size mean: 0.68 MB min: 0.68 MB max: 0.68 MB

The parameter can also be set when the node starts:

.. code-block:: console

  $ ros2 run image_tutorial image_publisher --ros-args -p camera.image_raw.zstd.zstd_level:=9

6. Decode the images without writing code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Nodes that do not use ``image_transport`` cannot read the ``zstd`` topic.
The ``republish`` node of ``image_transport`` converts between transports.
To decode the images into a new ``/camera/image_decoded`` topic:

.. code-block:: console

  $ ros2 run image_transport republish --ros-args \
      -p in_transport:=zstd -p out_transport:=raw \
      -r in/zstd:=/camera/image_raw/zstd \
      -r out:=/camera/image_decoded
