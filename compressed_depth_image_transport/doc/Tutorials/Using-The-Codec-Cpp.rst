Using the codec functions directly (C++)
----------------------------------------

Overview
~~~~~~~~

The compression code of this package is also available as a library, ``codecs``, which can be used without ``image_transport``.
This is useful to process messages that are not received from a topic, for example to decompress the ``compressedDepth`` messages stored in a bag.

This tutorial demonstrates how to compress and decompress a depth image with the two functions of the library.

Prerequisites
~~~~~~~~~~~~~

This tutorial assumes you have a working knowledge of ROS 2.

If you have not done so already `create a workspace <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_.

1. Write the code
~~~~~~~~~~~~~~~~~

Let's assume you have already created an empty ROS 2 package for C++ named ``depth_tutorial``.
Create a new file ``src/codec_example.cpp`` inside your package, with the following content:

.. code-block:: C++

  #include <cstdint>
  #include <cstring>
  #include <iostream>

  #include "compressed_depth_image_transport/codec.hpp"
  #include "sensor_msgs/image_encodings.hpp"
  #include "sensor_msgs/msg/compressed_image.hpp"
  #include "sensor_msgs/msg/image.hpp"

  int main()
  {
    // A 640x480 depth image in meters: a ramp from 0.5 m to 6.89 m
    sensor_msgs::msg::Image image;
    image.width = 640;
    image.height = 480;
    image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    image.step = image.width * sizeof(float);
    image.data.resize(image.step * image.height);
    for (uint32_t y = 0; y < image.height; ++y) {
      for (uint32_t x = 0; x < image.width; ++x) {
        const float depth = 0.5f + 0.01f * x;
        std::memcpy(&image.data[y * image.step + x * sizeof(float)], &depth, sizeof(depth));
      }
    }

    // Compress it
    const double depth_max = 10.0;
    const double depth_quantization = 100.0;
    const int png_level = 3;
    sensor_msgs::msg::CompressedImage::UniquePtr compressed =
      compressed_depth_image_transport::encodeCompressedDepthImage(
      image, "png", depth_max, depth_quantization, png_level);
    if (!compressed) {
      std::cerr << "Failed to compress the image" << std::endl;
      return 1;
    }
    std::cout << "format: " << compressed->format << std::endl;
    std::cout << "size: " << image.data.size() << " -> " << compressed->data.size() <<
      " bytes" << std::endl;

    // Decompress it
    sensor_msgs::msg::Image::SharedPtr decoded =
      compressed_depth_image_transport::decodeCompressedDepthImage(*compressed);
    if (!decoded) {
      std::cerr << "Failed to decompress the image" << std::endl;
      return 1;
    }

    // Compare the pixel in the center of the image
    const size_t offset = 240 * image.step + 320 * sizeof(float);
    float original_depth, decoded_depth;
    std::memcpy(&original_depth, &image.data[offset], sizeof(float));
    std::memcpy(&decoded_depth, &decoded->data[offset], sizeof(float));
    std::cout << "depth at the center: " << original_depth << " m -> " << decoded_depth <<
      " m" << std::endl;
    return 0;
  }

2. Examine the code
~~~~~~~~~~~~~~~~~~~

.. code-block:: C++

  #include "compressed_depth_image_transport/codec.hpp"

This header declares the two functions of the library.

.. code-block:: C++

    image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

This time the image stores the depth in meters, as 32-bit floating point values.

.. code-block:: C++

    sensor_msgs::msg::CompressedImage::UniquePtr compressed =
      compressed_depth_image_transport::encodeCompressedDepthImage(
      image, "png", depth_max, depth_quantization, png_level);
    if (!compressed) {
      std::cerr << "Failed to compress the image" << std::endl;
      return 1;
    }

``encodeCompressedDepthImage()`` compresses the image.
Its arguments are the same as the parameters of the publisher plugin: the format (``"png"`` or ``"rvl"``), the maximum depth, the depth quantization and the PNG compression level.

It returns a null pointer if the image cannot be compressed, for example because its encoding is not supported.

.. code-block:: C++

    sensor_msgs::msg::Image::SharedPtr decoded =
      compressed_depth_image_transport::decodeCompressedDepthImage(*compressed);
    if (!decoded) {
      std::cerr << "Failed to decompress the image" << std::endl;
      return 1;
    }

``decodeCompressedDepthImage()`` decompresses the message.
It does not need any argument: the encoding of the image and the format are read from the ``format`` field of the message, and the quantization parameters are stored together with the data.

It returns a null pointer if the message is not valid.

3. Update package.xml and CMakeLists.txt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Add the dependencies to your ``package.xml``:

.. code-block:: xml

  <depend>compressed_depth_image_transport</depend>
  <depend>sensor_msgs</depend>

Add the executable to your ``CMakeLists.txt``, and link it against the ``codecs`` library:

.. code-block:: cmake

  find_package(ament_cmake REQUIRED)
  find_package(compressed_depth_image_transport REQUIRED)
  find_package(sensor_msgs REQUIRED)

  add_executable(codec_example src/codec_example.cpp)
  target_link_libraries(codec_example PRIVATE
    compressed_depth_image_transport::codecs
    ${sensor_msgs_TARGETS})

  install(TARGETS codec_example
    DESTINATION lib/${PROJECT_NAME})

  ament_package()

4. Build and run
~~~~~~~~~~~~~~~~

Build your package from the root of your workspace, source it and run the executable:

.. code-block:: console

  $ colcon build --packages-select depth_tutorial
  $ source install/setup.bash
  $ ros2 run depth_tutorial codec_example
  format: 32FC1; compressedDepth png
  size: 1228800 -> 4245 bytes
  depth at the center: 3.7 m -> 3.70099 m

The decompressed depth is not exactly the original one.
The compression of floating point images is lossy, because the depth is quantized to 16 bits before compressing it.
The error is smaller for the measurements that are closer to the camera, and it can be reduced with a larger ``depth_quantization``.
