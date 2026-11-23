Compressed Depth Image Transport
================================

``compressed_depth_image_transport`` is a plugin package for `image_transport <https://docs.ros.org/en/rolling/p/image_transport/>`_.
It enables any node that uses ``image_transport`` to publish and subscribe to compressed depth images, without changing its code.

It provides the ``compressedDepth`` transport, which sends the depth images as ``sensor_msgs/msg/CompressedImage`` messages, compressed with PNG or with RVL (Run-length and Variable-Length encoding).

.. note::

   Before reading this page, make sure you understand the `image_transport <https://docs.ros.org/en/rolling/p/image_transport/>`_ package.
   For color or grayscale images use `compressed_image_transport <https://index.ros.org/p/compressed_image_transport/>`_ instead.

.. toctree::
   :maxdepth: 2

   self
   tutorials
   references
   Package information </standards>

1. Supported images
-------------------

The transport only works with single-channel depth images:

.. list-table::
   :header-rows: 1
   :widths: 25 20 55

   * - Encoding
     - Unit
     - Compression
   * - ``32FC1``
     - Meters
     - Lossy. The depth is quantized to 16 bits, see `4. Quantization of floating point images`_.
       The pixels which are ``NaN``, or not closer than ``depth_max``, are ``NaN`` in the decompressed image.
   * - ``16UC1`` (or ``mono16``)
     - Millimeters
     - Lossless. The pixels which are further than ``depth_max`` are ``0`` in the decompressed image.

Images with any other encoding are not published in the ``compressedDepth`` topic, and an error is logged.

2. ROS API
----------

2.1 Publisher plugin
~~~~~~~~~~~~~~~~~~~~

Published topics
^^^^^^^^^^^^^^^^

``<base_topic>/compressedDepth`` (``sensor_msgs/msg/CompressedImage``)
  The compressed depth image.

Parameters
^^^^^^^^^^

The parameters are declared in the node that publishes the images, and they can be changed at runtime.

Their names start with the name of the base topic, relative to the namespace of the node, in which the ``/`` characters are replaced by ``.``.
For example, the format used for the ``camera/depth/image_raw`` topic is set with the ``camera.depth.image_raw.compressedDepth.format`` parameter.

.. list-table::
   :header-rows: 1
   :widths: 40 10 10 40

   * - Parameter
     - Type
     - Default
     - Description
   * - ``<base_topic>.compressedDepth.format``
     - string
     - ``png``
     - Compression format: ``png`` or ``rvl``.
       The value is not validated: with any other value the images are not published.
   * - ``<base_topic>.compressedDepth.depth_max``
     - double
     - ``10.0``
     - Maximum depth value, in meters, in the range [1, 100].
       Measurements further away are discarded.
   * - ``<base_topic>.compressedDepth.depth_quantization``
     - double
     - ``100.0``
     - Depth value at which the sensor accuracy is 1 m (Kinect: >75), in the range [1, 150].
       Only used for floating point images.
   * - ``<base_topic>.compressedDepth.png_level``
     - int
     - ``3``
     - PNG compression level, in the range [0, 9].
       Higher values trade computation time for space savings.
       Only used with the ``png`` format.

.. note::

   When the node is in a namespace, the same parameters are also declared with a leading ``.`` (for example ``.camera.depth.image_raw.compressedDepth.format``).
   These names are deprecated and will be removed, do not use them.

2.2 Subscriber plugin
~~~~~~~~~~~~~~~~~~~~~

Subscribed topics
^^^^^^^^^^^^^^^^^

``<base_topic>/compressedDepth`` (``sensor_msgs/msg/CompressedImage``)
  The compressed depth image.

The subscriber plugin does not have parameters.
The format and the quantization parameters are read from every message, so they can be changed in the publisher at any time.

3. Choosing a format
--------------------

``png``
  The 16-bit image is compressed with PNG, which is lossless.
  The ``png_level`` parameter selects the trade-off between speed and size.
  The messages can be decoded by any version of this package.

``rvl``
  The 16-bit image is compressed with RVL, a lossless algorithm designed for depth images, which is much cheaper to compute than PNG [1]_.
  Use it when the CPU time of the compression matters more than the bandwidth.

.. [1] A. D. Wilson, "Fast Lossless Depth Image Compression", Proceedings of the 2017 ACM International Conference on Interactive Surfaces and Spaces.

4. Quantization of floating point images
----------------------------------------

Floating point images are converted to 16-bit integers before compressing them.
The *inverse* of the depth is quantized, so the resolution is better for the measurements that are closer to the camera, which is also how the accuracy of most depth sensors behaves.

For a depth :math:`d`, with :math:`z_0` the ``depth_quantization`` and :math:`d_{max}` the ``depth_max`` parameters, the stored value is:

.. math::

   q = \frac{a}{d} + b \qquad a = z_0 (z_0 + 1) \qquad b = 1 - \frac{a}{d_{max}}

The size of a quantization step at a depth :math:`d` is approximately :math:`d^2 / a`.
With the default parameters this is 0.1 mm at 1 m, 2.5 mm at 5 m and 1 cm at 10 m.
A larger ``depth_quantization`` gives a finer resolution.

Since the stored value has 16 bits, ``depth_quantization`` also limits the minimum depth that can be represented, which is about :math:`a / 65535`: 15 cm with the default value.

5. Message format
-----------------

This section is only relevant to decode the messages without this package.

The ``format`` field of the ``CompressedImage`` message is ``<encoding>; compressedDepth <format>``, for example ``16UC1; compressedDepth png``.
Messages without the last word were created by an old version of this package, and they use PNG.

The ``data`` field contains:

1. A header of 12 bytes: a 32-bit integer (always ``0``), followed by the two 32-bit floating point quantization parameters :math:`a` and :math:`b`. The parameters are only meaningful for floating point images.
2. The compressed 16-bit image:

   - ``png``: a PNG file.
   - ``rvl``: the number of columns and the number of rows, as 32-bit unsigned integers, followed by the RVL stream.

To recover the depth of a floating point image, a stored value of ``0`` is ``NaN``, and any other value :math:`q` is :math:`d = a / (q - b)`.

6. C++ API
----------

Most users do not need the C++ API of this package, because the plugins are loaded by ``image_transport``.

The functions used by the plugins are also available in the ``compressed_depth_image_transport::codecs`` library, to compress and decompress messages without ``image_transport``.
See the :doc:`Tutorials/Using-The-Codec-Cpp` tutorial and the :doc:`references`.
