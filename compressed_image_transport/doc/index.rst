Compressed Image Transport
==========================

``compressed_image_transport`` is a plugin package for `image_transport <https://docs.ros.org/en/rolling/p/image_transport/>`_.
It enables any node that uses ``image_transport`` to publish and subscribe to compressed images, without changing its code.

It provides the ``compressed`` transport, which sends every image as a ``sensor_msgs/msg/CompressedImage`` message that contains a JPEG, PNG or TIFF file.
Every image is compressed independently, so a subscriber can start at any time and a lost message only affects one image.

.. note::

   Before reading this page, make sure you understand the `image_transport <https://docs.ros.org/en/rolling/p/image_transport/>`_ package.
   For depth images use `compressed_depth_image_transport <https://index.ros.org/p/compressed_depth_image_transport/>`_ instead.

.. toctree::
   :maxdepth: 2

   self
   tutorials
   references
   Package information </standards>

1. Formats
----------

.. list-table::
   :header-rows: 1
   :widths: 12 14 22 52

   * - Format
     - Compression
     - Supported images
     - Notes
   * - ``jpeg``
     - Lossy
     - 8-bit and 16-bit
     - Color images are compressed as ``bgr8`` and the rest as ``mono8``, so 16-bit images lose their 8 least significant bits.
       Bayer images are converted to grayscale, unless ``jpeg_compress_bayer`` is ``true``: then the Bayer pattern is compressed as it is, and it can be debayered after decoding.
   * - ``png``
     - Lossless
     - 8-bit and 16-bit
     - The bit depth and the alpha channel are preserved.
   * - ``tiff``
     - Lossless
     - 8-bit, 16-bit and 32-bit
     - The image is stored without converting it.

Images with an unsupported bit depth are not published in the ``compressed`` topic, and an error is logged.

The subscriber converts the decoded image back to the encoding of the original image, which is stored in the message.

2. ROS API
----------

2.1 Publisher plugin
~~~~~~~~~~~~~~~~~~~~

Published topics
^^^^^^^^^^^^^^^^

``<base_topic>/compressed`` (``sensor_msgs/msg/CompressedImage``)
  The compressed image.

Parameters
^^^^^^^^^^

The parameters are declared in the node that publishes the images, and they can be changed at runtime.

Their names start with the name of the base topic, relative to the namespace of the node, in which the ``/`` characters are replaced by ``.``.
For example, the format used for the ``camera/image_raw`` topic is set with the ``camera.image_raw.compressed.format`` parameter.

.. list-table::
   :header-rows: 1
   :widths: 40 10 10 40

   * - Parameter
     - Type
     - Default
     - Description
   * - ``<base_topic>.compressed.format``
     - string
     - ``jpeg``
     - Compression format: ``jpeg``, ``png`` or ``tiff``.
       With any other value the images are not published, and an error is logged.
   * - ``<base_topic>.compressed.jpeg_quality``
     - int
     - ``95``
     - JPEG quality, in the range [1, 100].
       Higher values give a better image and larger messages.
   * - ``<base_topic>.compressed.jpeg_compress_bayer``
     - bool
     - ``false``
     - Compress Bayer images with JPEG without converting them to grayscale first.
   * - ``<base_topic>.compressed.png_level``
     - int
     - ``3``
     - PNG compression level, in the range [0, 9].
       Higher values trade computation time for space savings.
   * - ``<base_topic>.compressed.tiff.res_unit``
     - string
     - ``inch``
     - Resolution unit stored in the TIFF file: ``none``, ``inch`` or ``centimeter``.
   * - ``<base_topic>.compressed.tiff.xdpi``
     - int
     - ``-1``
     - Horizontal resolution stored in the TIFF file.
   * - ``<base_topic>.compressed.tiff.ydpi``
     - int
     - ``-1``
     - Vertical resolution stored in the TIFF file.

.. note::

   When the node is in a namespace, the parameters of the publisher are also declared with a leading ``.`` (for example ``.camera.image_raw.compressed.format``).
   These names are deprecated and will be removed, do not use them.

2.2 Subscriber plugin
~~~~~~~~~~~~~~~~~~~~~

Subscribed topics
^^^^^^^^^^^^^^^^^

``<base_topic>/compressed`` (``sensor_msgs/msg/CompressedImage``)
  The compressed image.

Parameters
^^^^^^^^^^

The parameters are declared in the node that subscribes to the images, with the same naming rule as the parameters of the publisher.

.. list-table::
   :header-rows: 1
   :widths: 40 10 10 40

   * - Parameter
     - Type
     - Default
     - Description
   * - ``<base_topic>.compressed.mode``
     - string
     - ``unchanged``
     - How the image is decoded by OpenCV: ``unchanged``, ``gray`` (convert to a single channel) or ``color`` (convert to three channels).

3. Message format
-----------------

This section is only relevant to decode the messages without this package.

The ``data`` field of the ``CompressedImage`` message is a complete JPEG, PNG or TIFF file, which can be decoded with any image library.
For example with OpenCV in Python: ``cv2.imdecode(numpy.frombuffer(msg.data, numpy.uint8), cv2.IMREAD_UNCHANGED)``.

The ``format`` field is ``<encoding>; <format> compressed <compressed encoding>``, for example ``rgb8; jpeg compressed bgr8``:

- ``<encoding>`` is the encoding of the original image.
- ``<format>`` is ``jpeg``, ``png`` or ``tiff``.
- ``<compressed encoding>`` is the encoding of the image stored in the file, when it has been converted before compressing it.

4. C++ API
----------

Most users do not need the C++ API of this package, because the plugins are loaded by ``image_transport``.
The classes of the plugins are described in the :doc:`references`.
