Zstd Image Transport
====================

``zstd_image_transport`` is a plugin package for `image_transport <https://docs.ros.org/en/rolling/p/image_transport/>`_.
It enables any node that uses ``image_transport`` to publish and subscribe to images compressed with a general purpose lossless algorithm, without changing its code.

It provides the ``zstd`` transport, which sends every image as a ``sensor_msgs/msg/CompressedImage`` message.
The compression is lossless and it does not depend on the encoding of the image, so it works with any type of image: color, grayscale, depth, floating point, etc.

.. note::

   Before reading this page, make sure you understand the `image_transport <https://docs.ros.org/en/rolling/p/image_transport/>`_ package.

.. warning::

   Despite the name of the package, the images are currently compressed with zlib (DEFLATE), not with Zstandard.

.. toctree::
   :maxdepth: 2

   self
   tutorials
   references
   Package information </standards>

1. When to use it
-----------------

The transport compresses the bytes of the image without taking into account that they are an image.
The decoded image is identical to the original one, but the compression ratio is usually much lower than the one of the image codecs:

- For color or grayscale images, `compressed_image_transport <https://index.ros.org/p/compressed_image_transport/>`_ (PNG) is also lossless and it usually produces smaller messages.
- For depth images, see `compressed_depth_image_transport <https://index.ros.org/p/compressed_depth_image_transport/>`_.

Use the ``zstd`` transport for the images that the other transports do not support, or when the image must not be converted in any way.

2. ROS API
----------

2.1 Publisher plugin
~~~~~~~~~~~~~~~~~~~~

Published topics
^^^^^^^^^^^^^^^^

``<base_topic>/zstd`` (``sensor_msgs/msg/CompressedImage``)
  The compressed image.

Parameters
^^^^^^^^^^

The parameters are declared in the node that publishes the images, and they can be changed at runtime.

Their names start with the name of the base topic, relative to the namespace of the node, in which the ``/`` characters are replaced by ``.``.
For example, the level used for the ``camera/image_raw`` topic is set with the ``camera.image_raw.zstd.zstd_level`` parameter.

.. list-table::
   :header-rows: 1
   :widths: 40 10 10 40

   * - Parameter
     - Type
     - Default
     - Description
   * - ``<base_topic>.zstd.zstd_level``
     - int
     - ``3``
     - Compression level, in the range [0, 9].
       Higher values trade computation time for space savings, and ``0`` does not compress the data.

.. note::

   When the node is in a namespace, the parameters of the publisher are also declared with a leading ``.`` (for example ``.camera.image_raw.zstd.zstd_level``).
   These names are deprecated and will be removed, do not use them.

2.2 Subscriber plugin
~~~~~~~~~~~~~~~~~~~~~

Subscribed topics
^^^^^^^^^^^^^^^^^

``<base_topic>/zstd`` (``sensor_msgs/msg/CompressedImage``)
  The compressed image.

The subscriber plugin does not have parameters.

3. Message format
-----------------

This section is only relevant to decode the messages without this package.

The ``format`` field of the ``CompressedImage`` message is ``zstd``.

The ``data`` field contains the fields of the original ``sensor_msgs/msg/Image`` message, followed by its compressed data.
The integers are stored in little endian:

.. list-table::
   :header-rows: 1
   :widths: 20 20 60

   * - Offset (bytes)
     - Size (bytes)
     - Content
   * - 0
     - 4
     - ``height``
   * - 4
     - 4
     - ``width``
   * - 8
     - 1
     - ``is_bigendian``
   * - 9
     - 4
     - ``step``
   * - 13
     - 4
     - Length of ``encoding``, :math:`n`
   * - 17
     - :math:`n`
     - ``encoding``
   * - 17 + :math:`n`
     - Rest of the data
     - The ``data`` of the image, compressed as a gzip stream

4. C++ API
----------

Most users do not need the C++ API of this package, because the plugins are loaded by ``image_transport``.
The classes of the plugins are described in the :doc:`references`.
