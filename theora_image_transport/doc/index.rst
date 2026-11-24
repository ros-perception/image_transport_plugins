Theora Image Transport
======================

``theora_image_transport`` is a plugin package for `image_transport <https://docs.ros.org/en/rolling/p/image_transport/>`_.
It enables any node that uses ``image_transport`` to publish and subscribe to a video stream, without changing its code.

It provides the ``theora`` transport, which encodes the images with the `Theora <https://www.theora.org/>`_ video codec and sends the stream as ``theora_image_transport/msg/Packet`` messages.
Since a video codec only encodes the differences between consecutive images, it needs much less bandwidth than the transports that compress every image independently.

.. note::

   Before reading this page, make sure you understand the `image_transport <https://docs.ros.org/en/rolling/p/image_transport/>`_ package.

.. toctree::
   :maxdepth: 2

   self
   tutorials
   references
   Message definitions </interfaces/message_definitions>
   Package information </standards>

1. Limitations
--------------

A video stream has some limitations that do not apply to the other transports.
Make sure they are acceptable for your application:

- **The compression is lossy**, and the color resolution is halved (4:2:0 chroma subsampling).
  Do not use this transport for depth images, or for images that are processed by algorithms that need the exact values of the pixels.
- **The decoded images are always** ``bgr8``.
  The publisher converts the images to this encoding before encoding them, so grayscale images become color images and 16-bit images are reduced to 8 bits.
- **A subscriber must receive the stream from its beginning.**
  The headers that are needed to decode the stream are only sent when the encoder starts, which happens when the first image is published to the ``theora`` topic.
  A subscriber that connects later cannot decode the stream, and it logs ``[theora] Packet was not a Theora header`` for every packet.
- **Packets must not be lost.**
  Use a reliable QoS with a queue that is large enough: three header packets are published in a row when the stream starts, so a queue of size 1 is not enough.
  If a packet is lost, the images are corrupted until the next key frame.

The stream is started again, with new headers, when the size of the images changes.

2. ROS API
----------

2.1 Publisher plugin
~~~~~~~~~~~~~~~~~~~~

Published topics
^^^^^^^^^^^^^^^^

``<base_topic>/theora`` (``theora_image_transport/msg/Packet``)
  The packets of the Theora stream: first the header packets, and then one packet for every image.

Parameters
^^^^^^^^^^

The parameters are declared in the node that publishes the images, and they can be changed at runtime.

Their names start with the name of the base topic, relative to the namespace of the node, in which the ``/`` characters are replaced by ``.``.
For example, the quality used for the ``camera/image_raw`` topic is set with the ``camera.image_raw.theora.quality`` parameter.

.. list-table::
   :header-rows: 1
   :widths: 40 10 10 40

   * - Parameter
     - Type
     - Default
     - Description
   * - ``<base_topic>.theora.optimize_for``
     - bool
     - ``true``
     - What the encoder tries to achieve: a constant ``quality`` (``true``) or the ``target_bitrate`` (``false``).
   * - ``<base_topic>.theora.quality``
     - int
     - ``31``
     - Encoding quality, in the range [0, 63].
       Higher values give a better image and a higher bitrate.
       Only used when ``optimize_for`` is ``true``.
   * - ``<base_topic>.theora.target_bitrate``
     - int
     - ``800000``
     - Target bitrate, in bits per second, in the range [0, 99200000].
       Only used when ``optimize_for`` is ``false``.
   * - ``<base_topic>.theora.keyframe_frequency``
     - int
     - ``64``
     - Maximum distance between key frames, in the range [1, 64].
       Lower values make the stream recover faster from a lost packet, and they increase the bitrate.

.. note::

   When the node is in a namespace, the parameters of the publisher are also declared with a leading ``.`` (for example ``.camera.image_raw.theora.quality``).
   These names are deprecated and will be removed, do not use them.

2.2 Subscriber plugin
~~~~~~~~~~~~~~~~~~~~~

Subscribed topics
^^^^^^^^^^^^^^^^^

``<base_topic>/theora`` (``theora_image_transport/msg/Packet``)
  The packets of the Theora stream.

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
   * - ``<base_topic>.theora.post_processing_level``
     - int
     - ``0``
     - Post-processing level, in the range [0, 7].
       Higher values can improve the appearance of the decoded images at the cost of more CPU.

3. Message format
-----------------

Every ``theora_image_transport/msg/Packet`` message contains one packet of the Theora stream.
It is an adaptation of the ``ogg_packet`` structure of `libogg <https://www.xiph.org/ogg/doc/libogg/ogg_packet.html>`_, with the header of the original image.

- The first packets of a stream are the Theora headers. The first one has the ``b_o_s`` (beginning of stream) flag set.
- Each of the following packets contains one frame, and the ``header`` of the message is the header of the corresponding image.

4. C++ API
----------

Most users do not need the C++ API of this package, because the plugins are loaded by ``image_transport``.
The classes of the plugins are described in the :doc:`references`.
