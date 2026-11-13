## Overview

`compressed_depth_image_transport` is a plugin package for
[image_transport](https://index.ros.org/p/image_transport). It enables
any node using `image_transport` classes to publish and subscribe to
compressed depth image/map topics. Compression parameters can be changed
on the fly using the dynamic_reconfigure package.

`compressed_depth_image_transport` works only with floating-point or
16-bit integer encoded depth images.

**`'Before reading this page, make sure you understand the [[https://index.ros.org/p/image_transport|image_transport]] package`**'

## Tutorials

See [Tutorials](https://index.ros.org/p/image_transport) for general
instruction on using `image_transport`.
`compressed_depth_image_transport` works similar to
`compressed_image_transport` which is featured in the
`image_transport` tutorial
[Examining the Simple Image Publisher and Subscriber](https://index.ros.org/p/image_transport).

## ROS API

### Publisher plugin

#### Published Topics

- `<base_topic>/compressedDepth`
  ([sensor_msgs/msg/CompressedImage](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CompressedImage.html)):
  Compressed depth transport subtopic.



```
{{#!wiki version diamondback_and_newer
{{{{#!wiki version melodic_and_older

==== Parameters ====

===== Reconfigurable parameters =====

Starting in Diamondback, all publisher parameters are dynamically reconfigurable per-topic. See [[dynamic_reconfigure]] for details on dynamically reconfigurable parameters.

 * {{{<base_topic>/compressedDepth/depth_max
```

 (`double`, default: `10.0`): Maximum depth
value (meter), in the range \[1, 100\]

- `<base_topic>/compressedDepth/depth_quantization` (`double`, default:
  `100`): Depth value at which the sensor accuracy is 1 m (Kinect:
  >75), in the range \[1, 150\].
- `<base_topic>/compressedDepth/png_level` (`int`, default:
  `9 (or 1 since 2023)`): PNG compression level, in the range \[1, 9\].
  Higher values trade computation time for space savings.

}}}}



```
{#!wiki version noetic_and_newer

==== Parameters ====

===== Reconfigurable parameters =====

Starting in Diamondback, all publisher parameters are dynamically reconfigurable per-topic. See [[dynamic_reconfigure]] for details on dynamically reconfigurable parameters.

 * {{{<base_topic>/compressedDepth/format
```

 (`enum [png | rvl]`, default: `png`):
Compression format. The RVL format has been added in Noetic and is not
available on previous ROS versions. Previous ROS versions have always
used PNG.

- `<base_topic>/compressedDepth/depth_max` (`double`, default: `10.0`):
  Maximum depth value (meter), in the range \[1, 100\]
- `<base_topic>/compressedDepth/depth_quantization` (`double`, default:
  `100`): Depth value at which the sensor accuracy is 1 m (Kinect:
  >75), in the range \[1, 150\].
- `<base_topic>/compressedDepth/png_level` (`int`, default:
  `9 (or 1 since 2023)`): PNG compression level, in the range \[1, 9\].
  Higher values trade computation time for space savings.

}}}} }}}}}

### Subscriber plugin

#### Subscribed Topics

- `<base_topic>/compressedDepth`
  ([sensor_msgs/msg/CompressedImage](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CompressedImage.html)):
  Compressed depth transport subtopic.
