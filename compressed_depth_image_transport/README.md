# compressed_depth_image_transport

`compressed_depth_image_transport` is a plugin package for [image_transport](https://docs.ros.org/en/rolling/p/image_transport/).
It enables any node that uses `image_transport` to publish and subscribe to compressed depth images, without changing its code.

It provides the `compressedDepth` transport, which compresses the images with PNG or RVL.
It only works with single-channel 16-bit (`16UC1`, depth in millimeters) and 32-bit floating point (`32FC1`, depth in meters) images.
For color or grayscale images use `compressed_image_transport` instead.

## Usage

Nodes that publish with `image_transport` advertise the compressed topic automatically when this package is installed:

```console
$ ros2 topic list
/camera/depth/image_raw
/camera/depth/image_raw/compressedDepth
```

To subscribe to it, select the `compressedDepth` transport in a node that subscribes with `image_transport`, or decompress the images with the `republish` node:

```console
$ ros2 run image_transport republish --ros-args \
    -p in_transport:=compressedDepth -p out_transport:=raw \
    -r in/compressedDepth:=/camera/depth/image_raw/compressedDepth \
    -r out:=/camera/depth/image_decompressed
```

## ROS API

### Publisher plugin

#### Published topics

- `<base_topic>/compressedDepth` ([sensor_msgs/msg/CompressedImage](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CompressedImage.html)): the compressed depth image.

#### Parameters

The parameters are declared in the node that publishes the images, and they can be changed at runtime.
In their names, the `/` characters of the base topic are replaced by `.`, for example `camera.depth.image_raw.compressedDepth.format`.

- `<base_topic>.compressedDepth.format` (`string`, default: `png`): compression format, `png` or `rvl`.
- `<base_topic>.compressedDepth.depth_max` (`double`, default: `10.0`): maximum depth value in meters, in the range [1, 100]. Measurements further away are discarded.
- `<base_topic>.compressedDepth.depth_quantization` (`double`, default: `100.0`): depth value at which the sensor accuracy is 1 m (Kinect: >75), in the range [1, 150]. Only used for floating point images.
- `<base_topic>.compressedDepth.png_level` (`int`, default: `3`): PNG compression level, in the range [0, 9]. Higher values trade computation time for space savings.

### Subscriber plugin

#### Subscribed topics

- `<base_topic>/compressedDepth` ([sensor_msgs/msg/CompressedImage](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CompressedImage.html)): the compressed depth image.

The subscriber plugin does not have parameters.

## Documentation

The full documentation, including tutorials, a description of the compression and the C++ API reference, is in the [doc](https://github.com/ros-perception/image_transport_plugins/tree/rolling/compressed_depth_image_transport/doc) directory, and it is published at [docs.ros.org](https://docs.ros.org/en/rolling/p/compressed_depth_image_transport/).
