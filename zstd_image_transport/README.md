# zstd_image_transport

`zstd_image_transport` is a plugin package for [image_transport](https://docs.ros.org/en/rolling/p/image_transport/).
It enables any node that uses `image_transport` to publish and subscribe to images compressed with a general purpose lossless algorithm, without changing its code.

It provides the `zstd` transport, which works with any type of image because it compresses the data without converting it.
Despite the name of the package, the images are currently compressed with zlib (DEFLATE), not with Zstandard.

## Usage

Nodes that publish with `image_transport` advertise the `zstd` topic automatically when this package is installed:

```console
$ ros2 topic list
/camera/image_raw
/camera/image_raw/zstd
```

To subscribe to it, select the `zstd` transport in a node that subscribes with `image_transport`, or decode the images with the `republish` node:

```console
$ ros2 run image_transport republish --ros-args \
    -p in_transport:=zstd -p out_transport:=raw \
    -r in/zstd:=/camera/image_raw/zstd \
    -r out:=/camera/image_decoded
```

## ROS API

In the names of the parameters, the `/` characters of the base topic are replaced by `.`, for example `camera.image_raw.zstd.zstd_level`.
The parameters can be changed at runtime.

### Publisher plugin

#### Published topics

- `<base_topic>/zstd` ([sensor_msgs/msg/CompressedImage](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CompressedImage.html)): the encoded image.

#### Parameters

- `<base_topic>.zstd.zstd_level` (`int`, default: `3`): Compression level, in the range [0, 9]. Higher values trade computation time for space savings, and `0` does not compress the data.

### Subscriber plugin

#### Subscribed topics

- `<base_topic>/zstd` ([sensor_msgs/msg/CompressedImage](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CompressedImage.html)): the encoded image.

The subscriber plugin does not have parameters.

## Documentation

The full documentation, including a tutorial and the C++ API reference, is in the [doc](https://github.com/ros-perception/image_transport_plugins/tree/rolling/zstd_image_transport/doc) directory, and it is published at [docs.ros.org](https://docs.ros.org/en/rolling/p/zstd_image_transport/).
