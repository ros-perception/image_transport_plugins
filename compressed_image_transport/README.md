# compressed_image_transport

`compressed_image_transport` is a plugin package for [image_transport](https://docs.ros.org/en/rolling/p/image_transport/).
It enables any node that uses `image_transport` to publish and subscribe to compressed images, without changing its code.

It provides the `compressed` transport, which sends every image as a JPEG, PNG or TIFF file.
For depth images use `compressed_depth_image_transport` instead.

## Usage

Nodes that publish with `image_transport` advertise the `compressed` topic automatically when this package is installed:

```console
$ ros2 topic list
/camera/image_raw
/camera/image_raw/compressed
```

To subscribe to it, select the `compressed` transport in a node that subscribes with `image_transport`, or decode the images with the `republish` node:

```console
$ ros2 run image_transport republish --ros-args \
    -p in_transport:=compressed -p out_transport:=raw \
    -r in/compressed:=/camera/image_raw/compressed \
    -r out:=/camera/image_decoded
```

## ROS API

In the names of the parameters, the `/` characters of the base topic are replaced by `.`, for example `camera.image_raw.compressed.format`.
The parameters can be changed at runtime.

### Publisher plugin

#### Published topics

- `<base_topic>/compressed` ([sensor_msgs/msg/CompressedImage](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CompressedImage.html)): the encoded image.

#### Parameters

- `<base_topic>.compressed.format` (`string`, default: `jpeg`): Compression format: `jpeg`, `png` or `tiff`.
- `<base_topic>.compressed.jpeg_quality` (`int`, default: `95`): JPEG quality, in the range [1, 100].
- `<base_topic>.compressed.jpeg_compress_bayer` (`bool`, default: `false`): Compress Bayer images with JPEG without converting them to grayscale first.
- `<base_topic>.compressed.png_level` (`int`, default: `3`): PNG compression level, in the range [0, 9].
- `<base_topic>.compressed.tiff.res_unit` (`string`, default: `inch`): Resolution unit stored in the TIFF file: `none`, `inch` or `centimeter`.
- `<base_topic>.compressed.tiff.xdpi` (`int`, default: `-1`): Horizontal resolution stored in the TIFF file.
- `<base_topic>.compressed.tiff.ydpi` (`int`, default: `-1`): Vertical resolution stored in the TIFF file.

### Subscriber plugin

#### Subscribed topics

- `<base_topic>/compressed` ([sensor_msgs/msg/CompressedImage](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CompressedImage.html)): the encoded image.

#### Parameters

- `<base_topic>.compressed.mode` (`string`, default: `unchanged`): How the image is decoded by OpenCV: `unchanged`, `gray` (convert to a single channel) or `color` (convert to three channels).

## Documentation

The full documentation, including a tutorial and the C++ API reference, is in the [doc](https://github.com/ros-perception/image_transport_plugins/tree/rolling/compressed_image_transport/doc) directory, and it is published at [docs.ros.org](https://docs.ros.org/en/rolling/p/compressed_image_transport/).
