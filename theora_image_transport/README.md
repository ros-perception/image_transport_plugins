# theora_image_transport

`theora_image_transport` is a plugin package for [image_transport](https://docs.ros.org/en/rolling/p/image_transport/).
It enables any node that uses `image_transport` to publish and subscribe to a video stream, without changing its code.

It provides the `theora` transport, which encodes the images with the [Theora](https://www.theora.org/) video codec. It needs much less bandwidth than the transports that compress every image independently.

## Usage

Nodes that publish with `image_transport` advertise the `theora` topic automatically when this package is installed:

```console
$ ros2 topic list
/camera/image_raw
/camera/image_raw/theora
```

To subscribe to it, select the `theora` transport in a node that subscribes with `image_transport`, or decode the images with the `republish` node:

```console
$ ros2 run image_transport republish --ros-args \
    -p in_transport:=theora -p out_transport:=raw \
    -r in/theora:=/camera/image_raw/theora \
    -r out:=/camera/image_decoded
```

## Limitations

- The compression is lossy and the decoded images are always `bgr8`.
- A subscriber must receive the stream from its beginning: it cannot decode the stream if it connects after the first image has been published.
- Packets must not be lost, so use a reliable QoS with a queue larger than 1.

## ROS API

In the names of the parameters, the `/` characters of the base topic are replaced by `.`, for example `camera.image_raw.theora.optimize_for`.
The parameters can be changed at runtime.

### Publisher plugin

#### Published topics

- `<base_topic>/theora` ([theora_image_transport/msg/Packet](https://docs.ros.org/en/rolling/p/theora_image_transport/msg/Packet.html)): the encoded image.

#### Parameters

- `<base_topic>.theora.optimize_for` (`bool`, default: `true`): What the encoder tries to achieve: a constant `quality` (`true`) or the `target_bitrate` (`false`).
- `<base_topic>.theora.quality` (`int`, default: `31`): Encoding quality, in the range [0, 63]. Higher values give a better image and a higher bitrate. Only used when `optimize_for` is `true`.
- `<base_topic>.theora.target_bitrate` (`int`, default: `800000`): Target bitrate, in bits per second, in the range [0, 99200000]. Only used when `optimize_for` is `false`.
- `<base_topic>.theora.keyframe_frequency` (`int`, default: `64`): Maximum distance between key frames, in the range [1, 64]. Lower values make the stream recover faster from a lost packet, and they increase the bitrate.

### Subscriber plugin

#### Subscribed topics

- `<base_topic>/theora` ([theora_image_transport/msg/Packet](https://docs.ros.org/en/rolling/p/theora_image_transport/msg/Packet.html)): the encoded image.

#### Parameters

- `<base_topic>.theora.post_processing_level` (`int`, default: `0`): Post-processing level, in the range [0, 7]. Higher values can improve the appearance of the decoded images at the cost of more CPU.

## Documentation

The full documentation, including a tutorial and the C++ API reference, is in the [doc](https://github.com/ros-perception/image_transport_plugins/tree/rolling/theora_image_transport/doc) directory, and it is published at [docs.ros.org](https://docs.ros.org/en/rolling/p/theora_image_transport/).
