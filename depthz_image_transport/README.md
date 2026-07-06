# depthz_image_transport

`image_transport` plugin for depth images (`32FC1` and `16UC1`).

**By default the 32FC1 transport is LOSSY**: depth is quantized to a uniform
0.1 mm grid before compression, so every decoded pixel is within ±0.05 mm of
the input — far below the noise floor of any real depth camera. Set the
`quantization` parameter to `0.0` for bit-exact lossless mode (NaN payloads
included), or to a larger step for more compression. `16UC1` input (already
integer millimeters) is always compressed losslessly.

The codec is vendored from
[facontidavide/depth_image_compression](https://github.com/facontidavide/depth_image_compression),
where the algorithm is documented. Method by input and configuration:

| input | `quantization` | blob method | guarantee |
|---|---|---|---|
| 32FC1 | `> 0` (default 0.1 mm) | `qpred` | ± step/2 per valid pixel, invalid (NaN/inf/≤0) → NaN |
| 32FC1 | `0.0`, ≤ 65536 distinct values | `dpred` | bit-exact |
| 32FC1 | `0.0`, > 65536 distinct values | `fpred` | bit-exact |
| 16UC1 | (ignored) | `dpred16` | bit-exact |

Lossless `dpred`/`fpred`/`dpred16` blobs are interchangeable with the
standalone library in both directions; `qpred` originates in this plugin
(standalone releases predating it reject it, as this plugin rejects the
standalone library's other methods). Every blob is self-describing:
dimensions, pixel format and quantization step are readable from its header
without decompressing anything.

## Performance

On real full-precision stereo depth (the hardest input: nearly every pixel
carries a unique float bit pattern), the default quantized mode compresses
substantially better than `compressedDepth` while encoding and decoding
several times faster — and with a much finer, explicitly bounded
quantization error than `compressedDepth`'s 16-bit inverse-depth stage.
Coarser steps trade precision for ratio; the lossless mode compresses the
least, since it must reproduce the sensor's mantissa noise bit-exactly.
Compression is data-dependent, so measure on your own streams: the
`benchmark/` directory contains a tool that runs any recorded MCAP depth
topic through the real publisher/subscriber plugins and reports ratio,
throughput, and error-bound verification (see `benchmark/README.md`).

## Usage

Subscribers select the transport with the standard `image_transport`
parameter (in rviz2: the *Transport Hint* dropdown of the Image/Camera
display):

```bash
ros2 run my_pkg my_depth_consumer --ros-args -p image_transport:=depthz
```

Publishers advertise all installed transports, so `<base_topic>/depthz`
appears automatically. To publish only selected transports (saving encoder
CPU), use the publishing node's `enable_pub_plugins` parameter:

```yaml
/camera_node:
  ros__parameters:
    depth.image_rect.enable_pub_plugins:
      - image_transport/raw
      - image_transport/depthz
```

An existing stream can be converted with
`ros2 run image_transport republish` (`out_transport:=depthz`), e.g. for
bag recording: record `<base_topic>/depthz` instead of
`<base_topic>/compressedDepth`.

## Parameters

| parameter | default | meaning |
|---|---|---|
| `<base_topic>.depthz.quantization` | `0.1` | 32FC1 quantization step in **millimeters**; decoded depth is within ± half this step. `0.0` = bit-exact lossless. Ignored for 16UC1. |
| `<base_topic>.depthz.zstd_level` | `1` | zstd level of the entropy stage (1–3); higher is slower with slightly better ratio. |

The published `CompressedImage.format` string advertises the lossiness
(e.g. `32FC1; depthz; lossy 0.100mm`), so bag consumers can tell without
decoding.

Only `32FC1` and `16UC1` encodings are accepted; other encodings are
declined with an error log (use `compressed` or `zstd` for color images).

See `benchmark/README.md` for the tooling that produced the numbers above.
