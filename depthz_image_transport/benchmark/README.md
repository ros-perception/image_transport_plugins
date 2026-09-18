# depthz_image_transport perf benchmark

Local tooling for profiling `DepthzPublisher`/`DepthzSubscriber` with `perf`,
using real depth data. Not part of the package's public surface or CI --
opt in with `-DDEPTHZ_BUILD_BENCHMARK=ON`.

`benchmark_depthz` talks to the plugin exclusively through the public
`image_transport`/`rclcpp` API (`ImageTransport::advertise`,
`image_transport::create_subscription(..., "depthz", ...)`), the same way
any real node would. Profiling this binary profiles exactly what a deployed
publisher or subscriber node spends its time on -- pluginlib loading
included -- not just the vendored codec's hot loop in isolation.

## 1. Get real depth frames

`extract_frames.py` pulls raw `sensor_msgs/Image` frames off an MCAP topic
into a compact `.dzbm` file (see the docstring for the format):

```sh
python3 extract_frames.py \
  --mcap ~/ws_eternal/src/Harvesting/harvest_bringup/test/test_data/rosbags/van_noord_tomato_20250919_181407/20250919_181407_0.mcap \
  --topic /zed_wrist/zed_node/depth/depth_registered \
  --out data/wrist_1920x1200.dzbm
```

Needs `pip install mcap mcap-ros2-support`. That bag has two raw 32FC1
streams from the ZED stereo cameras worth benchmarking:

| topic | resolution | frames |
|---|---|---|
| `/zed_base/zed_node/depth/depth_registered` | 960x600 | 10 |
| `/zed_wrist/zed_node/depth/depth_registered` | 1920x1200 | 10 |

Point `--mcap`/`--topic` at any other bag with a raw (uncompressed) 32FC1 or
16UC1 `sensor_msgs/Image` topic -- it doesn't have to be ZED/depth-camera
specific.

## 2. Build

```sh
colcon build --packages-select depthz_image_transport \
  --cmake-args -DDEPTHZ_BUILD_BENCHMARK=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

`RelWithDebInfo` keeps optimizations on (required -- see the package's own
`CMakeLists.txt` comment) while still emitting frame pointers/debug info for
`perf record -g` to unwind.

## 3. Run

```sh
benchmark_depthz --frames data/wrist_1920x1200.dzbm --mode roundtrip --iterations 300
```

`--mode`:
- `roundtrip` (default): a real `DepthzPublisher` encodes each frame, a real
  `DepthzSubscriber` decodes it. End-to-end throughput and compression
  ratio.
- `encode`: only the publisher plugin's encode path runs (the "subscriber"
  is a bare passthrough callback, kept alive only because
  `image_transport::Publisher` publishes on demand and needs to see a real
  subscriber count -- see its class doc).
- `decode`: all frames are pre-encoded once (untimed) by a real
  `DepthzPublisher`, then the timed loop republishes those captured blobs
  directly onto the internal `<topic>/depthz` topic with a plain
  `rclcpp::Publisher`, bypassing the publisher plugin entirely, while a real
  `DepthzSubscriber` decodes them. This isolates decode cost from encode
  cost.

Other flags: `--transport <name>` (benchmark any installed image_transport
plugin against the same frames, e.g. `compressedDepth`), `--iterations N`
(loop the dataset N times), `--warmup N` (untimed iterations first, so
`thread_local` scratch buffers reach their steady-state size before the
clock starts), `--zstd-level 1-3`, `--quantization MM` (depthz quantization
step in millimeters; the benchmark defaults to `0` = lossless, deliberately
overriding the plugin's own lossy 0.1 mm default so the bit-exact verify
stays meaningful — pass `--quantization 0.1` to measure the plugin's actual
default behavior), `--no-verify` (skip verification entirely; use before
profiling so the comparison doesn't show up as noise in the flamegraph),
`--qos-depth N`.

Output reports wall time, fps, MB/s (raw), compression ratio, and (unless
`--no-verify`) a verification against the source frames: bit-exact for
lossless depthz, the documented ± step/2 error bound plus NaN preservation
for quantized depthz, and informational-only for other transports.

## 4. Profile with perf

```sh
perf stat -d -- benchmark_depthz --frames data/wrist_1920x1200.dzbm --mode encode --no-verify
perf record -g --call-graph dwarf -o encode.perf.data -- \
  benchmark_depthz --frames data/wrist_1920x1200.dzbm --mode encode --no-verify
perf report -i encode.perf.data
```

`run_perf.sh` automates all of the above (build, extract both ZED streams,
run `perf stat` + `perf record` for all 3 modes x 2 resolutions) and drops
results under `results/`:

```sh
./run_perf.sh
```

Because the benchmark drives a `SingleThreadedExecutor` by hand (publish,
then `spin_some` until the callback fires, repeat) rather than a background
spin thread, encode and decode samples land on the same call stack you'd
expect from the source -- `dpred_encode`/`build_value_dict`/`zstd_append`
for encode, `dpred_decode`/`predict_unpack`/`ZSTD_decompressDCtx` for
decode -- with no cross-thread noise to untangle.
