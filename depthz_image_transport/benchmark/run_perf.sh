#!/usr/bin/env bash
# Build depthz_image_transport with the benchmark tool enabled, extract real
# depth frames from an MCAP bag, and profile encode/decode/roundtrip with
# `perf`. See README.md for the full explanation of what this measures.
#
# Usage:
#   ./run_perf.sh [outdir]
#
# Env overrides:
#   MCAP_FILE     path to the source .mcap (default: the ZED tomato bag used
#                 during development, see README.md)
#   ROS_DISTRO_SETUP  path to a ROS 2 setup.bash to source (default: guesses
#                 /opt/ros/*/setup.bash)
#   ITERATIONS    loop count per benchmark run (default: 300)
#   QUANTIZATION  depthz quantization step in mm (default: 0.1, the plugin's
#                 own default; set 0 to profile the lossless path)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$(cd "$PKG_DIR/../.." && pwd)"
OUT_DIR="${1:-$SCRIPT_DIR/results}"
mkdir -p "$OUT_DIR" "$SCRIPT_DIR/data"

MCAP_FILE="${MCAP_FILE:-$HOME/ws_eternal/src/Harvesting/harvest_bringup/test/test_data/rosbags/van_noord_tomato_20250919_181407/20250919_181407_0.mcap}"
ITERATIONS="${ITERATIONS:-300}"
QUANTIZATION="${QUANTIZATION:-0.1}"

if [ -z "${ROS_DISTRO_SETUP:-}" ]; then
  ROS_DISTRO_SETUP="$(ls /opt/ros/*/setup.bash 2>/dev/null | head -1 || true)"
fi
if [ -z "$ROS_DISTRO_SETUP" ] || [ ! -f "$ROS_DISTRO_SETUP" ]; then
  echo "error: could not find a ROS 2 setup.bash (set ROS_DISTRO_SETUP)" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$ROS_DISTRO_SETUP"

echo "== building depthz_image_transport (benchmark enabled) =="
COLCON_WS="$SCRIPT_DIR/colcon_ws"
mkdir -p "$COLCON_WS"
(
  cd "$COLCON_WS"
  colcon build --packages-select depthz_image_transport \
    --base-paths "$WS_DIR" \
    --cmake-args -DDEPTHZ_BUILD_BENCHMARK=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
)
# shellcheck disable=SC1091
source "$COLCON_WS/install/setup.bash"

BENCH_BIN="$COLCON_WS/install/depthz_image_transport/lib/depthz_image_transport/benchmark_depthz"
if [ ! -x "$BENCH_BIN" ]; then
  echo "error: benchmark_depthz was not built at $BENCH_BIN" >&2
  exit 1
fi

if [ ! -f "$MCAP_FILE" ]; then
  echo "error: MCAP_FILE not found: $MCAP_FILE" >&2
  echo "       point MCAP_FILE at any bag with a raw 32FC1 or 16UC1 sensor_msgs/Image topic" >&2
  exit 1
fi

echo "== extracting real depth frames from $MCAP_FILE =="
SMALL="$SCRIPT_DIR/data/base_960x600.dzbm"
LARGE="$SCRIPT_DIR/data/wrist_1920x1200.dzbm"
[ -f "$SMALL" ] || python3 "$SCRIPT_DIR/extract_frames.py" \
  --mcap "$MCAP_FILE" --topic /zed_base/zed_node/depth/depth_registered --out "$SMALL"
[ -f "$LARGE" ] || python3 "$SCRIPT_DIR/extract_frames.py" \
  --mcap "$MCAP_FILE" --topic /zed_wrist/zed_node/depth/depth_registered --out "$LARGE"

run_one() {
  local dataset="$1" mode="$2" tag="$3"
  echo "-- perf stat: $tag / $mode --"
  perf stat -d -o "$OUT_DIR/${tag}_${mode}.stat.txt" -- \
    "$BENCH_BIN" --frames "$dataset" --mode "$mode" --iterations "$ITERATIONS" \
    --quantization "$QUANTIZATION" --no-verify
  echo "-- perf record: $tag / $mode --"
  perf record -g --call-graph dwarf -o "$OUT_DIR/${tag}_${mode}.perf.data" -- \
    "$BENCH_BIN" --frames "$dataset" --mode "$mode" --iterations "$ITERATIONS" \
    --quantization "$QUANTIZATION" --no-verify
}

for tag_dataset in "small:$SMALL" "large:$LARGE"; do
  tag="${tag_dataset%%:*}"
  dataset="${tag_dataset#*:}"
  for mode in encode decode roundtrip; do
    run_one "$dataset" "$mode" "$tag"
  done
done

echo
echo "== done =="
echo "perf stat summaries : $OUT_DIR/*.stat.txt"
echo "perf record captures: $OUT_DIR/*.perf.data"
echo "inspect a capture with, e.g.:"
echo "  perf report -i $OUT_DIR/large_encode.perf.data"
echo "  perf script -i $OUT_DIR/large_encode.perf.data | stackcollapse-perf.pl | flamegraph.pl > large_encode.svg"
