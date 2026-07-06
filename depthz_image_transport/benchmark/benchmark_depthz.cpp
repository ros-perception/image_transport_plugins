// Copyright (c) 2026, Davide Faconti
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Black-box perf benchmark for depthz_image_transport, and a like-for-like
// baseline tool for any other image_transport plugin (--transport). This
// talks to the publisher/subscriber plugins exclusively through the public
// image_transport/rclcpp API -- the same way any real node would -- so
// "perf record" on this binary profiles exactly what a deployed publisher
// or subscriber node would spend its time on, pluginlib loading included.
//
// Three modes, selected with --mode:
//   roundtrip (default): a real publisher plugin encodes each frame and a
//     real subscriber plugin decodes it, end to end.
//   encode: only the publisher plugin's encode path runs. The "subscriber"
//     is a bare passthrough callback (so image_transport still sees a
//     subscriber and does not skip the on-demand encode), but no decoding
//     happens.
//   decode: frames are pre-encoded once (untimed) by a real publisher
//     plugin, then the timed loop republishes those captured blobs directly
//     onto the transport's internal sub-topic with a plain
//     rclcpp::Publisher, bypassing the publisher plugin entirely, while a
//     real subscriber plugin decodes them.
//
// Usage:
//   benchmark_depthz --frames data/wrist_1920x1200.dzbm --mode roundtrip
//     --iterations 200 --zstd-level 1
//   benchmark_depthz --frames data/wrist_1920x1200.dzbm --transport compressedDepth
//     --mode roundtrip --iterations 200
//
// See run_perf.sh for how to wrap this in `perf record`/`perf stat`.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace
{

using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;

struct Options
{
  std::string frames_path;
  std::string mode = "roundtrip";
  std::string transport = "depthz";
  int iterations = 200;
  int warmup = 5;
  int zstd_level = 1;
  // 0 = lossless (the plugin's own default is lossy 0.1 mm; see print_usage).
  double quantization_mm = 0.0;
  int qos_depth = 10;
  bool verify = true;
  std::chrono::milliseconds drain_timeout{5000};
};

void print_usage(const char * argv0)
{
  std::cerr <<
    "Usage: " << argv0 << " --frames <path.dzbm> [--mode roundtrip|encode|decode]\n"
    "         [--transport depthz|compressedDepth|...] [--iterations N] [--warmup N]\n"
    "         [--zstd-level 1-3] [--quantization MM] [--qos-depth N] [--no-verify]\n"
    "\n"
    "--transport selects any installed image_transport publisher/subscriber plugin\n"
    "by its registered transport name, not just depthz -- e.g. --transport\n"
    "compressedDepth benchmarks the PNG-based transport against the same frames,\n"
    "for a like-for-like comparison. --zstd-level and --quantization only apply to\n"
    "depthz; --quantization is the step in millimeters (default 0 = lossless here,\n"
    "overriding the plugin's own lossy 0.1 mm default so the bit-exact verify\n"
    "stays meaningful). With a nonzero step, verify checks the +/- step/2 error\n"
    "bound and NaN preservation instead of bit-exactness.\n";
}

Options parse_args(int argc, char ** argv)
{
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
        if (i + 1 >= argc) {
          print_usage(argv[0]);
          std::exit(2);
        }
        return argv[++i];
      };
    if (arg == "--frames") {
      opt.frames_path = next();
    } else if (arg == "--mode") {
      opt.mode = next();
    } else if (arg == "--transport") {
      opt.transport = next();
    } else if (arg == "--iterations") {
      opt.iterations = std::stoi(next());
    } else if (arg == "--warmup") {
      opt.warmup = std::stoi(next());
    } else if (arg == "--zstd-level") {
      opt.zstd_level = std::stoi(next());
    } else if (arg == "--quantization") {
      opt.quantization_mm = std::stod(next());
    } else if (arg == "--qos-depth") {
      opt.qos_depth = std::stoi(next());
    } else if (arg == "--no-verify") {
      opt.verify = false;
    } else if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      print_usage(argv[0]);
      std::exit(2);
    }
  }
  if (opt.frames_path.empty()) {
    std::cerr << "error: --frames is required\n";
    print_usage(argv[0]);
    std::exit(2);
  }
  if (opt.mode != "roundtrip" && opt.mode != "encode" && opt.mode != "decode") {
    std::cerr << "error: --mode must be roundtrip, encode, or decode\n";
    std::exit(2);
  }
  return opt;
}

// ---- .dzbm loader (see extract_frames.py for the format) -------------------

struct RawFrame
{
  std::string encoding;
  uint32_t width = 0;
  uint32_t height = 0;
  std::vector<uint8_t> data;
};

template<class T>
T read_pod(std::ifstream & f)
{
  T v{};
  f.read(reinterpret_cast<char *>(&v), sizeof(T));
  if (!f) {
    throw std::runtime_error("benchmark_depthz: truncated .dzbm file");
  }
  return v;
}

std::vector<RawFrame> load_dzbm(const std::string & path)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    throw std::runtime_error("benchmark_depthz: cannot open " + path);
  }
  char magic[4];
  f.read(magic, 4);
  if (!f || std::memcmp(magic, "DZBM", 4) != 0) {
    throw std::runtime_error("benchmark_depthz: " + path + " is not a .dzbm file");
  }
  const uint32_t version = read_pod<uint32_t>(f);
  if (version != 1) {
    throw std::runtime_error("benchmark_depthz: unsupported .dzbm version " +
      std::to_string(version));
  }
  const uint32_t count = read_pod<uint32_t>(f);
  std::vector<RawFrame> frames;
  frames.reserve(count);
  for (uint32_t i = 0; i < count; ++i) {
    RawFrame frame;
    const uint8_t enc_len = read_pod<uint8_t>(f);
    frame.encoding.resize(enc_len);
    f.read(frame.encoding.data(), enc_len);
    frame.width = read_pod<uint32_t>(f);
    frame.height = read_pod<uint32_t>(f);
    const uint32_t data_len = read_pod<uint32_t>(f);
    frame.data.resize(data_len);
    f.read(reinterpret_cast<char *>(frame.data.data()), data_len);
    if (!f) {
      throw std::runtime_error("benchmark_depthz: truncated frame " + std::to_string(i));
    }
    frames.push_back(std::move(frame));
  }
  return frames;
}

size_t bpp_of(const std::string & encoding)
{
  return encoding == "32FC1" ? 4 : 2;
}

Image::SharedPtr make_image(const RawFrame & frame, uint32_t frame_idx)
{
  auto msg = std::make_shared<Image>();
  // frame_idx rides in header.stamp.nanosec so the decode callback can find
  // the original frame to verify against (plugins copy the header verbatim).
  msg->header.stamp.nanosec = frame_idx;
  msg->header.frame_id = "bench";
  msg->encoding = frame.encoding;
  msg->width = frame.width;
  msg->height = frame.height;
  msg->is_bigendian = 0;
  msg->step = frame.width * static_cast<uint32_t>(bpp_of(frame.encoding));
  msg->data = frame.data;
  return msg;
}

// ---- benchmark bookkeeping --------------------------------------------------

struct Stats
{
  std::atomic<uint64_t> received{0};
  std::atomic<uint64_t> compressed_bytes{0};
  std::atomic<uint64_t> mismatches{0};
};

bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::atomic<uint64_t> & counter, uint64_t target,
  std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (counter.load() < target) {
    executor.spin_some(std::chrono::milliseconds(10));
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
  }
  return true;
}

int run(const Options & opt, const std::vector<RawFrame> & frames, size_t raw_bytes_per_pass)
{
  auto node_options = rclcpp::NodeOptions().use_intra_process_comms(true);
    // No slashes in the topic name: keeps the plugin's
    // "<base_topic>.depthz.zstd_level" parameter name simple to predict
    // (see depthz_publisher.cpp's advertiseImpl, which otherwise has to
    // strip the node namespace out of base_topic).
  const std::string base_topic = "depth_bench";
  auto node = std::make_shared<rclcpp::Node>("depthz_benchmark", node_options);

  rclcpp::QoS qos(opt.qos_depth);
  qos.reliable();

  image_transport::ImageTransport it(*node);
  Stats stats;

    // Publisher side: always create it, even in "decode" mode, so the
    // pre-encode capture pass below can use it.
  image_transport::Publisher pub = it.advertise(base_topic, qos, false);

  if (opt.transport == "depthz") {
    const std::string level_param = base_topic + ".depthz.zstd_level";
    const std::string quantization_param = base_topic + ".depthz.quantization";
    if (node->has_parameter(level_param) && node->has_parameter(quantization_param)) {
      node->set_parameter(rclcpp::Parameter(level_param, opt.zstd_level));
      node->set_parameter(rclcpp::Parameter(quantization_param, opt.quantization_mm));
    } else {
      std::cerr << "warning: depthz parameters were not declared -- is "
        "depthz_image_transport built/sourced? Falling back to its defaults "
        "(NOTE: the plugin's default is LOSSY 0.1 mm quantization).\n";
    }
  }

  const std::string transport_topic = base_topic + "/" + opt.transport;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

    // Raw subscription on the internal <base_topic>/<transport> topic:
    // exists in every mode so image_transport sees a subscriber and
    // actually runs the publisher plugin's encode path (transports are
    // on-demand, see image_transport::Publisher's class doc) and so we can
    // measure compressed size regardless of mode.
  auto passthrough_cb = [&stats](const CompressedImage::ConstSharedPtr & msg) {
      stats.compressed_bytes += msg->data.size();
    };
  auto raw_sub = node->create_subscription<CompressedImage>(
      transport_topic, qos, passthrough_cb);

  image_transport::Subscriber sub;
  rclcpp::Publisher<CompressedImage>::SharedPtr raw_pub;
  std::vector<CompressedImage::SharedPtr> captured_blobs;

    // Lossy depthz runs verify against the documented contract instead of
    // bit-exactness: valid pixels within +/- step/2 (plus a few float ULPs),
    // invalid pixels (NaN/inf/<= 0) decoded as NaN.
  const bool lossy_depthz = opt.transport == "depthz" && opt.quantization_mm > 0.0;
  const float qtol = static_cast<float>(opt.quantization_mm * 1e-3 * 0.5);
  auto decode_cb =
    [&stats, &frames, &opt, lossy_depthz, qtol](const Image::ConstSharedPtr & msg) {
      stats.received++;
      if (!opt.verify) {
        return;
      }
      const uint32_t frame_idx = msg->header.stamp.nanosec;
      if (frame_idx >= frames.size()) {
        stats.mismatches++;
        return;
      }
      const RawFrame & original = frames[frame_idx];
      if (msg->data.size() != original.data.size()) {
        stats.mismatches++;
        return;
      }
      if (lossy_depthz && original.encoding == "32FC1") {
        const float * in = reinterpret_cast<const float *>(original.data.data());
        const float * out = reinterpret_cast<const float *>(msg->data.data());
        const size_t n = original.data.size() / 4;
        for (size_t i = 0; i < n; ++i) {
          const bool valid_in = std::isfinite(in[i]) && in[i] > 0.0f;
          const float tol = qtol + 4.0f * std::numeric_limits<float>::epsilon() * in[i];
          if (valid_in ? !(std::fabs(out[i] - in[i]) <= tol) : !std::isnan(out[i])) {
            stats.mismatches++;
            return;
          }
        }
        return;
      }
      if (std::memcmp(msg->data.data(), original.data.data(), original.data.size()) != 0) {
        stats.mismatches++;
      }
    };

  if (opt.mode == "roundtrip" || opt.mode == "decode") {
    sub = image_transport::create_subscription(
        *node, base_topic, decode_cb, opt.transport, qos);
  }
  if (opt.mode == "decode") {
    raw_pub = node->create_publisher<CompressedImage>(transport_topic, qos);
  }

    // Wait for the DDS/intra-process graph to actually match publishers
    // with subscribers before timing anything.
  const auto discovery_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (pub.getNumSubscribers() == 0 ||
    (sub && sub.getNumPublishers() == 0))
  {
    executor.spin_some(std::chrono::milliseconds(10));
    if (std::chrono::steady_clock::now() > discovery_deadline) {
      std::cerr << "error: publisher/subscriber never matched (pluginlib load failure? "
        "did you source install/setup.bash after building with depthz_image_transport?)\n";
      return 1;
    }
  }

  if (opt.mode == "decode") {
      // Untimed pre-pass: encode every frame once for real, capture the
      // resulting blobs so the timed loop can republish them without
      // paying encode cost.
    captured_blobs.reserve(frames.size());
    std::atomic<uint64_t> captured{0};
    auto capture_cb = [&captured_blobs, &captured](const CompressedImage::ConstSharedPtr & msg) {
        captured_blobs.push_back(std::make_shared<CompressedImage>(*msg));
        captured++;
      };
    auto capture_sub = node->create_subscription<CompressedImage>(
        transport_topic, qos, capture_cb);
    executor.spin_some(std::chrono::milliseconds(10));    // let it match too
    for (uint32_t i = 0; i < frames.size(); ++i) {
      pub.publish(*make_image(frames[i], i));
      if (!spin_until(executor, captured, i + 1, opt.drain_timeout)) {
        std::cerr << "error: pre-encode capture timed out waiting for frame " << i << "\n";
        return 1;
      }
    }
    if (captured_blobs.size() != frames.size()) {
      std::cerr << "error: pre-encode capture got " << captured_blobs.size() <<
        "/" << frames.size() << " blobs\n";
      return 1;
    }
    stats.compressed_bytes = 0;    // don't double-count the pre-pass
    std::cout << "pre-encoded " << captured_blobs.size() << " frames for decode-only mode\n";
  }

    // Messages are built once and restamped per publish: a fresh Image
    // would copy the multi-MB frame inside the timed loop and distort the
    // measurement (publish takes the message by const ref).
  std::vector<Image::SharedPtr> images;
  if (opt.mode != "decode") {
    images.reserve(frames.size());
    for (uint32_t i = 0; i < frames.size(); ++i) {
      images.push_back(make_image(frames[i], i));
    }
  }
  auto publish_one = [&](uint32_t iteration, uint32_t frame_idx) {
      if (opt.mode == "decode") {
        CompressedImage & blob = *captured_blobs[frame_idx];
        blob.header.stamp.sec = static_cast<int32_t>(iteration);
        raw_pub->publish(blob);
      } else {
        Image & msg = *images[frame_idx];
        msg.header.stamp.sec = static_cast<int32_t>(iteration);
        pub.publish(msg);
      }
    };

    // Warmup: lets thread_local scratch buffers in the codec grow to their
    // steady-state size and warms caches/branch predictors before we start
    // the clock (see depth_codec.cpp's tl_encode_scratch()/tl_decode_scratch()).
  for (int w = 0; w < opt.warmup; ++w) {
    for (uint32_t f = 0; f < frames.size(); ++f) {
      publish_one(1000000 + w, f);
      executor.spin_some(std::chrono::milliseconds(20));
    }
  }
  stats.received = 0;
  stats.compressed_bytes = 0;
  stats.mismatches = 0;

  const uint64_t total_frames = static_cast<uint64_t>(opt.iterations) * frames.size();
  const auto t0 = std::chrono::steady_clock::now();
  for (int it_idx = 0; it_idx < opt.iterations; ++it_idx) {
    for (uint32_t f = 0; f < frames.size(); ++f) {
      publish_one(static_cast<uint32_t>(it_idx), f);
      executor.spin_some(std::chrono::milliseconds(20));
    }
  }
  bool drained = true;
  if (opt.mode == "encode") {
      // Best effort: nothing decodes in this mode, so there is no reliable
      // completion counter to wait on.
    spin_until(executor, stats.compressed_bytes, 1, opt.drain_timeout);
  } else {
    drained = spin_until(executor, stats.received, total_frames, opt.drain_timeout);
  }
  const auto t1 = std::chrono::steady_clock::now();
  const double elapsed_s = std::chrono::duration<double>(t1 - t0).count();

  if (!drained) {
    std::cerr << "warning: only " << stats.received.load() << "/" << total_frames <<
      " frames were decoded before the drain timeout -- results below are unreliable\n";
  }

  const double raw_mb = static_cast<double>(raw_bytes_per_pass) * opt.iterations / 1e6;
  const double compressed_mb = static_cast<double>(stats.compressed_bytes.load()) / 1e6;

  std::cout   << "\n--- " << opt.transport << " " << opt.mode << " benchmark ---\n"
              << "frames published : " << total_frames << " (" << opt.iterations <<
    " x " << frames.size() << ")\n"
              << "wall time        : " << elapsed_s << " s\n"
              << "throughput       : " << (total_frames / elapsed_s) << " fps, " <<
    (raw_mb / elapsed_s) << " MB/s (raw)\n";
  if (stats.compressed_bytes.load() > 0) {
    std::cout   << "raw size         : " << raw_mb << " MB\n"
                << "compressed size  : " << compressed_mb << " MB\n"
                << "compression ratio: " << (raw_mb / compressed_mb) << "x\n";
  }
  if (opt.verify && (opt.mode == "roundtrip" || opt.mode == "decode")) {
      // For depthz a mismatch is a real bug: against bit-exactness when
      // lossless, against the documented +/- step/2 error bound and NaN
      // preservation when quantizing. Other transports (e.g.
      // compressedDepth, which quantizes 32FC1 to 16 bits before PNG with
      // no comparable contract) may legitimately be lossy, so mismatches
      // are only reported, not treated as a failure.
    const bool contract_expected = opt.transport == "depthz";
    const char * ok_label = lossy_depthz ? "  (within error bound)" : "  (bit-exact)";
    std::cout   << "decoded frames   : " << stats.received.load() << "\n"
                << "verify mismatches: " << stats.mismatches.load() <<
      (stats.mismatches.load() == 0 ? ok_label :
    contract_expected ? "  <-- CORRUPTION" : "  (expected: lossy transport)") << "\n";
    if (contract_expected && stats.mismatches.load() > 0) {
      return 1;
    }
  }
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  const Options opt = parse_args(argc, argv);

  std::vector<RawFrame> frames;
  try {
    frames = load_dzbm(opt.frames_path);
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
  if (frames.empty()) {
    std::cerr << "error: " << opt.frames_path << " contains no frames\n";
    return 1;
  }
  size_t raw_bytes_per_pass = 0;
  for (const auto & f : frames) {
    raw_bytes_per_pass += static_cast<size_t>(f.width) * f.height * bpp_of(f.encoding);
  }
  std::cout << "loaded " << frames.size() << " frames (" << frames.front().encoding
            << ", " << frames.front().width << "x" << frames.front().height << ") from "
            << opt.frames_path << " -- " << raw_bytes_per_pass / 1e6 << " MB/pass\n";

  rclcpp::init(0, nullptr);
  const int result = run(opt, frames, raw_bytes_per_pass);
  rclcpp::shutdown();
  return result;
}
