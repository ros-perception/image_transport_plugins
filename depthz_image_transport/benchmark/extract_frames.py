#!/usr/bin/env python3

# Copyright (c) 2026, Davide Faconti
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Extract raw sensor_msgs/Image frames from an MCAP topic into a .dzbm file.

.dzbm is a minimal container used by benchmark_depthz.cpp so the benchmark
does not need to link an MCAP reader: it just mmaps/reads flat records.
Row padding (step != width * bpp) is stripped here, so every frame in the
file is stored contiguously.

Format (little-endian):
    magic    : 4 bytes  b'DZBM'
    version  : uint32   1
    count    : uint32
    frames[count]:
        encoding_len : uint8
        encoding     : encoding_len bytes (ascii, e.g. '32FC1')
        width        : uint32
        height       : uint32
        data_len     : uint32
        data         : data_len bytes, row-major, contiguous
"""
import argparse
import struct
import sys

MAGIC = b'DZBM'
VERSION = 1

# bytes-per-pixel for the encodings depthz_image_transport supports.
BPP = {'32FC1': 4, '16UC1': 2}


def repack_contiguous(msg):
    bpp = BPP.get(msg.encoding)
    if bpp is None:
        raise ValueError(
            f"unsupported encoding '{msg.encoding}' (depthz only handles 32FC1/16UC1)")
    row_bytes = msg.width * bpp
    if msg.step == row_bytes:
        return bytes(msg.data[:row_bytes * msg.height])
    # Rows are padded (step > width * bpp): copy row-by-row.
    out = bytearray(row_bytes * msg.height)
    src = bytes(msg.data)
    for y in range(msg.height):
        out[y * row_bytes:(y + 1) * row_bytes] = src[y * msg.step:y * msg.step + row_bytes]
    return bytes(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mcap', required=True, help='input .mcap file')
    parser.add_argument('--topic', required=True, help='sensor_msgs/Image topic to extract')
    parser.add_argument('--out', required=True, help='output .dzbm file')
    parser.add_argument(
        '--max-frames', type=int, default=0,
        help='stop after this many frames (0 = all)')
    args = parser.parse_args()

    try:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
    except ImportError:
        sys.exit(
            "error: this script needs the 'mcap' and 'mcap_ros2' Python packages "
            '(pip install mcap mcap-ros2-support)')

    frames = []
    with open(args.mcap, 'rb') as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _schema, _channel, _message, ros_msg in reader.iter_decoded_messages(
                topics=[args.topic]):
            frames.append(ros_msg)
            if args.max_frames and len(frames) >= args.max_frames:
                break

    if not frames:
        sys.exit(f"error: no messages found on topic '{args.topic}' in {args.mcap}")

    with open(args.out, 'wb') as out:
        out.write(MAGIC)
        out.write(struct.pack('<II', VERSION, len(frames)))
        for msg in frames:
            packed = repack_contiguous(msg)
            enc = msg.encoding.encode('ascii')
            out.write(struct.pack('<B', len(enc)))
            out.write(enc)
            out.write(struct.pack('<III', msg.width, msg.height, len(packed)))
            out.write(packed)

    total_bytes = sum(BPP[m.encoding] * m.width * m.height for m in frames)
    print(
        f'wrote {len(frames)} frames ({frames[0].encoding}, '
        f'{frames[0].width}x{frames[0].height}) -> {args.out} '
        f'({total_bytes / 1e6:.1f} MB raw)')


if __name__ == '__main__':
    main()
