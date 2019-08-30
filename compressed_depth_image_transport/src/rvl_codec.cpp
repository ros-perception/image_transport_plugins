// This file contains a modified copy of the code published by Andrew D. Wilson
// in "Fast Lossless Depth Image Compression" in the proceedings of SIGCHI 2017.
// The original code is licensed under the MIT License.

#include "compressed_depth_image_transport/rvl_codec.h"

namespace compressed_depth_image_transport {

void RvlCodec::EncodeVLE(int value) {
  do {
    int nibble = value & 0x7;        // lower 3 bits
    if (value >>= 3) nibble |= 0x8;  // more to come
    word <<= 4;
    word |= nibble;
    if (++nibblesWritten == 8)  // output word
    {
      *pBuffer++ = word;
      nibblesWritten = 0;
      word = 0;
    }
  } while (value);
}

int RvlCodec::DecodeVLE() {
  unsigned int nibble;
  int value = 0, bits = 29;
  do {
    if (!nibblesWritten) {
      word = *pBuffer++;  // load word
      nibblesWritten = 8;
    }
    nibble = word & 0xf0000000;
    value |= (nibble << 1) >> bits;
    word <<= 4;
    nibblesWritten--;
    bits -= 3;
  } while (nibble & 0x80000000);
  return value;
}

int RvlCodec::CompressRVL(const unsigned short* input, unsigned char* output,
                          int numPixels) {
  buffer = pBuffer = (int*)output;
  nibblesWritten = 0;
  const unsigned short* end = input + numPixels;
  unsigned short previous = 0;
  while (input != end) {
    int zeros = 0, nonzeros = 0;
    for (; (input != end) && !*input; input++, zeros++)
      ;
    EncodeVLE(zeros);  // number of zeros
    for (const unsigned short* p = input; (p != end) && *p++; nonzeros++)
      ;
    EncodeVLE(nonzeros);  // number of nonzeros
    for (int i = 0; i < nonzeros; i++) {
      unsigned short current = *input++;
      int delta = current - previous;
      int positive = (delta << 1) ^ (delta >> 31);
      EncodeVLE(positive);  // nonzero value
      previous = current;
    }
  }
  if (nibblesWritten)  // last few values
    *pBuffer++ = word << 4 * (8 - nibblesWritten);
  return int((unsigned char*)pBuffer - (unsigned char*)buffer);  // num bytes
}

void RvlCodec::DecompressRVL(const unsigned char* input, unsigned short* output,
                             int numPixels) {
  buffer = pBuffer = const_cast<int*>(reinterpret_cast<const int*>(input));
  nibblesWritten = 0;
  unsigned short current, previous = 0;
  int numPixelsToDecode = numPixels;
  while (numPixelsToDecode) {
    int zeros = DecodeVLE();  // number of zeros
    numPixelsToDecode -= zeros;
    for (; zeros; zeros--) *output++ = 0;
    int nonzeros = DecodeVLE();  // number of nonzeros
    numPixelsToDecode -= nonzeros;
    for (; nonzeros; nonzeros--) {
      int positive = DecodeVLE();  // nonzero value
      int delta = (positive >> 1) ^ -(positive & 1);
      current = previous + delta;
      *output++ = current;
      previous = current;
    }
  }
}

}  // namespace compressed_depth_image_transport
