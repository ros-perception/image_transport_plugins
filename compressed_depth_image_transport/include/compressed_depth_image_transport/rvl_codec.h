#ifndef COMPRESSED_DEPTH_IMAGE_TRANSPORT_RVL_CODEC_H_
#define COMPRESSED_DEPTH_IMAGE_TRANSPORT_RVL_CODEC_H_

namespace compressed_depth_image_transport {

class RvlCodec {
 public:
  int CompressRVL(const unsigned short* input, unsigned char* output,
                  int numPixels);
  void DecompressRVL(const unsigned char* input, unsigned short* output,
                     int numPixels);

 private:
  void EncodeVLE(int value);
  int DecodeVLE();
  int *buffer, *pBuffer, word, nibblesWritten;
};

}  // namespace compressed_depth_image_transport

#endif  // COMPRESSED_DEPTH_IMAGE_TRANSPORT_RVL_CODEC_H_
