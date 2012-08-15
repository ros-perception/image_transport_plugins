#ifndef COMPRESSED_DEPTH_IMAGE_TRANSPORT_COMPRESSION_COMMON
#define COMPRESSED_DEPTH_IMAGE_TRANSPORT_COMPRESSION_COMMON

namespace compressed_depth_image_transport
{

// Compression formats
enum compressionFormat
{
  UNDEFINED = -1, INV_DEPTH
};

// Compression configuration
struct ConfigHeader
{
  // compression format
  compressionFormat format;
  // quantization parameters (used in depth image compression)
  float depthParam[2];
};

} //namespace compressed_depth_image_transport

#endif
