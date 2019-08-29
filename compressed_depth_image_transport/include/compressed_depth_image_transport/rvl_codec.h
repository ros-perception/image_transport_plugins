// This file contains a modified copy of the code published by Andrew D. Wilson
// in "Fast Lossless Depth Image Compression" in the proceedings of SIGCHI 2017.
// The original code is licensed under the MIT License.

class RvlCodec {
  public:
  int CompressRVL(const short* input, char* output, int numPixels);
  void DecompressRVL(const char* input, short* output, int numPixels);
  private:
    void EncodeVLE(int value);
    int DecodeVLE();
    int *buffer, *pBuffer, word, nibblesWritten;
};
