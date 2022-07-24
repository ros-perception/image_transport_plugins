/*
From https://github.com/ShadowMitia/libqoi
By Dimitri Belopopsky
MIT License
*/

#ifndef QOI_HEADER
#define QOI_HEADER

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <vector>

namespace qoi {

struct header {
    // Image width in pixels
    std::uint32_t width{};
    // Image height in pixels
    std::uint32_t height{};
    // 3 = RGB, 4 = RGBA
    std::uint8_t channels{};
    // 0 = sRGB with linear alpha
    // 1 = all channels linear
    std::uint8_t colorspace{};
};

constexpr auto SRGB = 0x0;
constexpr auto LINEAR = 0x1;

constexpr unsigned char OP_RGB = 0b11111110;
constexpr unsigned char OP_RGBA = 0b11111111;
constexpr unsigned char OP_INDEX = 0b0;
constexpr unsigned char OP_DIFF = 0b01000000;
constexpr unsigned char OP_LUMA = 0b10000000;
constexpr unsigned char OP_RUN = 0b11000000;

constexpr std::array<unsigned char, 4> QOI_MAGIC{'q', 'o', 'i', 'f'};

constexpr unsigned char END_MARKER_LENGTH = 8; // in bytes
constexpr std::array<unsigned char, 8> padding{0, 0, 0, 0, 0, 0, 0, 1};
constexpr unsigned char HEADER_SIZE = 14; // in bytes

bool is_valid(std::vector<unsigned char> const& bytes) noexcept;

header get_header(std::vector<unsigned char> const& image_to_decode);

std::vector<unsigned char> decode(std::vector<unsigned char> const& image_to_decode);

std::vector<unsigned char> encode(std::vector<unsigned char> const& orig_pixels, std::uint32_t width, std::uint32_t height, unsigned char channels);

namespace utils {
std::vector<unsigned char> read_binary(std::string const& path);
void write_binary(const std::string& path, std::vector<unsigned char> const& bytes);
} // namespace utils

#ifdef QOI_HEADER_ONLY
#include "qoi.cpp"
#endif

} // namespace qoi

#endif
