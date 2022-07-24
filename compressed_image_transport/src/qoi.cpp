/*
From https://github.com/ShadowMitia/libqoi
By Dimitri Belopopsky
MIT License
*/

#include "compressed_image_transport/qoi.hpp"

#include <iostream>

namespace qoi {
namespace utils {
std::vector<unsigned char> read_binary(std::string const& path) {
    std::ifstream file(path, std::fstream::ate | std::fstream::binary);
    const auto size = file.tellg();
    file.seekg(0);
    std::vector<unsigned char> output;
    output.resize(static_cast<std::size_t>(size));
    if (not file.read(reinterpret_cast<char*>(output.data()), size)) { // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        std::cerr << "Couldn't read file : " << path << '\n';
        return {};
    }
    return output;
}

void write_binary(const std::string& path, std::vector<unsigned char> const& bytes) {
    std::ofstream file(path, std::ios::out | std::ios::binary);
    const auto size = static_cast<std::streamsize>(bytes.size());
    if (not file.write(reinterpret_cast<const char*>(bytes.data()), size)) { // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        std::cerr << "Couldn't write file : " << path << '\n';
    }
}

} // namespace utils
struct RGBA {
    unsigned char r{0};
    unsigned char g{0};
    unsigned char b{0};
    unsigned char a{0};

    friend constexpr bool operator==(RGBA const& a, RGBA const& b) { return a.r == b.r and a.g == b.g and a.b == b.b and a.a == b.a; };
};

std::uint32_t read_4_be_bytes(std::vector<unsigned char> const& bytes, std::size_t position) {
    const auto res = 0x0U // peekaboo
                     | std::uint32_t(bytes[position + 0] << 24U) | std::uint32_t(bytes[position + 1] << 16U) | std::uint32_t(bytes[position + 2] << 8U) |
                     std::uint32_t(bytes[position + 3] << 0U);
    return res;
};

constexpr std::uint32_t pixel_hash(RGBA const& pix) noexcept { return (pix.r * 3 + pix.g * 5 + pix.b * 7 + pix.a * 11); };

bool is_valid(const std::vector<unsigned char>& bytes) noexcept {
    if (bytes.size() < qoi::HEADER_SIZE) {
        return false;
    }

    if (bytes[0] != QOI_MAGIC[0] and bytes[1] != QOI_MAGIC[1] and bytes[2] != QOI_MAGIC[3]) {
        return false;
    }

    return true;
}

header get_header(std::vector<unsigned char> const& image_to_decode) {
    header image_header{};
    image_header.width = read_4_be_bytes(image_to_decode, 4);
    image_header.height = read_4_be_bytes(image_to_decode, 8);
    image_header.channels = image_to_decode[12];
    image_header.colorspace = image_to_decode[13];

    // fmt::print("Header\n");
    // fmt::print("Magic {:c}{:c}{:c}{:c}\n", image_to_decode[0], image_to_decode[1], image_to_decode[2], image_to_decode[3]);
    // fmt::print("Width {}\n", image_header.width);
    // fmt::print("Height {}\n", image_header.height);
    // fmt::print("Channels {}\n", image_header.channels);
    // fmt::print("Colorspace {}\n", image_header.colorspace);

    return image_header;
}

std::vector<unsigned char> decode(std::vector<unsigned char> const& image_to_decode) {

    const auto image_header = get_header(image_to_decode);
    const auto size = static_cast<unsigned long>(image_header.width * image_header.height) * static_cast<unsigned long>(image_header.channels);

    std::vector<unsigned char> pixels;
    pixels.resize(size);

    std::array<qoi::RGBA, 64> previous_pixels;
    qoi::RGBA previous_pixel{0, 0, 0, 255};

    std::size_t index = qoi::HEADER_SIZE;

    for (std::size_t pixel_index = 0; pixel_index < size;) {
        if (index < (image_to_decode.size() - qoi::END_MARKER_LENGTH)) {
            const auto tag = image_to_decode[index++];

            if (tag == qoi::OP_RGB) {
                const auto r = image_to_decode[index++];
                const auto g = image_to_decode[index++];
                const auto b = image_to_decode[index++];
                previous_pixel = qoi::RGBA{r, g, b, previous_pixel.a};
            } else if (tag == qoi::OP_RGBA) {
                const auto r = image_to_decode[index++];
                const auto g = image_to_decode[index++];
                const auto b = image_to_decode[index++];
                const auto a = image_to_decode[index++];
                previous_pixel = qoi::RGBA{r, g, b, a};
            } else {
                constexpr unsigned char MASK = 0b11000000U;

                if ((tag & MASK) == qoi::OP_INDEX) {
                    const auto previous_index = (tag & 0b00111111U);
                    previous_pixel = previous_pixels[previous_index];
                } else if ((tag & MASK) == qoi::OP_DIFF) {
                    previous_pixel.r += static_cast<unsigned char>((((tag & 0b110000U) >> 4U) & 0b11U) - 2);
                    previous_pixel.g += static_cast<unsigned char>((((tag & 0b001100U) >> 2U) & 0b11U) - 2);
                    previous_pixel.b += static_cast<unsigned char>((((tag & 0b000011U) >> 0U) & 0b11U) - 2);
                } else if ((tag & MASK) == qoi::OP_LUMA) {
                    const auto data = image_to_decode[index++];
                    const auto vg = static_cast<unsigned char>((tag & 0x3FU) - 32);
                    previous_pixel.r += (vg - 8 + static_cast<unsigned char>((data >> 4U) & 0xFU));
                    previous_pixel.g += vg;
                    previous_pixel.b += (vg - 8 + static_cast<unsigned char>((data >> 0U) & 0xFU));
                } else if ((tag & MASK) == qoi::OP_RUN) {
                    const auto run = tag & 0b00111111U;
                    for (std::size_t j = 0; j < run; j++) {
                        if (image_header.channels == 4) {
                            pixels[pixel_index++] = previous_pixel.r;
                            pixels[pixel_index++] = previous_pixel.g;
                            pixels[pixel_index++] = previous_pixel.b;
                            pixels[pixel_index++] = previous_pixel.a;
                        } else {
                            pixels[pixel_index++] = previous_pixel.r;
                            pixels[pixel_index++] = previous_pixel.g;
                            pixels[pixel_index++] = previous_pixel.b;
                        }
                    }
                }
            }
            const auto pixel_hash_index = (pixel_hash(previous_pixel) % 64);
            previous_pixels[static_cast<std::size_t>(pixel_hash_index)] = previous_pixel;
        }

        if (image_header.channels == 4) {
            pixels[pixel_index++] = previous_pixel.r;
            pixels[pixel_index++] = previous_pixel.g;
            pixels[pixel_index++] = previous_pixel.b;
            pixels[pixel_index++] = previous_pixel.a;
        } else {
            pixels[pixel_index++] = previous_pixel.r;
            pixels[pixel_index++] = previous_pixel.g;
            pixels[pixel_index++] = previous_pixel.b;
        }
    }

    return pixels;
}

std::vector<unsigned char> encode(std::vector<unsigned char> const& orig_pixels, std::uint32_t width, std::uint32_t height, unsigned char channels) {

    // TODO: extract as argument
    qoi::header head{};
    head.width = width;
    head.height = height;
    head.channels = channels;
    head.colorspace = qoi::SRGB;

    const auto size = head.width * head.height * head.channels;
    const auto qoi_size = head.width * head.height * (head.channels + 1) + qoi::HEADER_SIZE + qoi::END_MARKER_LENGTH;
    std::vector<unsigned char> encoded_pixels;
    encoded_pixels.resize(qoi_size, 0x0);

    std::array<qoi::RGBA, 64> previous_pixels;
    qoi::RGBA previous_pixel{0, 0, 0, 255};
    qoi::RGBA pixel{0, 0, 0, 255};

    encoded_pixels[0] = qoi::QOI_MAGIC[0];
    encoded_pixels[1] = qoi::QOI_MAGIC[1];
    encoded_pixels[2] = qoi::QOI_MAGIC[2];
    encoded_pixels[3] = qoi::QOI_MAGIC[3];
    encoded_pixels[4] = static_cast<unsigned char>((head.width & 0xFF000000U) >> 24U);
    encoded_pixels[5] = static_cast<unsigned char>((head.width & 0x00FF0000U) >> 16U);
    encoded_pixels[6] = static_cast<unsigned char>((head.width & 0x0000FF00U) >> 8U);
    encoded_pixels[7] = static_cast<unsigned char>((head.width & 0x000000FFU) >> 0U);
    encoded_pixels[8] = static_cast<unsigned char>((head.height & 0xFF000000U) >> 24U);
    encoded_pixels[9] = static_cast<unsigned char>((head.height & 0x00FF0000U) >> 16U);
    encoded_pixels[10] = static_cast<unsigned char>((head.height & 0x0000FF00U) >> 8U);
    encoded_pixels[11] = static_cast<unsigned char>((head.height & 0x000000FFU) >> 0U);
    encoded_pixels[12] = head.channels;
    encoded_pixels[13] = head.colorspace;

    std::size_t index = qoi::HEADER_SIZE;

    auto run = 0U;

    for (std::size_t pixel_index = 0; pixel_index < size; pixel_index += head.channels) {
        if (head.channels == 4) {
            pixel = qoi::RGBA{orig_pixels[pixel_index + 0], orig_pixels[pixel_index + 1], orig_pixels[pixel_index + 2], orig_pixels[pixel_index + 3]};
        } else {
            pixel.r = orig_pixels[pixel_index + 0];
            pixel.g = orig_pixels[pixel_index + 1];
            pixel.b = orig_pixels[pixel_index + 2];
        }
        if (previous_pixel == pixel) {
            run++;
            if (run == 62 || pixel_index == (size - head.channels)) {
                encoded_pixels[index++] = static_cast<unsigned char>(qoi::OP_RUN | (run - 1));
                run = 0;
            }

        } else {

            if (run > 0) {
                encoded_pixels[index++] = static_cast<unsigned char>(qoi::OP_RUN | (run - 1));
                run = 0;
            }

            const auto index_pos = static_cast<unsigned char>(qoi::pixel_hash(pixel) % 64);

            if (previous_pixels[index_pos] == pixel) {
                encoded_pixels[index++] = qoi::OP_INDEX | index_pos;
            } else {
                previous_pixels[index_pos] = pixel;

                if (pixel.a == previous_pixel.a) {
                    const auto vr = static_cast<signed char>(pixel.r - previous_pixel.r);
                    const auto vg = static_cast<signed char>(pixel.g - previous_pixel.g);
                    const auto vb = static_cast<signed char>(pixel.b - previous_pixel.b);

                    const auto vg_r = vr - vg;
                    const auto vg_b = vb - vg;

                    if (vr > -3 and vr < 2 and vg > -3 and vg < 2 and vb > -3 and vb < 2) {
                        encoded_pixels[index++] = static_cast<unsigned char>(qoi::OP_DIFF | (vr + 2) << 4U | (vg + 2) << 2U | (vb + 2));
                    } else if (vg_r > -9 and vg_r < 8 and vg > -33 and vg < 32 and vg_b > -9 and vg_b < 8) {
                        encoded_pixels[index++] = static_cast<unsigned char>(qoi::OP_LUMA | static_cast<unsigned char>(vg + 32));
                        encoded_pixels[index++] = static_cast<unsigned char>(static_cast<unsigned char>(vg_r + 8) << 4U | static_cast<unsigned char>(vg_b + 8));
                    } else {
                        encoded_pixels[index++] = qoi::OP_RGB;
                        encoded_pixels[index++] = pixel.r;
                        encoded_pixels[index++] = pixel.g;
                        encoded_pixels[index++] = pixel.b;
                    }
                } else {
                    encoded_pixels[index++] = qoi::OP_RGBA;
                    encoded_pixels[index++] = pixel.r;
                    encoded_pixels[index++] = pixel.g;
                    encoded_pixels[index++] = pixel.b;
                    encoded_pixels[index++] = pixel.a;
                }
            }
        }
        previous_pixel = pixel;
    }

    for (auto const& pad : padding) {
        encoded_pixels[index++] = pad;
    }
    encoded_pixels.resize(index);
    encoded_pixels.shrink_to_fit();

    return encoded_pixels;
}

} // namespace qoi
