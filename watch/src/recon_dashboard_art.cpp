#include <Arduino.h>
#include <lvgl.h>
#include <array>
#include "recon_dashboard_art.h"

namespace {

constexpr size_t RECON_ART_W = 302;
constexpr size_t RECON_ART_H = 245;
constexpr size_t RECON_ART_PIXELS = RECON_ART_W * RECON_ART_H;

constexpr uint32_t reconPalette[64] = {
    0xFFE9E6F1, 0xFFD7B2D5, 0xFF64EBFA, 0xFF3EAADC, 0xFFA86EA8, 0xFF505F7F, 0xFF0673D2, 0xFF0551AD,
    0xFFD828C5, 0xFF543B54, 0xFF413547, 0xFF36263A, 0xFF043C99, 0xFF032982, 0xFF0D2D53, 0xFF0C214C,
    0xFFFC1CEC, 0xFFFC19EB, 0xFFFA1AE3, 0xFFFA14E5, 0xFFF90EE2, 0xFFE011BF, 0xFFAB0E8C, 0xFF620D53,
    0xFF2B1530, 0xFF031760, 0xFF031541, 0xFF0B152D, 0xFF040B2A, 0xFF14131C, 0xFF0B0D19, 0xFF070A16,
    0xFF090710, 0xFF010714, 0xFFC00295, 0xFF82025F, 0xFF61024B, 0xFF520135, 0xFF3E012E, 0xFF2A0128,
    0xFF280118, 0xFF190112, 0xFF0A021C, 0xFF0B020F, 0xFF0F010B, 0xFF05030B, 0xFF0A0107, 0xFF040207,
    0xFF070104, 0xFF040103, 0xFF030103, 0xFF020203, 0xFF020002, 0xFF000316, 0xFF00020A, 0xFF000205,
    0xFF000104, 0xFF000004, 0xFF010101, 0xFF010001, 0xFF000001, 0xFF010100, 0xFF010000, 0xFF000000,
};

constexpr char reconEncoded[] =
#include "recon_dashboard_art_data.inc"
;

constexpr uint8_t decodeRecon64(char c)
{
    return (c >= 'A' && c <= 'Z') ? static_cast<uint8_t>(c - 'A') :
           (c >= 'a' && c <= 'z') ? static_cast<uint8_t>(26 + c - 'a') :
           (c >= '0' && c <= '9') ? static_cast<uint8_t>(52 + c - '0') :
           (c == '+') ? 62 : 63;
}

constexpr std::array<uint32_t, RECON_ART_PIXELS> makeReconPixels()
{
    std::array<uint32_t, RECON_ART_PIXELS> out{};
    for (size_t i = 0; i < RECON_ART_PIXELS; ++i) {
        out[i] = reconPalette[decodeRecon64(reconEncoded[i])];
    }
    return out;
}

static constexpr auto recon_dashboard_art_img_map = makeReconPixels();

} // namespace

const lv_image_dsc_t recon_dashboard_art_img = {
    .header = {
        .magic = LV_IMAGE_HEADER_MAGIC,
        .cf = LV_COLOR_FORMAT_ARGB8888,
        .flags = 0,
        .w = RECON_ART_W,
        .h = RECON_ART_H,
        .stride = RECON_ART_W * 4,
        .reserved_2 = 0,
    },
    .data_size = sizeof(recon_dashboard_art_img_map),
    .data = reinterpret_cast<const uint8_t *>(recon_dashboard_art_img_map.data()),
    .reserved = nullptr,
};