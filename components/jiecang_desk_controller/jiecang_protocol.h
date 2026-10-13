#pragma once
// Pure protocol helpers for the Jiecang control box serial interface.
// No ESPHome dependencies, so this header can be unit tested on the host.
//
// Frame format (both directions):
//   start start cmd len data[len] checksum 0x7E
//   start = 0xF1 (to the desk) / 0xF2 (from the desk)
//   checksum = (cmd + len + sum(data)) mod 256

#include <cstdint>
#include <initializer_list>
#include <vector>

namespace esphome {
namespace jiecang_desk_controller {
namespace protocol {

static const uint8_t START_TX = 0xF1;
static const uint8_t START_RX = 0xF2;
static const uint8_t END = 0x7E;

inline uint8_t checksum(uint8_t cmd, uint8_t len, const uint8_t *data) {
  uint8_t sum = cmd + len;
  for (uint8_t i = 0; i < len; i++) sum += data[i];
  return sum;
}

inline std::vector<uint8_t> build_command(uint8_t cmd, std::initializer_list<uint8_t> data = {}) {
  std::vector<uint8_t> out{START_TX, START_TX, cmd, static_cast<uint8_t>(data.size())};
  out.insert(out.end(), data.begin(), data.end());
  out.push_back(checksum(cmd, static_cast<uint8_t>(data.size()), data.begin()));
  out.push_back(END);
  return out;
}

inline std::vector<uint8_t> build_goto_height(float height_cm) {
  const uint16_t mm = static_cast<uint16_t>(static_cast<int>(height_cm) * 10);
  return build_command(0x1B, {static_cast<uint8_t>(mm >> 8), static_cast<uint8_t>(mm & 0xFF)});
}

// Heights are sent in mm; the component works in cm.
inline float decode_height_cm(uint8_t high, uint8_t low) {
  return static_cast<float>((high << 8) | low) / 10.0f;
}

}  // namespace protocol
}  // namespace jiecang_desk_controller
}  // namespace esphome
