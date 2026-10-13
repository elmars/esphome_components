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

// The box ignores the first bytes after ~10 s of silence. We wake it up after 5 s.
static const uint32_t WAKEUP_AFTER_MS = 5000;

inline bool needs_wakeup(uint32_t now_ms, uint32_t last_rx_ms) {
  return now_ms - last_rx_ms >= WAKEUP_AFTER_MS;  // unsigned: survives millis() overflow
}

inline std::vector<uint8_t> wakeup_bytes() { return std::vector<uint8_t>(6, 0x00); }

static const uint8_t MAX_DATA = 8;

struct Frame {
  uint8_t cmd{0};
  uint8_t len{0};
  uint8_t data[MAX_DATA]{};
};

// Byte-wise parser for frames from the desk. Uses the length field, so 0x7E inside
// data or checksum is fine. Frames with a bad checksum, a missing end byte or
// len > MAX_DATA are dropped.
class FrameParser {
 public:
  // Returns true when a complete, valid frame is available via frame().
  bool feed(uint8_t b) {
    switch (state_) {
      case WAIT_START1:
        if (b == START_RX) state_ = WAIT_START2;
        break;
      case WAIT_START2:
        state_ = (b == START_RX) ? WAIT_CMD : WAIT_START1;
        break;
      case WAIT_CMD:
        if (b == START_RX) break;  // tolerate extra start bytes
        frame_ = Frame{};
        frame_.cmd = b;
        state_ = WAIT_LEN;
        break;
      case WAIT_LEN:
        if (b > MAX_DATA) {
          state_ = WAIT_START1;
          break;
        }
        frame_.len = b;
        pos_ = 0;
        state_ = (b == 0) ? WAIT_SUM : WAIT_DATA;
        break;
      case WAIT_DATA:
        frame_.data[pos_++] = b;
        if (pos_ == frame_.len) state_ = WAIT_SUM;
        break;
      case WAIT_SUM:
        sum_ = b;
        state_ = WAIT_END;
        break;
      case WAIT_END:
        if (b == END && sum_ == checksum(frame_.cmd, frame_.len, frame_.data)) {
          state_ = WAIT_START1;
          return true;
        }
        state_ = (b == START_RX) ? WAIT_START2 : WAIT_START1;
        break;
    }
    return false;
  }

  const Frame &frame() const { return frame_; }

 private:
  enum State : uint8_t { WAIT_START1, WAIT_START2, WAIT_CMD, WAIT_LEN, WAIT_DATA, WAIT_SUM, WAIT_END };
  State state_{WAIT_START1};
  Frame frame_{};
  uint8_t pos_{0};
  uint8_t sum_{0};
};

}  // namespace protocol
}  // namespace jiecang_desk_controller
}  // namespace esphome
