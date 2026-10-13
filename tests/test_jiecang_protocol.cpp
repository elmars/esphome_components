// Host-Tests für jiecang_protocol.h (ohne ESPHome).
// Build: g++ -std=c++17 -Wall -Wextra -Werror -I components/jiecang_desk_controller -o test_protocol tests/test_jiecang_protocol.cpp && ./test_protocol
#include <cassert>
#include <cstdio>
#include <vector>
#include "jiecang_protocol.h"

using namespace esphome::jiecang_desk_controller::protocol;
using Bytes = std::vector<uint8_t>;

static void test_build_command() {
  // Doku: Move table up = F1 F1 01 00 01 7E
  assert((build_command(0x01) == Bytes{0xF1, 0xF1, 0x01, 0x00, 0x01, 0x7E}));
  // Doku: Lock status abfragen = F1 F1 1F 01 00 20 7E
  assert((build_command(0x1F, {0x00}) == Bytes{0xF1, 0xF1, 0x1F, 0x01, 0x00, 0x20, 0x7E}));
}

static void test_build_goto_height() {
  // Doku: 1000 mm = f1 f1 1b 2 3 e8 8 7e, 900 mm = f1 f1 1b 2 3 84 a4 7e
  assert((build_goto_height(100.0f) == Bytes{0xF1, 0xF1, 0x1B, 0x02, 0x03, 0xE8, 0x08, 0x7E}));
  assert((build_goto_height(90.0f) == Bytes{0xF1, 0xF1, 0x1B, 0x02, 0x03, 0x84, 0xA4, 0x7E}));
  // Ist-Stand (Befund 6): Nachkommastellen werden abgeschnitten, 72,5 cm -> 720 mm
  assert((build_goto_height(72.5f) == Bytes{0xF1, 0xF1, 0x1B, 0x02, 0x02, 0xD0, 0xEF, 0x7E}));
}

static void test_decode_height() {
  assert(decode_height_cm(0x03, 0xE8) == 100.0f);
  assert(decode_height_cm(0x04, 0x7E) == 115.0f);
}

int main() {
  test_build_command();
  test_build_goto_height();
  test_decode_height();
  std::puts("all protocol tests passed");
  return 0;
}
