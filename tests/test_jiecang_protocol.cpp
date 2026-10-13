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

static std::vector<Frame> parse(const Bytes &bytes) {
  FrameParser p;
  std::vector<Frame> out;
  for (uint8_t b : bytes)
    if (p.feed(b)) out.push_back(p.frame());
  return out;
}

static Bytes height_frame(uint16_t mm) {
  const uint8_t hi = mm >> 8, lo = mm & 0xFF, flag = 0x00;
  const uint8_t data[] = {hi, lo, flag};
  return {0xF2, 0xF2, 0x01, 0x03, hi, lo, flag, checksum(0x01, 0x03, data), 0x7E};
}

static void test_parser_doc_example() {
  // Doku S. 7: F2 F2 01 02 03 E8 EE 7E
  auto f = parse({0xF2, 0xF2, 0x01, 0x02, 0x03, 0xE8, 0xEE, 0x7E});
  assert(f.size() == 1);
  assert(f[0].cmd == 0x01 && f[0].len == 2 && f[0].data[0] == 0x03 && f[0].data[1] == 0xE8);
}

static void test_parser_0x7e_in_payload() {
  // Befund 1: 638 / 894 / 1150 mm haben 0x7E als Low-Byte
  for (uint16_t mm : {638, 894, 1150}) {
    auto f = parse(height_frame(mm));
    assert(f.size() == 1);
    assert(decode_height_cm(f[0].data[0], f[0].data[1]) == mm / 10.0f);
  }
}

static void test_parser_checksum_is_0x7e() {
  // 887 mm = 03 77: 01+03+03+77+00 = 7E
  auto f = parse(height_frame(887));
  assert(f.size() == 1);
  assert(decode_height_cm(f[0].data[0], f[0].data[1]) == 88.7f);
}

static void test_parser_rejects_bad_checksum_and_resyncs() {
  Bytes bytes = {0xF2, 0xF2, 0x01, 0x03, 0x03, 0xE8, 0x00, 0x00, 0x7E};  // falsche Summe
  Bytes good = height_frame(1000);
  bytes.insert(bytes.end(), good.begin(), good.end());
  auto f = parse(bytes);
  assert(f.size() == 1);
  assert(decode_height_cm(f[0].data[0], f[0].data[1]) == 100.0f);
}

static void test_parser_garbage_and_extra_start() {
  Bytes bytes = {0x00, 0xFF, 0x12, 0xF2, 0x7E, 0xF2, 0xF2, 0xF2, 0x01, 0x02, 0x03, 0xE8, 0xEE, 0x7E};
  auto f = parse(bytes);
  assert(f.size() == 1 && f[0].cmd == 0x01);
}

static void test_parser_oversized_length_dropped() {
  Bytes bytes = {0xF2, 0xF2, 0x01, 0x09, 0x01, 0x02, 0x7E};
  Bytes good = height_frame(1000);
  bytes.insert(bytes.end(), good.begin(), good.end());
  auto f = parse(bytes);
  assert(f.size() == 1 && f[0].len == 3);
}

static void test_parser_truncated_frame_no_bogus_output() {
  // Abgeschnittener Frame, danach zwei gültige. Höchstens der erste gültige darf verloren gehen,
  // ein falscher Frame darf nie herauskommen.
  Bytes bytes = {0xF2, 0xF2, 0x01, 0x03, 0x03};
  for (int i = 0; i < 2; i++) {
    Bytes good = height_frame(1000);
    bytes.insert(bytes.end(), good.begin(), good.end());
  }
  auto f = parse(bytes);
  assert(f.size() >= 1);
  for (auto &fr : f) assert(decode_height_cm(fr.data[0], fr.data[1]) == 100.0f);
}

static void test_parser_zero_length_frame() {
  auto f = parse({0xF2, 0xF2, 0x04, 0x00, 0x04, 0x7E});  // RESET-Info
  assert(f.size() == 1 && f[0].cmd == 0x04 && f[0].len == 0);
}

static void test_needs_wakeup() {
  assert(needs_wakeup(5000, 0));
  assert(!needs_wakeup(4999, 0));
  assert(!needs_wakeup(0x00000100, 0xFFFFF000));  // millis() overflow: 4352 ms elapsed
  assert((wakeup_bytes() == Bytes{0, 0, 0, 0, 0, 0}));
}

int main() {
  test_build_command();
  test_build_goto_height();
  test_decode_height();
  test_parser_doc_example();
  test_parser_0x7e_in_payload();
  test_parser_checksum_is_0x7e();
  test_parser_rejects_bad_checksum_and_resyncs();
  test_parser_garbage_and_extra_start();
  test_parser_oversized_length_dropped();
  test_parser_truncated_frame_no_bogus_output();
  test_parser_zero_length_frame();
  test_needs_wakeup();
  std::puts("all protocol tests passed");
  return 0;
}
