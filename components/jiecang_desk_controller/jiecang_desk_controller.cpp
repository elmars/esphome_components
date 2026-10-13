#include "jiecang_desk_controller.h"
#include "esphome/core/log.h"
#include "jiecang_protocol.h"

namespace esphome {
    namespace jiecang_desk_controller {

        static const char *const TAG = "jiecang_desk_controller";

        float JiecangDeskController::byte2float(int high, int low) {
            return protocol::decode_height_cm(high, low);
        }

        void JiecangDeskController::handle_frame(const protocol::Frame &f) {
            this->last_message_time = millis();
            const uint8_t *d = f.data;

            switch (f.cmd) {
                case 0x01: {
                    if (f.len < 2) return;
                    ESP_LOGV(TAG, "height 0x%02X%02X", d[0], d[1]);
                    float new_height = byte2float(d[0], d[1]);
                    if (new_height == current_height) return;
                    current_height = new_height;
                    if (height != nullptr) height->publish_state(current_height);
                    if (number_height != nullptr) number_height->publish_state(current_height);

                    if (height_pct != nullptr && limit_max != 0)
                        height_pct->publish_state((current_height - limit_min) / (limit_max - limit_min) * 100);
                    if (number_height_pct != nullptr && limit_max != 0)
                        number_height_pct->publish_state(roundf((current_height - limit_min) / (limit_max - limit_min) * 1000)/10);
                    break;
                }

                case 0x1B:
                    if (f.len < 2) return;
                    ESP_LOGV(TAG, "target_height 0x%02X%02X = %f", d[0], d[1], byte2float(d[0], d[1]));
                    break;

                case 0x20:
                    if (f.len < 1) return;
                    ESP_LOGV(TAG, "limits 0x%02X  max %i min %i", d[0], (d[0] & 1), (d[0] >> 4));
                    if ((d[0] & 1) == 0) { // low nibble 0 -> no max limit, use physical_max
                        limit_max = physical_max;
                        if (height_max != nullptr) height_max->publish_state(limit_max);
                        if (number_height != nullptr) number_height->set_max_value(limit_max);
                    }
                    if ((d[0] >> 4) == 0) { // high nibble 0 -> no min limit, use physical_min
                        limit_min = physical_min;
                        if (height_min != nullptr) height_min->publish_state(limit_min);
                        if (number_height != nullptr) number_height->set_min_value(limit_min);
                    }
                    break;

                case 0x07:
                    if (f.len < 4) return;
                    ESP_LOGV(TAG, "physical limits 0x%02X%02X 0x%02X%02X", d[0], d[1], d[2], d[3]);
                    physical_max = byte2float(d[0], d[1]);
                    physical_min = byte2float(d[2], d[3]);
                    break;

                case 0x21:
                    if (f.len < 2) return;
                    ESP_LOGV(TAG, "height_max 0x%02X%02X", d[0], d[1]);
                    limit_max = byte2float(d[0], d[1]);
                    if (height_max != nullptr) height_max->publish_state(limit_max);
                    if (number_height != nullptr) number_height->set_max_value(limit_max);
                    break;

                case 0x22:
                    if (f.len < 2) return;
                    ESP_LOGV(TAG, "height_min 0x%02X%02X", d[0], d[1]);
                    limit_min = byte2float(d[0], d[1]);
                    if (height_min != nullptr) height_min->publish_state(limit_min);
                    if (number_height != nullptr) number_height->set_min_value(limit_min);
                    break;

                case 0x25: case 0x26: case 0x27: case 0x28: {
                    if (f.len < 2) return;
                    Sensor *positions[] = {position1, position2, position3, position4};
                    Sensor *pos = positions[f.cmd - 0x25];
                    ESP_LOGV(TAG, "position%d 0x%02X%02X", f.cmd - 0x24, d[0], d[1]);
                    if (pos != nullptr) pos->publish_state(byte2float(d[0], d[1]));
                    break;
                }

                default:
                    ESP_LOGV(TAG, "unknown message cmd 0x%02X len %u", f.cmd, f.len);
            }
        }

        void JiecangDeskController::update() {
            while (available()) {
                if (parser_.feed(static_cast<uint8_t>(read()))) {
                    handle_frame(parser_.frame());
                }
            }
            // Retry until the box has reported its limits (e.g. boot race after a power cut).
            if (!limits_known() && millis() - last_limits_request >= 5000) {
                last_limits_request = millis();
                ESP_LOGD(TAG, "Requesting limits");
                request_physical_limits();
                request_limits();
            }
        }

        void JiecangDeskController::send_frame(const std::vector<uint8_t> &frame) {
            if (protocol::needs_wakeup(millis(), this->last_message_time)) {
                ESP_LOGV(TAG, "Sending wake-up bytes.");
                write_array(protocol::wakeup_bytes());
            }
            write_array(frame);
        }

        void JiecangDeskController::send_simple_command(unsigned char cmd) {
            send_frame(protocol::build_command(cmd));
        }

        void JiecangDeskController::add_button(button::Button *btn, int action) {
            btn->add_on_press_callback([this, action]() { this->button_press_action(action); });
        }

        void JiecangDeskController::step_up() {
            send_simple_command(0x01);
        }

        void JiecangDeskController::step_down() {
            send_simple_command(0x02);
        }

        void JiecangDeskController::stop() {
            send_simple_command(0x2B);
        }

        void JiecangDeskController::goto_position(int pos) {
            switch (pos) {
                case 1:
                    send_simple_command(0x05);
                    break;
                case 2:
                    send_simple_command(0x06);
                    break;
                case 3:
                    send_simple_command(0x27);
                    break;
                case 4:
                    send_simple_command(0x28);
                    break;
            }
        }

        void JiecangDeskController::save_position(int pos) {
            switch (pos) {
                case 1:
                    send_simple_command(0x03);
                    break;
                case 2:
                    send_simple_command(0x04);
                    break;
                case 3:
                    send_simple_command(0x25);
                    break;
                case 4:
                    send_simple_command(0x26);
                    break;
            }
        }

        void JiecangDeskController::goto_height(float height) {
            send_frame(protocol::build_goto_height(height));
        }

        void JiecangDeskController::move_up() {
            if (!limits_known()) {
                ESP_LOGW(TAG, "Limits unknown, ignoring move_up");
                return;
            }
            goto_height(limit_max);
        }

        void JiecangDeskController::move_down() {
            if (!limits_known()) {
                ESP_LOGW(TAG, "Limits unknown, ignoring move_down");
                return;
            }
            goto_height(limit_min);
        }

        void JiecangDeskController::request_physical_limits() {
            send_simple_command(0x0C);
        }

        void JiecangDeskController::request_limits() {
            send_simple_command(0x20);
        }

        void JiecangDeskController::request_settings() {
            send_simple_command(0x07);
        }

        // Returns true (and disarms) if "save position" was pressed within the last 10 s.
        bool JiecangDeskController::take_save_position_mode() {
            bool armed = save_position_mode && millis() - save_position_mode_since < 10000;
            save_position_mode = false;
            return armed;
        }

        void JiecangDeskController::button_press_action(int action) {
            ESP_LOGV("JiecangDeskController", "button_press_action %i", action);
            switch (action) {
                case BUTTON_STEP_UP:
                    step_up();
                    break;
                case BUTTON_STEP_DOWN:
                    step_down();
                    break;
                case BUTTON_STOP:
                    stop();
                    save_position_mode = false;
                    break;
                case BUTTON_MOVE_UP:
                    move_up();
                    break;
                case BUTTON_MOVE_DOWN:
                    move_down();
                    break;
                case BUTTON_POSITION1:
                    if (take_save_position_mode()) {
                        save_position(1);
                        return;
                    }
                    goto_position(1);
                    break;
                case BUTTON_POSITION2:
                    if (take_save_position_mode()) {
                        save_position(2);
                        return;
                    }
                    goto_position(2);
                    break;
                case BUTTON_POSITION3:
                    if (take_save_position_mode()) {
                        save_position(3);
                        return;
                    }
                    goto_position(3);
                    break;
                case BUTTON_POSITION4:
                    if (take_save_position_mode()) {
                        save_position(4);
                        return;
                    }
                    goto_position(4);
                    break;
                case BUTTON_SAVE_POSITION:
                    save_position_mode = true;
                    save_position_mode_since = millis();
                    break;
            }
        }

        void JiecangDeskController::add_number(JiecangDeskNumber *number, int type) {
            switch (type) {
                case NUMBER_HEIGHT:
                    number_height = number;
                    break;
                case NUMBER_HEIGHT_PCT:
                    number_height_pct = number;
                    number_height_pct->set_min_value(0);
                    number_height_pct->set_max_value(100);
                    break;
                default:
                    return;
            }
            number->set_type(type);
            number->set_parent(this);
        }

        void JiecangDeskController::number_control(int type, float value) {
            ESP_LOGV("JiecangDeskController", "number_control %i", type);
            switch (type) {
                case NUMBER_HEIGHT:
                    goto_height(value);
                    break;
                case NUMBER_HEIGHT_PCT:
                    if (limit_max != 0) goto_height((limit_max - limit_min) * value / 100 + limit_min);
                    break;
            }
        }

        void JiecangDeskButton::press_action() {}

    } //namespace jiecang_desk_controller
} //namespace esphome

