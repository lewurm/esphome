#include "t6615.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace t6615 {

static const char *const TAG = "t6615";

static const uint32_t T6615_TIMEOUT = 1000;
// Single-point calibration triggers a measurement cycle on the sensor; the documented reply is
// just an ACK acknowledging the command was received, but allow generous headroom in case the
// sensor delays the ACK while it starts the calibration cycle.
static const uint32_t T6615_SLOW_TIMEOUT = 5000;
// At 19200 baud each byte is ~520 us, so a 15-byte payload arrives in ~8 ms once the
// header has been seen. Cap how long we are willing to spin waiting for the tail.
static const uint32_t T6615_PAYLOAD_TIMEOUT = 50;
static const uint8_t T6615_MAGIC = 0xFF;
static const uint8_t T6615_ADDR_HOST = 0xFA;
static const uint8_t T6615_ADDR_SENSOR = 0xFE;
static const uint8_t T6615_COMMAND_GET_PPM[] = {0x02, 0x03};
static const uint8_t T6615_COMMAND_GET_SERIAL[] = {0x02, 0x01};
static const uint8_t T6615_COMMAND_GET_VERSION[] = {0x02, 0x0D};
static const uint8_t T6615_COMMAND_GET_ELEVATION[] = {0x02, 0x0F};
static const uint8_t T6615_COMMAND_GET_STATUS[] = {0xB6};
static const uint8_t T6615_COMMAND_SET_ELEVATION[] = {0x03, 0x0F};
// Set single-point ppm target (CMD_SET_SGPT_PPM): 0x03 0x11 followed by a 16-bit ppm value
// (MSB first). Per the T63182-004 protocol doc this must be sent BEFORE CMD_SGPT_CALIBRATE; the
// calibrate command itself carries no payload.
static const uint8_t T6615_COMMAND_SET_SGPT_PPM[] = {0x03, 0x11};
// Single-point calibration (CMD_SGPT_CALIBRATE): bare 0x9B. The ppm target is set separately via
// T6615_COMMAND_SET_SGPT_PPM above.
static const uint8_t T6615_COMMAND_SGPT_CALIBRATE = 0x9B;

bool T6615Component::command_in_flight_() const {
  if (this->command_ == T6615Command::NONE)
    return false;
  // Pick the per-command grace window. CALIBRATE and SET_SGPT_PPM can take a few seconds to ack;
  // everything else should respond within ~1 s. Both loop() and update() consult this so the poll
  // loop will not stomp an in-flight slow command with a routine query.
  const bool slow = this->command_ == T6615Command::CALIBRATE || this->command_ == T6615Command::SET_SGPT_PPM;
  const uint32_t timeout = slow ? T6615_SLOW_TIMEOUT : T6615_TIMEOUT;
  return (millis() - this->command_time_) < timeout;
}

void T6615Component::send_ppm_command_() {
  this->command_time_ = millis();
  this->command_ = T6615Command::GET_PPM;
  this->write_byte(T6615_MAGIC);
  this->write_byte(T6615_ADDR_SENSOR);
  this->write_byte(sizeof(T6615_COMMAND_GET_PPM));
  this->write_array(T6615_COMMAND_GET_PPM, sizeof(T6615_COMMAND_GET_PPM));
}

void T6615Component::send_status_command_() {
  this->command_time_ = millis();
  this->command_ = T6615Command::GET_STATUS;
  this->write_byte(T6615_MAGIC);
  this->write_byte(T6615_ADDR_SENSOR);
  this->write_byte(sizeof(T6615_COMMAND_GET_STATUS));
  this->write_array(T6615_COMMAND_GET_STATUS, sizeof(T6615_COMMAND_GET_STATUS));
}

void T6615Component::send_elevation_command_() {
  this->command_time_ = millis();
  this->command_ = T6615Command::GET_ELEVATION;
  this->write_byte(T6615_MAGIC);
  this->write_byte(T6615_ADDR_SENSOR);
  this->write_byte(sizeof(T6615_COMMAND_GET_ELEVATION));
  this->write_array(T6615_COMMAND_GET_ELEVATION, sizeof(T6615_COMMAND_GET_ELEVATION));
}

void T6615Component::send_serial_command_() {
  this->command_time_ = millis();
  this->command_ = T6615Command::GET_SERIAL;
  this->write_byte(T6615_MAGIC);
  this->write_byte(T6615_ADDR_SENSOR);
  this->write_byte(sizeof(T6615_COMMAND_GET_SERIAL));
  this->write_array(T6615_COMMAND_GET_SERIAL, sizeof(T6615_COMMAND_GET_SERIAL));
}

void T6615Component::send_version_command_() {
  this->command_time_ = millis();
  this->command_ = T6615Command::GET_VERSION;
  this->write_byte(T6615_MAGIC);
  this->write_byte(T6615_ADDR_SENSOR);
  this->write_byte(sizeof(T6615_COMMAND_GET_VERSION));
  this->write_array(T6615_COMMAND_GET_VERSION, sizeof(T6615_COMMAND_GET_VERSION));
}

void T6615Component::send_set_sgpt_ppm_command_(uint16_t target_ppm) {
  this->command_time_ = millis();
  this->command_ = T6615Command::SET_SGPT_PPM;
  const uint8_t payload[] = {T6615_COMMAND_SET_SGPT_PPM[0], T6615_COMMAND_SET_SGPT_PPM[1],
                             static_cast<uint8_t>(target_ppm >> 8), static_cast<uint8_t>(target_ppm & 0xFF)};
  this->write_byte(T6615_MAGIC);
  this->write_byte(T6615_ADDR_SENSOR);
  this->write_byte(sizeof(payload));
  this->write_array(payload, sizeof(payload));
}

void T6615Component::send_sgpt_calibrate_command_() {
  this->command_time_ = millis();
  this->command_ = T6615Command::CALIBRATE;
  this->write_byte(T6615_MAGIC);
  this->write_byte(T6615_ADDR_SENSOR);
  this->write_byte(1);
  this->write_byte(T6615_COMMAND_SGPT_CALIBRATE);
}

void T6615Component::calibrate(uint16_t target_ppm) {
  ESP_LOGI(TAG, "Single-point calibration: setting target to %u ppm, then triggering calibrate", target_ppm);
  // Drain any pending in-flight response so the upcoming ACKs can be matched cleanly. The full
  // protocol sequence per T63182-004 is SET_SGPT_PPM (ack) -> SGPT_CALIBRATE (ack); the chain
  // to the second command is kicked off from the SET_SGPT_PPM case in loop().
  while (this->available())
    this->read();
  this->send_set_sgpt_ppm_command_(target_ppm);
}

void T6615Component::publish_status_(uint8_t status) {
  if (this->status_sensor_ != nullptr)
    this->status_sensor_->publish_state(status);
  if (this->error_sensor_ != nullptr)
    this->error_sensor_->publish_state((status & T6615_STATUS_ERROR) ? 1 : 0);
  if (this->warmup_sensor_ != nullptr)
    this->warmup_sensor_->publish_state((status & T6615_STATUS_WARMUP) ? 1 : 0);
  if (this->calibration_sensor_ != nullptr)
    this->calibration_sensor_->publish_state((status & T6615_STATUS_CALIBRATION) ? 1 : 0);
  if (this->idle_sensor_ != nullptr)
    this->idle_sensor_->publish_state((status & T6615_STATUS_IDLE) ? 1 : 0);
  if (this->self_test_sensor_ != nullptr)
    this->self_test_sensor_->publish_state((status & T6615_STATUS_SELF_TEST) ? 1 : 0);
  if (this->service_mode_sensor_ != nullptr)
    this->service_mode_sensor_->publish_state((status & T6615_STATUS_SERVICE_MODE) ? 1 : 0);
}

void T6615Component::loop() {
  // The reply framing is [MAGIC][ADDR][LEN][DATA...]. We first wait for the 3-byte header and
  // only then for `LEN` payload bytes. The CALIBRATE ack is a 3-byte header with len=0, which
  // the previous "wait for 4 bytes" logic could never satisfy.
  if (this->available() < 3) {
    if (!this->command_in_flight_() && this->command_ != T6615Command::NONE) {
      ESP_LOGW(TAG, "timeout receiving answer for command %u, clearing buffer.",
               static_cast<uint8_t>(this->command_));
      while (this->available())
        this->read();
      this->command_ = T6615Command::NONE;
      this->command_time_ = 0;
      this->status_set_warning();
    }
    return;
  }

  uint8_t response_buffer[3 + 15];
  memset(response_buffer, '\0', sizeof(response_buffer));
  this->read_array(response_buffer, 3);

  if (response_buffer[0] != T6615_MAGIC || response_buffer[1] != T6615_ADDR_HOST) {
    ESP_LOGW(TAG, "Got bad data from T6615! Magic was %02X and address was %02X", response_buffer[0],
             response_buffer[1]);
    while (this->available())
      this->read();
    this->command_ = T6615Command::NONE;
    this->command_time_ = 0;
    this->status_set_warning();
    return;
  }

  this->status_clear_warning();

  const uint8_t payload_len = response_buffer[2];

  // Wait briefly for the payload bytes if they have not already arrived. At 19200 baud the
  // longest payload we read (15 bytes) lands within ~8 ms of the header, so a 50 ms cap is
  // very loose without holding the main loop too long.
  if (payload_len > 0) {
    const uint8_t to_read = payload_len > 15 ? 15 : payload_len;
    const uint32_t wait_start = millis();
    while (this->available() < to_read) {
      if (millis() - wait_start > T6615_PAYLOAD_TIMEOUT) {
        ESP_LOGW(TAG, "T6615 incomplete payload: got %u of %u bytes", this->available(), to_read);
        while (this->available())
          this->read();
        this->command_ = T6615Command::NONE;
        this->command_time_ = 0;
        this->status_set_warning();
        return;
      }
      delay(1);
    }
    this->read_array(response_buffer + 3, to_read);
  }

  switch (this->command_) {
    case T6615Command::GET_PPM: {
      const uint16_t ppm = encode_uint16(response_buffer[3], response_buffer[4]);
      ESP_LOGD(TAG, "T6615 Received CO2=%u ppm", ppm);
      if (this->co2_sensor_ != nullptr)
        this->co2_sensor_->publish_state(ppm);
      break;
    }
    case T6615Command::GET_STATUS: {
      const uint8_t status = response_buffer[3];
      ESP_LOGD(TAG, "T6615 Received status=0x%02X (error=%u warmup=%u calibration=%u idle=%u self_test=%u service=%u)",
               status, (status & T6615_STATUS_ERROR) ? 1 : 0, (status & T6615_STATUS_WARMUP) ? 1 : 0,
               (status & T6615_STATUS_CALIBRATION) ? 1 : 0, (status & T6615_STATUS_IDLE) ? 1 : 0,
               (status & T6615_STATUS_SELF_TEST) ? 1 : 0, (status & T6615_STATUS_SERVICE_MODE) ? 1 : 0);
      this->publish_status_(status);
      break;
    }
    case T6615Command::GET_ELEVATION: {
      const uint16_t elevation = encode_uint16(response_buffer[3], response_buffer[4]);
      ESP_LOGD(TAG, "T6615 Received elevation=%u ft", elevation);
      if (this->elevation_sensor_ != nullptr)
        this->elevation_sensor_->publish_state(elevation);
      break;
    }
    case T6615Command::GET_SERIAL: {
      response_buffer[3 + payload_len] = '\0';
      ESP_LOGD(TAG, "T6615 Received serial=%s", response_buffer + 3);
      break;
    }
    case T6615Command::GET_VERSION: {
      response_buffer[3 + payload_len] = '\0';
      ESP_LOGD(TAG, "T6615 Received version=%s", response_buffer + 3);
      break;
    }
    case T6615Command::SET_SGPT_PPM: {
      // Per spec the reply is <ACK> (header-only, len=0). Once we have it, chain into the bare
      // CMD_SGPT_CALIBRATE that actually triggers the calibration cycle. Return early so the
      // tail cleanup at the bottom of loop() doesn't clobber the freshly-set command_.
      if (payload_len != 0) {
        ESP_LOGW(TAG, "T6615 SET_SGPT_PPM ack with payload len=%u, first byte=0x%02X", payload_len,
                 response_buffer[3]);
      }
      ESP_LOGD(TAG, "T6615 SET_SGPT_PPM ack received; sending SGPT_CALIBRATE");
      this->send_sgpt_calibrate_command_();
      return;
    }
    case T6615Command::CALIBRATE: {
      // The Telaire protocol acks a successful write with a header-only reply (len=0). A non-zero
      // len here would carry an error/status byte from the sensor.
      if (payload_len == 0) {
        ESP_LOGI(TAG, "T6615 calibration accepted");
      } else {
        ESP_LOGW(TAG, "T6615 calibration ack with payload len=%u, first byte=0x%02X", payload_len,
                 response_buffer[3]);
      }
      break;
    }
    default:
      break;
  }
  while (this->available())
    this->read();
  this->command_time_ = 0;
  this->command_ = T6615Command::NONE;
}

void T6615Component::update() {
  // Don't start a new command if one is still in flight. This matters most for slow commands
  // (CALIBRATE, SET_SGPT_PPM) whose response can take longer than this poll interval.
  if (this->command_in_flight_()) {
    return;
  }

  // Rotate through PPM (most frequent), STATUS, and ELEVATION. Once after boot also fetch serial
  // and version so they appear in the log for identification.
  switch (this->query_index_) {
    case 0:
      this->send_serial_command_();
      break;
    case 1:
      this->send_version_command_();
      break;
    default: {
      const uint8_t phase = (this->query_index_ - 2) % 6;
      if (phase == 4) {
        this->send_status_command_();
      } else if (phase == 5) {
        this->send_elevation_command_();
      } else {
        this->send_ppm_command_();
      }
      break;
    }
  }
  if (this->query_index_ < 255)
    this->query_index_++;
}

void T6615Component::dump_config() {
  ESP_LOGCONFIG(TAG, "T6615:");
  LOG_SENSOR("  ", "CO2", this->co2_sensor_);
  LOG_SENSOR("  ", "Status", this->status_sensor_);
  LOG_SENSOR("  ", "Error", this->error_sensor_);
  LOG_SENSOR("  ", "Warmup", this->warmup_sensor_);
  LOG_SENSOR("  ", "Calibration", this->calibration_sensor_);
  LOG_SENSOR("  ", "Idle", this->idle_sensor_);
  LOG_SENSOR("  ", "Self-Test", this->self_test_sensor_);
  LOG_SENSOR("  ", "Service Mode", this->service_mode_sensor_);
  LOG_SENSOR("  ", "Elevation", this->elevation_sensor_);
  this->check_uart_settings(19200);
}

}  // namespace t6615
}  // namespace esphome
