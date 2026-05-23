#include "t6615.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace t6615 {

static const char *const TAG = "t6615";

static const uint32_t T6615_TIMEOUT = 1000;
static const uint8_t T6615_MAGIC = 0xFF;
static const uint8_t T6615_ADDR_HOST = 0xFA;
static const uint8_t T6615_ADDR_SENSOR = 0xFE;
static const uint8_t T6615_COMMAND_GET_PPM[] = {0x02, 0x03};
static const uint8_t T6615_COMMAND_GET_SERIAL[] = {0x02, 0x01};
static const uint8_t T6615_COMMAND_GET_VERSION[] = {0x02, 0x0D};
static const uint8_t T6615_COMMAND_GET_ELEVATION[] = {0x02, 0x0F};
static const uint8_t T6615_COMMAND_GET_STATUS[] = {0xB6};
static const uint8_t T6615_COMMAND_GET_ABC[] = {0xB7, 0x00};
static const uint8_t T6615_COMMAND_ENABLE_ABC[] = {0xB7, 0x01};
static const uint8_t T6615_COMMAND_DISABLE_ABC[] = {0xB7, 0x02};
static const uint8_t T6615_COMMAND_SET_ELEVATION[] = {0x03, 0x0F};

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
  if (this->available() < 4) {
    if (this->command_ != T6615Command::NONE && millis() - this->command_time_ > T6615_TIMEOUT) {
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

  uint8_t response_buffer[4 + 15];
  memset(response_buffer, '\0', sizeof(response_buffer));

  /* by the time we get here, we know we have at least four bytes in the buffer */
  this->read_array(response_buffer, 4);

  // Read header
  if (response_buffer[0] != T6615_MAGIC || response_buffer[1] != T6615_ADDR_HOST) {
    ESP_LOGW(TAG, "Got bad data from T6615! Magic was %02X and address was %02X", response_buffer[0],
             response_buffer[1]);
    /* make sure the buffer is empty */
    while (this->available())
      this->read();
    this->command_ = T6615Command::NONE;
    this->command_time_ = 0;
    this->status_set_warning();
    return;
  }

  this->status_clear_warning();

  // response_buffer[2] is the payload length; response_buffer[3] is the first payload byte.
  const uint8_t payload_len = response_buffer[2];

  switch (this->command_) {
    case T6615Command::GET_PPM: {
      /* GET_PPM reply payload is 2 bytes; we already have the first in response_buffer[3] */
      this->read_array(response_buffer + 4, 1);
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
      this->read_array(response_buffer + 4, 1);
      const uint16_t elevation = encode_uint16(response_buffer[3], response_buffer[4]);
      ESP_LOGD(TAG, "T6615 Received elevation=%u ft", elevation);
      if (this->elevation_sensor_ != nullptr)
        this->elevation_sensor_->publish_state(elevation);
      break;
    }
    case T6615Command::GET_SERIAL: {
      /* GET_SERIAL payload is up to 15 bytes; read remaining bytes */
      const uint8_t remaining = (payload_len > 1 && payload_len <= 15) ? (payload_len - 1) : 14;
      this->read_array(response_buffer + 4, remaining);
      response_buffer[4 + remaining] = '\0';
      ESP_LOGD(TAG, "T6615 Received serial=%s", response_buffer + 3);
      break;
    }
    case T6615Command::GET_VERSION: {
      const uint8_t remaining = (payload_len > 1 && payload_len <= 15) ? (payload_len - 1) : 14;
      this->read_array(response_buffer + 4, remaining);
      response_buffer[4 + remaining] = '\0';
      ESP_LOGD(TAG, "T6615 Received version=%s", response_buffer + 3);
      break;
    }
    default:
      break;
  }
  /* drain anything unexpected */
  while (this->available())
    this->read();
  this->command_time_ = 0;
  this->command_ = T6615Command::NONE;
}

void T6615Component::update() {
  // Don't start a new command if one is still in flight.
  if (this->command_ != T6615Command::NONE && millis() - this->command_time_ < T6615_TIMEOUT) {
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
