#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace t6615 {

enum class T6615Command : uint8_t {
  NONE = 0,
  GET_PPM,
  GET_SERIAL,
  GET_VERSION,
  GET_ELEVATION,
  GET_STATUS,
  SET_ELEVATION,
  SET_SGPT_PPM,
  CALIBRATE,
};

// Status byte bits returned by GET_STATUS (0xB6) per the Telaire T6615 datasheet.
// A non-zero bit indicates the named condition is currently active.
enum T6615StatusBit : uint8_t {
  T6615_STATUS_ERROR = 0x01,         // sensor reports a fault condition
  T6615_STATUS_WARMUP = 0x02,        // sensor still warming up (CO2 reads are not valid)
  T6615_STATUS_CALIBRATION = 0x04,   // calibration in progress
  T6615_STATUS_IDLE = 0x08,          // sensor idle (no active measurement)
  T6615_STATUS_SELF_TEST = 0x10,     // self-test pending / in progress
  T6615_STATUS_SERVICE_MODE = 0x40,  // service mode active
};

class T6615Component : public PollingComponent, public uart::UARTDevice {
 public:
  void loop() override;
  void update() override;
  void dump_config() override;

  // Trigger a single-point calibration. The sensor will treat the currently sampled gas as having
  // the provided ppm value (typical use: place sensor in fresh outdoor air and call with 400).
  void calibrate(uint16_t target_ppm);

  void set_co2_sensor(sensor::Sensor *co2_sensor) { this->co2_sensor_ = co2_sensor; }
  void set_status_sensor(sensor::Sensor *s) { this->status_sensor_ = s; }
  void set_error_sensor(sensor::Sensor *s) { this->error_sensor_ = s; }
  void set_warmup_sensor(sensor::Sensor *s) { this->warmup_sensor_ = s; }
  void set_calibration_sensor(sensor::Sensor *s) { this->calibration_sensor_ = s; }
  void set_idle_sensor(sensor::Sensor *s) { this->idle_sensor_ = s; }
  void set_self_test_sensor(sensor::Sensor *s) { this->self_test_sensor_ = s; }
  void set_service_mode_sensor(sensor::Sensor *s) { this->service_mode_sensor_ = s; }
  void set_elevation_sensor(sensor::Sensor *s) { this->elevation_sensor_ = s; }

 protected:
  void send_ppm_command_();
  void send_status_command_();
  void send_elevation_command_();
  void send_serial_command_();
  void send_version_command_();
  void send_set_sgpt_ppm_command_(uint16_t target_ppm);
  void send_sgpt_calibrate_command_();

  bool command_in_flight_() const;

  void publish_status_(uint8_t status);

  T6615Command command_ = T6615Command::NONE;
  uint32_t command_time_ = 0;
  uint8_t query_index_ = 0;

  sensor::Sensor *co2_sensor_{nullptr};
  sensor::Sensor *status_sensor_{nullptr};
  sensor::Sensor *error_sensor_{nullptr};
  sensor::Sensor *warmup_sensor_{nullptr};
  sensor::Sensor *calibration_sensor_{nullptr};
  sensor::Sensor *idle_sensor_{nullptr};
  sensor::Sensor *self_test_sensor_{nullptr};
  sensor::Sensor *service_mode_sensor_{nullptr};
  sensor::Sensor *elevation_sensor_{nullptr};
};

template<typename... Ts> class T6615CalibrateAction : public Action<Ts...>, public Parented<T6615Component> {
 public:
  TEMPLATABLE_VALUE(uint16_t, target_ppm)

  void play(const Ts &...x) override { this->parent_->calibrate(this->target_ppm_.value(x...)); }
};

}  // namespace t6615
}  // namespace esphome
