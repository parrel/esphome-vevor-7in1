#pragma once
// Vevor 7-in-1 Weather Station decoder for ESPHome.
//
// Input is either CC1101 packet mode (the radio delivers the 21-byte frame) or
// remote_receiver (bits rebuilt from mark/space timings). Each path is only
// compiled in when the configuration uses it.

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"

#ifdef USE_VEVOR_DECODER_CC1101
#include "esphome/components/cc1101/cc1101.h"
#endif
#ifdef USE_VEVOR_DECODER_REMOTE_RECEIVER
#include "esphome/components/remote_base/remote_base.h"
#endif

#include <vector>

namespace esphome {
namespace vevor_decoder {

// 0xAA header, 17 data bytes, counter, checksum, counter + 1.
static const int FRAME_BYTES = 21;

// remote_receiver: room for a whole ~85 ms burst (~940 bits, two frame copies).
static const int MAX_BITS = 1024;

// Sentinel for "accept any station id".
static const int32_t SENSOR_ID_ANY = -1;

// clang-format off
class VevorDecoder : public Component
#ifdef USE_VEVOR_DECODER_CC1101
                   , public cc1101::CC1101Listener
#endif
#ifdef USE_VEVOR_DECODER_REMOTE_RECEIVER
                   , public remote_base::RemoteReceiverDumperBase
#endif
{
  // clang-format on
 public:
  void set_temperature_sensor(sensor::Sensor *s) { this->temperature_ = s; }
  void set_humidity_sensor(sensor::Sensor *s) { this->humidity_ = s; }
  void set_wind_speed_sensor(sensor::Sensor *s) { this->wind_speed_ = s; }
  void set_wind_gust_sensor(sensor::Sensor *s) { this->wind_gust_ = s; }
  void set_wind_direction_sensor(sensor::Sensor *s) { this->wind_dir_ = s; }
  void set_rain_sensor(sensor::Sensor *s) { this->rain_ = s; }
  void set_uv_index_sensor(sensor::Sensor *s) { this->uv_ = s; }
  void set_illuminance_sensor(sensor::Sensor *s) { this->light_ = s; }
  void set_battery_low_binary_sensor(binary_sensor::BinarySensor *s) { this->battery_ = s; }
  // Packet mode only.
  void set_rssi_sensor(sensor::Sensor *s) { this->rssi_ = s; }
  void set_frequency_offset_sensor(sensor::Sensor *s) { this->freq_offset_ = s; }
  void set_link_quality_sensor(sensor::Sensor *s) { this->lqi_ = s; }

  // Only publish frames from this station id. SENSOR_ID_ANY accepts all.
  void set_sensor_id(int32_t sensor_id) { this->sensor_id_ = sensor_id; }
  // NRZ bit period of the on-air signal, in microseconds.
  void set_bit_period(uint32_t bit_period_us) { this->bit_period_ = bit_period_us; }
  // Hold the previous rain total when a frame reports a lower one.
  void set_rain_hold(bool rain_hold) { this->rain_hold_ = rain_hold; }
  // Drop illuminance readings that contradict the UV index in the same frame.
  void set_illuminance_filter(bool enabled) { this->illuminance_filter_ = enabled; }

  // Frames come from CC1101 packet mode rather than remote_receiver.
  void set_packet_input(bool packet_input) { this->packet_input_ = packet_input; }

  float get_setup_priority() const override { return setup_priority::DATA; }
  void dump_config() override;

#ifdef USE_VEVOR_DECODER_CC1101
  void on_packet(const std::vector<uint8_t> &packet, float freq_offset, float rssi, uint8_t lqi) override;
#endif
#ifdef USE_VEVOR_DECODER_REMOTE_RECEIVER
  bool dump(remote_base::RemoteReceiveData src) override;
#endif

 protected:
  enum class FrameResult { PUBLISHED, DUPLICATE, FILTERED };

  // Header, checksum and counter.
  static bool frame_valid_(const uint8_t *b);
  // Station filter and duplicate suppression, then publish.
  FrameResult handle_frame_(const uint8_t *frame);
  void publish_frame_(const uint8_t *b);

#ifdef USE_VEVOR_DECODER_REMOTE_RECEIVER
  // Turns the raw mark/space timings into NRZ bits in bits_.
  // Returns the number of bits produced.
  int timings_to_bits_(const std::vector<int32_t> &raw);
  // Reads 21 bytes starting at bit offset, applying inversion, and verifies
  // them with frame_valid_(). Returns true if the frame is sound.
  bool extract_frame_(int bit_offset, uint8_t inv, uint8_t *out);
#endif

  sensor::Sensor *temperature_{nullptr};
  sensor::Sensor *humidity_{nullptr};
  sensor::Sensor *wind_speed_{nullptr};
  sensor::Sensor *wind_gust_{nullptr};
  sensor::Sensor *wind_dir_{nullptr};
  sensor::Sensor *rain_{nullptr};
  sensor::Sensor *uv_{nullptr};
  sensor::Sensor *light_{nullptr};
  binary_sensor::BinarySensor *battery_{nullptr};
  sensor::Sensor *rssi_{nullptr};
  sensor::Sensor *freq_offset_{nullptr};
  sensor::Sensor *lqi_{nullptr};

  int32_t sensor_id_{SENSOR_ID_ANY};
  bool packet_input_{false};
  uint32_t bit_period_{90};
  bool rain_hold_{true};
  bool illuminance_filter_{true};

#ifdef USE_VEVOR_DECODER_REMOTE_RECEIVER
  uint8_t bits_[MAX_BITS];  // fixed array, no heap allocation
#endif

  // Last published frame, to drop the burst's second copy.
  uint8_t last_frame_[FRAME_BYTES]{};
  uint32_t last_frame_ms_{0};
  bool have_last_frame_{false};

  // Rain is a monotonic tick counter: it climbs, or it restarts at exactly zero
  // after a battery pull. Any other decrease is corruption - typically the
  // station sampling its own 16-bit counter mid-carry, which reports 256 ticks
  // too few with a perfectly valid checksum - and is never accepted, however
  // often it repeats. A drop to zero is the one ambiguous case, so it is held
  // back until the station has repeated it, which corruption will not do.
  // Counts are kept as raw ticks so the comparisons are exact integer ones.
  // -1 = nothing seen yet.
  //
  // A station that has been power-cycled usually announces itself with a new
  // id, which resolves the ambiguity immediately: the rain counter behind a
  // new id has nothing to do with the one we were tracking. That is only a
  // shortcut, not a guarantee, so the repeat check above still backs it up.
  int32_t last_sensor_id_{-1};
  int32_t last_rain_ticks_{-1};
  uint8_t pending_rain_reset_count_{0};
};

}  // namespace vevor_decoder
}  // namespace esphome
