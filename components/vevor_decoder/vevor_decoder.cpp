#include "vevor_decoder.h"

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cmath>
#include <cstring>

namespace esphome {
namespace vevor_decoder {

static const char *const TAG = "vevor_decoder";

static const uint8_t FRAME_HEADER = 0xAA;
static const int FRAME_BITS = FRAME_BYTES * 8;  // 168
// The checksum covers every byte before it; the second counter is the first + 1.
static const int CHECKSUM_BYTE = 19;
static const int COUNTER_BYTE = 18;
static const int COUNTER_PLUS_ONE_BYTE = 20;

// A burst's two copies arrive ~30 ms apart, bursts 20 s apart.
static const uint32_t DUPLICATE_WINDOW_MS = 1000;

// Sync word 0xCA 0x54, MSB first, as individual bits.
static const uint8_t SYNC_WORD[16] = {
    1, 1, 0, 0, 1, 0, 1, 0,  // 0xCA
    0, 1, 0, 1, 0, 1, 0, 0,  // 0x54
};

// Cheap rejects so we bail out of non-Vevor bursts before doing real work.
// A frame needs sync (16 bits) + 168 data bits, so anything that cannot carry
// that many bits is not worth converting. The real filtering is done by the
// sync word and the checksum.
static const int MIN_RAW_TIMINGS = 40;
static const int MIN_DECODED_BITS = 16 + FRAME_BITS;
// A run longer than this many bit periods is a gap between bursts rather than
// payload. It has to be generous: NRZ payload legitimately contains long runs
// of identical bits (several zero bytes in a row is only 24+ bits), and
// discarding such a run destroys the frame around it.
static const int MAX_RUN_LENGTH = 64;

// Number of consecutive frames that must report a zeroed rain counter before it
// is accepted as a real reset rather than a corrupted frame.
static const uint8_t RAIN_RESET_CONFIRMATIONS = 3;

// A station that samples its own 16-bit rain counter while it carries reports
// the wrapped low byte with a stale high byte, i.e. exactly this many ticks too
// few. The frame's checksum is correct: the station transmits what it misread.
static const int32_t RAIN_MISSED_CARRY_TICKS = 256;

// Illuminance sanity check: with a UV index of N, lux above this multiple of
// (N + 1) is treated as a bit error rather than a real reading.
static const float LUX_PER_UV_STEP = 20000.0f;

// Protocol scaling constants.
static const int BASELINE = 257;       // offset applied to several fields
static const float TEMP_OFFSET = 500.0f;
static const float TEMP_SCALE = 0.1f;       // °C per count
static const float WIND_SCALE = 8.333f;     // counts per km/h
static const float GUST_SCALE = 1.25f;      // counts per km/h
static const float RAIN_SCALE = 0.233f;     // mm per counter tick

void VevorDecoder::dump_config() {
  ESP_LOGCONFIG(TAG, "Vevor 7-in-1 Weather Station Decoder:");
  if (this->sensor_id_ == SENSOR_ID_ANY) {
    ESP_LOGCONFIG(TAG, "  Station ID filter: any");
  } else {
    ESP_LOGCONFIG(TAG, "  Station ID filter: 0x%04X", (unsigned) this->sensor_id_);
  }
  if (this->packet_input_) {
    ESP_LOGCONFIG(TAG, "  Input: CC1101 packet mode");
  } else {
    ESP_LOGCONFIG(TAG, "  Input: remote_receiver");
    ESP_LOGCONFIG(TAG, "  Bit period: %u us", (unsigned) this->bit_period_);
  }
  ESP_LOGCONFIG(TAG, "  Rain hold on decrease: %s", YESNO(this->rain_hold_));
  ESP_LOGCONFIG(TAG, "  Illuminance plausibility filter: %s", YESNO(this->illuminance_filter_));
  LOG_SENSOR("  ", "Temperature", this->temperature_);
  LOG_SENSOR("  ", "Humidity", this->humidity_);
  LOG_SENSOR("  ", "Wind speed", this->wind_speed_);
  LOG_SENSOR("  ", "Wind gust", this->wind_gust_);
  LOG_SENSOR("  ", "Wind direction", this->wind_dir_);
  LOG_SENSOR("  ", "Rain", this->rain_);
  LOG_SENSOR("  ", "UV index", this->uv_);
  LOG_SENSOR("  ", "Illuminance", this->light_);
  LOG_BINARY_SENSOR("  ", "Battery low", this->battery_);
}

bool VevorDecoder::frame_valid_(const uint8_t *b) {
  if (b[0] != FRAME_HEADER)
    return false;

  uint8_t checksum = 0;
  for (int k = 0; k < CHECKSUM_BYTE; k++)
    checksum += b[k];
  if (checksum != b[CHECKSUM_BYTE])
    return false;

  return b[COUNTER_PLUS_ONE_BYTE] == (uint8_t) (b[COUNTER_BYTE] + 1);
}

VevorDecoder::FrameResult VevorDecoder::handle_frame_(const uint8_t *frame) {
  const uint16_t sensor_id = (frame[2] << 8) | frame[3];
  if (this->sensor_id_ != SENSOR_ID_ANY && sensor_id != (uint16_t) this->sensor_id_) {
    ESP_LOGV(TAG, "Ignoring frame from station 0x%04X (filtered)", sensor_id);
    return FrameResult::FILTERED;
  }

  const uint32_t now = millis();
  if (this->have_last_frame_ && now - this->last_frame_ms_ < DUPLICATE_WINDOW_MS &&
      memcmp(frame, this->last_frame_, FRAME_BYTES) == 0) {
    ESP_LOGV(TAG, "[%04X] Second copy of the frame, already published", sensor_id);
    return FrameResult::DUPLICATE;
  }
  memcpy(this->last_frame_, frame, FRAME_BYTES);
  this->last_frame_ms_ = now;
  this->have_last_frame_ = true;

  this->publish_frame_(frame);
  return FrameResult::PUBLISHED;
}

#ifdef USE_VEVOR_DECODER_CC1101
void VevorDecoder::on_packet(const std::vector<uint8_t> &packet, float freq_offset, float rssi, uint8_t lqi) {
  // Chance sync matches in noise arrive a few times a minute; the checks below
  // reject them.
  if (packet.size() != (size_t) FRAME_BYTES) {
    ESP_LOGV(TAG, "Ignoring %u-byte packet, expected %d", (unsigned) packet.size(), FRAME_BYTES);
    return;
  }

  // With inverted FSK polarity (sync 0x35 0xAB) every byte arrives inverted.
  const uint8_t inv = packet[0] == (uint8_t) ~FRAME_HEADER ? 0xFF : 0x00;
  uint8_t frame[FRAME_BYTES];
  for (int k = 0; k < FRAME_BYTES; k++)
    frame[k] = packet[k] ^ inv;

  if (!frame_valid_(frame)) {
    ESP_LOGV(TAG, "Packet rejected (RSSI %.1f dBm, LQI %u): %s", rssi, lqi, format_hex_pretty(packet).c_str());
    return;
  }

  if (this->handle_frame_(frame) == FrameResult::PUBLISHED) {
    ESP_LOGD(TAG, "[%02X%02X] Signal: RSSI %.1f dBm, frequency offset %+.1f kHz, LQI %u", frame[2], frame[3], rssi,
             freq_offset / 1000.0f, lqi);
  }
}
#endif

#ifdef USE_VEVOR_DECODER_REMOTE_RECEIVER
int VevorDecoder::timings_to_bits_(const std::vector<int32_t> &raw) {
  // The receiver hands us mark/space durations; the signal itself is NRZ, so
  // each duration expands into one or more bits of the same value.
  // Mark (positive) = 1, space (negative) = 0.
  const int period = (int) this->bit_period_;
  const int half_period = period / 2;
  int bit_count = 0;

  for (int32_t val : raw) {
    const uint8_t bit_val = val > 0 ? 1 : 0;
    const int duration = val > 0 ? val : -val;

    int num = (duration + half_period) / period;
    if (num < 1)
      num = 1;
    // Runs this long are inter-burst gaps, not payload. Skip them rather than
    // stopping: the frame we want may follow the gap, and the sync search below
    // tries every offset anyway.
    if (num > MAX_RUN_LENGTH)
      continue;

    if (bit_count + num > MAX_BITS)
      return MAX_BITS;
    for (int j = 0; j < num; j++)
      this->bits_[bit_count++] = bit_val;
  }
  return bit_count;
}

bool VevorDecoder::extract_frame_(int bit_offset, uint8_t inv, uint8_t *out) {
  for (int k = 0; k < FRAME_BYTES; k++) {
    uint8_t byte = 0;
    for (int m = 0; m < 8; m++)
      byte = (byte << 1) | (this->bits_[bit_offset + k * 8 + m] ^ inv);
    out[k] = byte;
  }
  return frame_valid_(out);
}

bool VevorDecoder::dump(remote_base::RemoteReceiveData src) {
  const auto &raw = src.get_raw_data();
  const int raw_size = (int) raw.size();

  // Vevor bursts are long. Don't judge them by how they start: on a weak or
  // slightly off-frequency signal the burst opens with noise, and the frame
  // after it can still be perfectly good.
  if (raw_size < MIN_RAW_TIMINGS)
    return false;

  const int bit_count = this->timings_to_bits_(raw);
  if (bit_count < MIN_DECODED_BITS)
    return false;

  // Search for the sync word in both polarities: which one a given receiver
  // produces depends on how the FSK demodulator is wired up.
  const int limit = bit_count - 16 - FRAME_BITS;
  int sync_matches = 0;
  for (int i = 0; i < limit; i++) {
    for (uint8_t inv = 0; inv < 2; inv++) {
      bool match = true;
      for (int j = 0; j < 16; j++) {
        if (this->bits_[i + j] != (SYNC_WORD[j] ^ inv)) {
          match = false;
          break;
        }
      }
      if (!match)
        continue;
      sync_matches++;

      uint8_t frame[FRAME_BYTES];
      if (!this->extract_frame_(i + 16, inv, frame))
        continue;

      return this->handle_frame_(frame) != FrameResult::FILTERED;
    }
  }

#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
  // One line per undecodable burst, with the bits it produced. remote_receiver's
  // own VERY_VERBOSE dump of the same burst is hundreds of lines and the logger
  // drops some of them, so this is the way to look at a failing burst.
  static const char HEX_DIGITS[] = "0123456789abcdef";
  char hex[MAX_BITS / 4 + 1];
  const int nibbles = (bit_count + 3) / 4;
  for (int n = 0; n < nibbles; n++) {
    uint8_t v = 0;
    for (int m = 0; m < 4; m++) {
      const int k = n * 4 + m;
      v = (v << 1) | (k < bit_count ? this->bits_[k] : 0);
    }
    hex[n] = HEX_DIGITS[v];
  }
  hex[nibbles] = '\0';
  ESP_LOGV(TAG, "Burst not decoded: %d timings, %d bits, %d sync matches; bits: %s", raw_size, bit_count,
           sync_matches, hex);
#endif
  return false;
}
#endif  // USE_VEVOR_DECODER_REMOTE_RECEIVER

void VevorDecoder::publish_frame_(const uint8_t *b) {
  const uint16_t sensor_id = (b[2] << 8) | b[3];
  const bool battery_low = (b[4] & 0x80) != 0;

  // Stations generally pick a new id when they power up, so a change of id
  // means the rain counter that follows belongs to a freshly reset station
  // (or a different one) and cannot be compared with what we had. Drop the
  // tracking rather than mistake the new counter for a corrupted old one.
  if ((int32_t) sensor_id != this->last_sensor_id_) {
    if (this->last_sensor_id_ >= 0) {
      ESP_LOGI(TAG, "Station id changed %04X -> %04X, restarting rain tracking",
               (unsigned) this->last_sensor_id_, sensor_id);
    }
    this->last_sensor_id_ = sensor_id;
    this->last_rain_ticks_ = -1;
    this->pending_rain_reset_count_ = 0;
  }

  const float temperature = (((b[5] << 8) | b[6]) - TEMP_OFFSET) * TEMP_SCALE;
  const float humidity = (float) b[7];

  float wind_speed = (((b[8] << 8) | b[9]) - BASELINE) / WIND_SCALE;
  if (wind_speed < 0.0f)
    wind_speed = 0.0f;
  const float wind_gust = b[10] / GUST_SCALE;

  int wind_direction = (((b[11] & 0x0F) << 8) | b[12]) - BASELINE;
  if (wind_direction < 0)
    wind_direction += 360;

  // The rain counter is an absolute tick count offset by BASELINE. It can only
  // climb, or restart at exactly 0 after a battery pull, so every frame has a
  // floor below which its total is impossible rather than merely implausible:
  // the last good total, or 0 when there is nothing to compare against.
  const int32_t rain_ticks = ((b[13] << 8) | b[14]) - BASELINE;
  const int32_t rain_floor = (this->rain_hold_ && this->last_rain_ticks_ > 0) ? this->last_rain_ticks_ : 0;
  float rain_mm = NAN;
  if (rain_ticks >= rain_floor) {
    // First reading, or the counter moved the only way it legitimately can.
    rain_mm = rain_ticks * RAIN_SCALE;
    this->last_rain_ticks_ = rain_ticks;
    this->pending_rain_reset_count_ = 0;
  } else if (rain_ticks == 0) {
    // The one decrease the counter is capable of: it was reset (battery pull)
    // and starts over at zero. A bit flip produces a one-off wrong value, while
    // a real reset keeps reporting zero, so wait and see.
    this->pending_rain_reset_count_++;
    if (this->pending_rain_reset_count_ >= RAIN_RESET_CONFIRMATIONS) {
      ESP_LOGI(TAG, "[%04X] Rain counter reset confirmed over %u frames", sensor_id,
               (unsigned) this->pending_rain_reset_count_);
      rain_mm = 0.0f;
      this->last_rain_ticks_ = 0;
      this->pending_rain_reset_count_ = 0;
    } else {
      // Republish the last good total so the sensor stays fresh.
      rain_mm = this->last_rain_ticks_ * RAIN_SCALE;
      ESP_LOGW(TAG, "[%04X] Rain counter reported zero (%u/%u), holding %.2f mm", sensor_id,
               (unsigned) this->pending_rain_reset_count_, (unsigned) RAIN_RESET_CONFIRMATIONS, rain_mm);
    }
  } else {
    // Below the floor and not a reset, so the frame is wrong however sound its
    // checksum is. The usual cause is the station sampling its own 16-bit
    // counter mid-carry, losing exactly RAIN_MISSED_CARRY_TICKS; no number of
    // repeats makes that value real, so it is never adopted.
    const int32_t lost = this->last_rain_ticks_ - rain_ticks;
    const char *cause = (lost > 0 && lost % RAIN_MISSED_CARRY_TICKS == 0) ? " (missed carry in the station)" : "";
    this->pending_rain_reset_count_ = 0;
    if (this->last_rain_ticks_ >= 0) {
      rain_mm = this->last_rain_ticks_ * RAIN_SCALE;
      ESP_LOGW(TAG, "[%04X] Impossible rain count %d ticks%s, holding %.2f mm", sensor_id, (int) rain_ticks, cause,
               rain_mm);
    } else {
      ESP_LOGW(TAG, "[%04X] Impossible rain count %d ticks%s, dropping the reading", sensor_id, (int) rain_ticks,
               cause);
    }
  }

  int uv_index = (b[15] & 0x1F) - 1;
  if (uv_index < 0)
    uv_index = 0;

  const bool lux_x10 = (b[16] & 0x80) != 0;
  float illuminance = (float) ((((b[16] & 0x7F) << 8) | b[17]) - BASELINE) * (lux_x10 ? 10.0f : 1.0f);
  if (illuminance < 0.0f)
    illuminance = 0.0f;

  // UV and illuminance come from the same sensor head, so they should agree.
  // A disagreement is a cheap way to catch a bit error in either field.
  bool illuminance_valid = true;
  if (this->illuminance_filter_) {
    if (uv_index > 0 && illuminance == 0.0f)
      illuminance_valid = false;
    if (illuminance > (uv_index + 1) * LUX_PER_UV_STEP)
      illuminance_valid = false;
  }

  ESP_LOGD(TAG, "[%04X] T=%.1f°C H=%.0f%% wind=%.1f (gust %.1f) km/h dir=%d° rain=%.1fmm UV=%d lux=%.0f%s%s",
           sensor_id, temperature, humidity, wind_speed, wind_gust, wind_direction, rain_mm, uv_index, illuminance,
           illuminance_valid ? "" : " [lux filtered]", battery_low ? " [battery low]" : "");

  if (this->temperature_ != nullptr)
    this->temperature_->publish_state(temperature);
  if (this->humidity_ != nullptr)
    this->humidity_->publish_state(humidity);
  if (this->wind_speed_ != nullptr)
    this->wind_speed_->publish_state(wind_speed);
  if (this->wind_gust_ != nullptr)
    this->wind_gust_->publish_state(wind_gust);
  if (this->wind_dir_ != nullptr)
    this->wind_dir_->publish_state((float) wind_direction);
  if (this->rain_ != nullptr && !std::isnan(rain_mm))
    this->rain_->publish_state(rain_mm);
  if (this->uv_ != nullptr)
    this->uv_->publish_state((float) uv_index);
  if (this->light_ != nullptr && illuminance_valid)
    this->light_->publish_state(illuminance);
  if (this->battery_ != nullptr)
    this->battery_->publish_state(battery_low);
}

}  // namespace vevor_decoder
}  // namespace esphome
