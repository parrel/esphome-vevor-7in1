# ESPHome Vevor 7-in-1 Weather Station

An ESPHome external component that decodes the 868/915 MHz FSK telegrams sent by
Vevor 7-in-1 weather stations (also sold under other brands using the same
protocol) and publishes them as native ESPHome sensors.

Input comes from a CC1101 in packet mode (recommended) or from
`remote_receiver`.

> **Before you buy anything:** you need a CC1101 module for the **868 MHz**
> band *with an 868 MHz antenna* (915 MHz for US stations). The 433 MHz modules
> and antennas that dominate search results will not work — see
> [Hardware](#hardware).

## What you get

| Option | Type | Unit |
| --- | --- | --- |
| `temperature` | sensor | °C |
| `humidity` | sensor | % |
| `wind_speed` | sensor | km/h |
| `wind_gust` | sensor | km/h |
| `wind_direction` | sensor | ° |
| `rain` | sensor | mm (total increasing, wraps at 15 209.8 mm) |
| `uv_index` | sensor | index |
| `illuminance` | sensor | lx |
| `battery_low` | binary sensor | — |
| `rssi` | sensor, packet mode | dBm |
| `frequency_offset` | sensor, packet mode | kHz |
| `link_quality` | sensor, packet mode | LQI (lower is better) |

All of them are optional; leave out the ones you don't want. The last three are
diagnostics from the radio, handy when tuning `frequency` or the antenna.

## Hardware

- CC1101 module **for the 868 MHz band** (EU) — or 915 MHz if your station is a
  US model. See the warning above: the band matters, and so does the antenna.
- SPI wired to the ESP32, plus the module's **GDO0** pin to a free GPIO.

## Configuration

Packet mode needs ESPHome **2026.2 or newer**.

```yaml
external_components:
  - source: github://parrel/esphome-vevor-7in1@main
    components: [vevor_decoder]

spi:
  clk_pin: GPIO18
  mosi_pin: GPIO22
  miso_pin: GPIO19

cc1101:
  id: radio
  cs_pin: GPIO15
  gdo0_pin: GPIO12
  # 915 MHz for a US station.
  frequency: 868.35MHz
  modulation_type: 2-FSK
  symbol_rate: 11260
  fsk_deviation: 38kHz
  filter_bandwidth: 203kHz
  foc_bs_cs_gate: false
  packet_mode: true
  packet_length: 21
  sync1: 0xCA
  sync0: 0x54

vevor_decoder:
  cc1101_id: radio
  # Set this once you have seen your station's id in the logs, so a
  # neighbour's station cannot feed data into your sensors.
  #sensor_id: 0x1A2B

  temperature:
    name: "Outside Temperature"
  humidity:
    name: "Outside Humidity"
  wind_speed:
    name: "Wind Speed"
  wind_gust:
    name: "Wind Gust"
  wind_direction:
    name: "Wind Direction"
  rain:
    name: "Rain Total"
  uv_index:
    name: "UV Index"
  illuminance:
    name: "Illuminance"
  battery_low:
    name: "Weather Station Battery Low"
  rssi:
    name: "Weather Station RSSI"
  frequency_offset:
    name: "Weather Station Frequency Offset"
  link_quality:
    name: "Weather Station Link Quality"
```

Every sensor is optional; leave out the ones you do not want.

### Radio settings

Packet mode is less forgiving than `remote_receiver`:

- **`frequency`**: must be within 50 kHz of the station; see
  [Troubleshooting](#troubleshooting).
- **`symbol_rate: 11260`**: the station's measured rate; 11111 slips bits.
- **`foc_bs_cs_gate: false`**: otherwise frequency correction and clock
  recovery stay frozen and frames arrive corrupted.
- **`sync1`/`sync0`**: if nothing decodes, try the inverted `0x35` / `0xAB`.

The build rejects a `cc1101` block that cannot receive Vevor frames. Each
decoded frame logs the signal quality:

```
[D][vevor_decoder]: [1A2B] Signal: RSSI -85.5 dBm, frequency offset +9.5 kHz, LQI 45
```

### Alternative: remote_receiver

The original input, kept for existing setups, older ESPHome versions and
wiring on GDO2. Some setups never receive a frame this way.

```yaml
cc1101:
  cs_pin: GPIO15
  frequency: 868.35MHz
  modulation_type: 2-FSK
  symbol_rate: 11111
  fsk_deviation: 38kHz
  filter_bandwidth: 203kHz

remote_receiver:
  id: rf_receiver
  pin: GPIO12
  filter: 65us
  idle: 2000us
  # The default (192) cuts the burst off after the first copy.
  receive_symbols: 512

vevor_decoder:
  receiver_id: rf_receiver
  # ...sensors as above
```

### Options

- **`cc1101_id`** (*optional*, ID): the `cc1101` to use in packet mode.
- **`receiver_id`** (*optional*, ID): the `remote_receiver` to use instead.
  Without either, the only `remote_receiver` is used, else the only `cc1101`.
- **`sensor_id`** (*optional*, 0–0xFFFF): only publish frames from this station
  id. **Set this.** Without it, a neighbour's station on the same protocol will
  silently overwrite your readings. Leave it out for the first run, read the id
  out of the logs, then add it.
- **`bit_period`** (*optional*, time, default `90us`): `remote_receiver` only.
  The on-air NRZ bit period. Only worth touching if your receiver's timing is
  skewed and frames never validate.
- **`rain_hold`** (*optional*, boolean, default `true`): rain is a counter that
  can only climb, or restart at exactly zero after a battery pull. A frame
  reporting any *other* lower total is corrupt and is never accepted, however
  often it repeats: the common cause is the station reading its own 16-bit
  counter while it carries, which reports the wrapped low byte with a stale high
  byte — exactly 256 ticks (59.6 mm) short, with a valid checksum, once every
  59.6 mm of rain. A drop to zero is the ambiguous case: if the station id
  changed it is a reset and zero is taken immediately, otherwise the previous
  total is held until the station repeats the zero three times, which corruption
  will not do but a real reset will. Set to `false` to publish every total as
  received — a count below zero is impossible either way and is still rejected.
- **`illuminance_filter`** (*optional*, boolean, default `true`): drop
  illuminance readings that contradict the UV index in the same frame — zero lux
  with non-zero UV, or lux far above what the reported UV index allows. This is
  a heuristic; turn it off if you see legitimate readings being suppressed.
- Any of the sensor blocks above, each taking the standard ESPHome
  sensor/binary sensor options, including `filters:`.

## Multiple stations

One receiver can serve any number of stations. Declare a `vevor_decoder` block
per station, each pinned to its own `sensor_id`:

```yaml
vevor_decoder:
  - sensor_id: 0x1A2B
    temperature:
      name: "Garden Temperature"
    rain:
      name: "Garden Rain"

  - sensor_id: 0x7C41
    temperature:
      name: "Allotment Temperature"
    rain:
      name: "Allotment Rain"
```

Every block sees every frame and keeps the ones matching its own `sensor_id`,
so the stations stay fully independent — including their rain totals, which are
tracked per block.

`sensor_id` is what makes this work, and it is not optional here. Without it
each block accepts every frame, and all your stations end up writing to all your
sensors.

Two things worth knowing. Each block decodes each frame independently, so the
cost grows with the number of stations — irrelevant for a handful, worth
remembering if you have many. And since stations generally pick a new id at
power-up, replacing batteries in one of them means finding its new id and
editing the matching block.

## Finding your station id

Leave `sensor_id` out, set the logger to `DEBUG`, and watch for lines like:

```
[D][vevor_decoder]: [1A2B] T=18.4°C H=72% wind=3.2 (gust 5.6) km/h dir=214° rain=12.6mm UV=2 lux=18400
```

`1A2B` is the id to put in your config. Stations of this kind generally pick a
new id each time they power up, so expect to update this after a battery
change — check the logs again rather than assuming it survived.

## Troubleshooting

Stations are often well off their nominal frequency (70 kHz has been seen),
while the CC1101 only corrects up to ±50 kHz. Set the logger to `VERBOSE` and
compare with:

- **Nothing at all, not even `Packet rejected` lines:** check the wiring,
  especially GDO0 to `gdo0_pin`.
- **Only `Packet rejected` lines with random bytes, RSSI around −100 dBm, at
  irregular times:** the radio works but never finds the station. Step
  `frequency` from 868.25 to 868.45 MHz in 25 kHz steps, a minute each.
- **Frames decode:** fine-tune `frequency` until the logged frequency offset is
  within a few kHz. An offset near ±49 kHz is the correction's limit, so the
  station is further out than shown. LQI drops (lower is better) as you get
  closer.
- **`Packet rejected` every 20 s with a strong RSSI:** the station is received
  but corrupted; check `symbol_rate: 11260` and `foc_bs_cs_gate: false`.
- **`remote_receiver` with "0 sync matches" on every burst:** usually the same
  frequency problem. Packet mode shows the offset, which makes it easy to find.

## License

MIT — see [LICENSE](LICENSE).

---

Protocol reference: [rtl_433](https://github.com/merbanan/rtl_433),
`src/devices/vevor_7in1.c`.
