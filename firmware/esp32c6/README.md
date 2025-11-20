Matter Multi-Sensor (UART)
==========================

Matter over Thread application for an ESP32-C6 Sleepy End Device (SED) that acts as a bridge for an external STM32-based sensor head. The ESP32 exposes three environmental sensors (temperature, humidity, CO₂) plus three Matter Power Source endpoints while the STM32 pushes measurements over a simple ASCII UART protocol.

The design goal is reliability with minimal wake time: the ESP32 sleeps between Thread polls, wakes instantly on `WAKE`, and mirrors STM32 telemetry to Matter clusters without additional parsing on the controller side.

> **Related docs:** [Project overview](../../README.md) for hardware context, [STM32 firmware README](../stm32/README.md) for the sensor-side state machine, and [Home Assistant Matter guide](../../home-assistant/matter/README.md) for commissioning and dashboards.

Key features
------------
- ESP-Matter based Matter-over-Thread SED with commissioning helpers (QR/manual codes sent over UART).
- Low-speed UART (9600 Bd) plus WAKE/READY GPIO handshake so the STM32 can wake the ESP32 from light sleep when a frame is ready to send.
- Environmental telemetry comes from the STM32 (`DATA ...` frames) and is mapped to Matter Temperature Measurement, Relative Humidity Measurement, Air Quality, and the custom Concentration Measurement cluster (CO₂).
- Rich power-source modelling: independent endpoints for USB (wired), Li-Ion (rechargeable), and LiSOCl₂ (primary cell) including voltage, presence and charge/replace status derived from telemetry.
- UART command channel allows the STM32 to control commissioning (`COMM START/STOP`), factory reset, QR/manual code retrieval, and to query fabrics or runtime status.
- Light-sleep aware architecture: the ESP32 drops into light sleep between Thread poll intervals and only wakes when `WAKE` rises or when Matter stack work (commissioning/events) is pending.

Hardware & wiring
-----------------
- **MCU**: ESP32-C6, built as a low-power Sleepy End Device (Thread stack enabled in `sdkconfig`).
- **UART**: `UART1` @ 9600 baud, 8N1.
  - `GPIO5` – ESP TX → STM32 RX
  - `GPIO4` – ESP RX ← STM32 TX
  - Idle lines stay HIGH. CRLF (`\r\n`) terminates every line.
- **Handshake GPIOs**:
  - `WAKE` (`GPIO2`, input, RTC capable). Driven HIGH by the STM32 to signal that a frame is ready and to wake the ESP32 from light sleep.
  - `READY` (`GPIO3`, output). Raised by the ESP32 once it is ready to read a frame; the STM32 should only transmit while `READY=1`.
- **Power meta telemetry**: The STM32 reports Li-Ion, LiSOCl₂ and USB state (voltages, charger state) through `DATA BAT ...` lines. Those values are forwarded to the relevant Matter Power Source clusters.

Prerequisites
-------------
- ESP-IDF with esp-matter support (v5.2 or v5.3 are known-good).
- A serial/JTAG adapter wired to `UART1` pins and access to the WAKE/READY GPIOs.
- A Thread border router in your test environment (Home Assistant + SkyConnect works well).

Thread SED & light-sleep behavior
---------------------------------
- The device runs as a Thread Sleepy End Device, so after finishing Matter/FreeRTOS work it quickly enters light sleep to reduce consumption.
- ESP-IDF PM configuration keeps the CPU at a fixed frequency but enables light sleep whenever the idle task runs; the OpenThread stack schedules periodic polls to keep parent links alive.
- UART activity gates sleep via the WAKE/READY handshake:
  - STM32 asserts `WAKE` to wake the ESP32 (WAKE is RTC capable, so it exits light sleep immediately).
  - The UART task raises `READY`, acquires a PM lock (`esp_pm_lock_acquire()`), drains the line, processes the incoming `DATA`/command line, and only then releases the lock and lowers `READY`.
  - As soon as `READY` falls and no further work is pending, the ESP32 returns to light sleep until the next Thread poll or WAKE event.
- Commissioning and other Matter tasks can also keep the chip awake temporarily; once those tasks finish, the default behavior is to sleep again unless the STM32 keeps the link busy.

ICD presets in `sdkconfig`
--------------------------
This SED follows the esp-matter ICD example profiles (`examples/icd_app`) so you can pick between Short Idle Time (SIT) and Long Idle Time (LIT) behavior by selecting a preset `sdkconfig.defaults.*` file before regenerating `sdkconfig`.

**Default SIT profile (current `sdkconfig.defaults`):**

| Parameter | Value |
| --- | --- |
| ICD Fast Polling Interval | 500 ms |
| ICD Slow Polling Interval | 5000 ms |
| ICD Active Mode Duration | 1000 ms |
| ICD Idle Mode Duration | 60 s |
| ICD Active Mode Threshold | 1000 ms |

**Optional LIT profile (`sdkconfig.defaults.esp32c6.lit` or `.esp32h2.lit`):**

| Parameter | Value |
| --- | --- |
| ICD Fast Polling Interval | 500 ms |
| ICD Slow Polling Interval | 20000 ms |
| ICD Active Mode Duration | 1000 ms |
| ICD Idle Mode Duration | 600 s |
| ICD Active Mode Threshold | 5000 ms |

Pick the preset that matches your responsiveness vs. power target, then run `idf.py set-target esp32c6` followed by `idf.py reconfigure` to apply the chosen defaults. SIT gives snappier reaction to new data at the cost of more parent polls; LIT stretches the idle window and slow poll interval for lower average current on battery.

What each interval does:
- Fast Polling Interval: how often the SED asks its parent for pending data while it considers itself active (short, responsive window).
- Slow Polling Interval: how often it polls in idle mode (dominant factor for average current).
- Active Mode Duration: how long the device stays in the fast-poll state after activity such as UART traffic or a Matter event.
- Idle Mode Duration: how long the device remains in idle before considering a re-entry to active mode.
- Active Mode Threshold: minimum elapsed time between fast-poll periods; shorter values bias toward responsiveness, longer values bias toward lower current.

Estimated current draw (ESP32-C6 module only, no radio TX):
- SIT preset: ~0.25-0.35 mA average in steady idle (Thread polls every 5 s, brief wake bursts).
- LIT preset: ~0.10-0.15 mA average in steady idle (20 s slow polls dominate).
- Active UART window or commissioning: 8-15 mA while the CPU stays awake; Thread TX/RX spikes add short peaks. The STM32 and sensors add on top of these numbers.

UART text protocol
------------------
Every UART frame is a single ASCII line terminated with `CRLF`. The STM32 is the master that asserts `WAKE`, waits for `READY=1`, sends one line, and then releases `WAKE`.

### DATA frames (STM32 → ESP32)

| Syntax | Purpose | Notes |
| --- | --- | --- |
| `DATA HEX <12 hex digits>` | Compact binary payload (T/RH/CO₂). | Bytes: `T_hi T_lo RH_hi RH_lo CO2_hi CO2_lo`. Temperature is 0.01 °C units, humidity 0.01 % RH, CO₂ ppm. |
| `DATA KV T=<i16> RH=<u16> CO2=<u16>` | Human-readable telemetry. | Same units as HEX. |
| `DATA BAT KV LI=<u16> LS=<u16> USB=<0|1> CHG=<0..255>` | Power metadata. | `LI/LS` are millivolts for Li-Ion and LiSOCl₂ packs, `USB` indicates VBUS presence, `CHG` mirrors the raw BQ25185 charger status. |

Valid frames update Matter attributes immediately (`update_matter_clusters()` and `update_power_source_clusters_from_meta()`). Invalid data (out-of-range numbers or malformed payloads) is ignored and only logged.

### Command frames (STM32 ↔ ESP32)

STM32 may initiate the following commands; every command receives at least one response line via `replyf()`:

| Command | Response(s) | Notes |
| --- | --- | --- |
| `PING` | `PONG` | Health check. |
| `STATUS?` | `STATUS ONLINE role=SED` or `STATUS OFFLINE` | Thread attachment status. |
| `QR?` / `GET QR` | `COMM QR <code>` + `COMM MANUAL <code>` | Forces regeneration of QR and manual setup payloads. |
| `VERSION?` | `VERSION app=1.0.0 idf=<idf version> matter=1.x` | Firmware identification. |
| `FACTORYRESET` | `OK` | Triggers `InitiateFactoryReset()` on the Matter stack thread. |
| `FABRICS?` | Multi-line dump bracketed by `FABRICS BEGIN/END`. Each entry looks like `FABRIC idx=...` with compressed ID, fabric ID, node ID and vendor ID. |
| `COMM START` / `COMMISSION OPEN` | `COMM STATE Discover` + QR/manual lines | Schedules `OpenBasicCommissioningWindow()` for 10 minutes. |
| `COMM STOP` / `COMMISSION CLOSE` | `COMM STATE Closed` + `COMM CLOSED` | Calls `CloseCommissioningWindow()`. |
| (any other string) | `ERR unknown` | Default case. |

Additional asynchronous responses:
- `COMM STATE <Discover|Connected|Timeout|Closed|Complete|Stopped>` whenever Matter emits the respective commissioning events.
- `COMM QR ...`, `COMM MANUAL ...`, `COMM DONE`, `COMM TIMEOUT`, `COMM CLOSED` for higher-level commissioning state reporting.
- `STATUS ...` lines may also be pushed proactively when Thread state changes.

### Console hooks on the STM32 side

All of the commands above are exposed to the end user through the STM32 UART console
(`firmware/stm32/Core/Src/conf_console.c`). Typical usage:

- `ESP PING` / `ESP STATUS` / `ESP VERSION` — verify UART health and cached runtime status.
- `ESP QR` together with `ESP COMM START` / `ESP COMM STOP` — pull fresh QR/manual codes and open/close the Matter commissioning window (the STM32 also caches the data in EEPROM and draws the QR on the E-Ink UI).
- `ESP FABRICS`, `ESP FABRIC REMOVE <idx>`, `ESP FACTORYRESET` — manage stored commissioners without touching ESP-IDF firmware.
- `ESP SEND HEX|KV ...` — transmit ad-hoc telemetry frames via Matter for testing.

Make sure the STM32 config console stays on `comms_mode = COMMS_MATTER`, adjust measurement/telemetry intervals as needed, then `SAVE` to persist both STM32 config and ESP pairing data to EEPROM before rebooting.

Matter endpoints & clusters
---------------------------
- **Endpoint 1 – Temperature sensor**: standard Temperature Measurement cluster with ±40 °C to 125 °C range (0.01 °C format).
- **Endpoint 2 – Humidity sensor**: Relative Humidity Measurement cluster (0–100 % in 0.01 % units).
- **Endpoint 3 – Air quality / CO₂**:
  - Air Quality cluster (0x005B) exposing only the `AirQuality` attribute. The qualitative state is derived from CO₂ ppm thresholds (`Good` < 800 ppm → `ExtremelyPoor` ≥ 3000 ppm).
  - Concentration Measurement cluster (0x040D) configured for CO₂ in ppm (MEA feature, measurement medium “Air”). Attributes include `MeasuredValue`, `Min/MaxMeasuredValue`, `MeasurementUnit`, `MeasurementMedium`, and `Uncertainty`.
- **Endpoints 4–6 – Power Source clusters** (`init_power_source_endpoints()`):
  - USB wired source: `Status`/`WiredPresent`.
  - Li-Ion rechargeable source: voltage, presence, replacement-needed, charge level and charge state mapped from STM32 telemetry + BQ25185 status codes.
  - LiSOCl₂ replaceable source: voltage, presence, replacement-needed and coarse charge level heuristics.

Building & flashing
-------------------
1. Export the ESP-IDF environment that includes esp-matter (esp-idf v5.2/v5.3 works best). Example:
   ```bash
   source ~/esp/esp-idf/export.sh
   ```
2. Select the board target (the defaults assume ESP32-C6):
   ```bash
   idf.py set-target esp32c6
   ```
   Optional preset configs are provided under `sdkconfig.defaults.*` if you need to regenerate `sdkconfig`.
3. Build, flash and start a monitor:
   ```bash
   idf.py -p /dev/ttyUSB0 build flash monitor
   ```
4. Connect the STM32 UART and WAKE/READY lines after flashing so the ESP32 can immediately begin reading frames.

During development you can use `idf.py monitor` to watch the UART handshake logs (`UART RX task started`, `WAKE HIGH`, `DATA parse failed`, etc.). The monitor also shows the generated QR/manual codes when a commissioning window opens.

Bringing the system up
----------------------
1. Power on both MCUs; ensure the STM32 keeps `WAKE=LOW` until it sees `READY=HIGH` to avoid dropping frames at boot.
2. Send a `PING` frame to verify the UART link. If you do not receive `PONG`, check wiring or baud rate.
3. Issue `COMM START` to open a commissioning window and wait for the `COMM STATE Discover` message followed by the QR/manual payload. Use those codes to onboard the device with your Matter controller.
4. Begin streaming periodic `DATA` frames (either `HEX` or `KV`). The ESP32 updates the Matter attributes immediately; you can verify this via your controller’s state table or by subscribing to the clusters.
5. Periodically send `DATA BAT ...` so the power-source endpoints remain in sync with the real hardware state.

Troubleshooting tips
--------------------
- If the ESP32 never responds, make sure `WAKE` is toggling and that the STM32 waits for `READY=1` before transmitting.
- Commissioning commands run on the Matter platform thread; keep sending `COMM START` until you see `COMM STATE Discover` if fabrics were previously present.
- Use `FABRICS?` to inspect currently commissioned controllers, and `FACTORYRESET` to wipe NVS if the device behaves unexpectedly.
- Light sleep is enabled when PM is configured and Thread is idle; any UART RX window automatically acquires a PM lock to avoid missed bytes. If your STM32 chatters, consider batching multiple values into a single line.
