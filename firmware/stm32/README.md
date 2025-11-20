# LocuSense STM32U0 Firmware

Firmware for the LocuSense air-quality node built around an STM32U073RCT6.
The code measures CO2/temperature/humidity/VOC with Sensirion sensors, renders the
data on a 2.13" e-paper display, pushes a compact 6-byte payload over LoRaWAN or
Matter/Thread (via ESP32-C6), and turns power rails on/off aggressively to maximize
runtime from the Li-Ion + LiSOCl₂ hybrid battery pack.

> **See also:** [project overview](../../README.md) for hardware context, [ESP32-C6 firmware](../esp32c6/README.md) for the Matter bridge, and the Home Assistant guides ([Matter](../../home-assistant/matter/README.md) / [LoRaWAN](../../home-assistant/lora/README.md)) for end-to-end integrations.

## Key Features
- **Single state machine** in `Core/Src/app.c` drives measurements, communications,
  GUI, sleep, user configuration, recovery paths, and sensor calibration.
- **Sensirion sensor stack**: SCD41 (CO2), SHT41 (T/RH), SGP40 (VOC with the
  `sensirion_gas_index_algorithm.c`). All drivers live in
  `Core/Src`.
- **Dual transport**: LoRaWAN via Seeed Wio-E5 (SPI2 + AT modem) or Matter/Thread
  using an ESP32-C6 UART bridge (`Core/Src/WioE5.c`, `Core/Src/esp32c6.c`). Offline
  GUI-only mode is also available.
- **E-paper GUI** for the Waveshare 2.13" v4 display (full dashboard, partial
  updates, QR codes) implemented in `Core/Src/GUI*.c` with bitmap assets in
  `ImageData*.c`.
- **Power management**: software controls for PS1 (sensors), PS2 (radio), PS4
  (display), and sleep GPIO handling (`Core/Src/gpio_sleep.c`) to reach STOP2.
- **User configuration tools**: PB11 button (5 s = CONFIG, 15 s = recalibration,
  30 s = reset) and an interactive UART4 console with HELP menu
  (`Core/Src/button_handler.c`, `Core/Src/conf_console.c`).
- **Persistent storage**: I2C EEPROM M24C02 keeps `AppConfig`, Wio-E5 OTAA
  credentials, and Matter QR/manual codes (`Core/Src/ee_config.c`).

## Hardware Blocks
### MCU and Peripherals
- **STM32U073RCT6** @ 64 MHz, 256 KB Flash, 40 KB SRAM. Pin mapping comes from
  `LocuSense_final.ioc` and `Core/Inc/main.h`.
- **UART4 (PA0/PA1)** is the console/debug port; `_write()` in `Core/Src/main.c`
  routes `printf` there.
- **UART1 (PA9/PA10)** serves the ESP32-C6 bridge and can be reused for direct AT
  control of the radio.
- **SPI1** drives the e-paper panel; **SPI2** is wired to Wio-E5.
- **I2C1** talks to all Sensirion sensors plus the EEPROM.
- **ADC1** samples both battery rails and BQ25185 STAT pins.
- **LPTIM2** provides a 10 s tick for continuous VOC sampling whenever USB power
  is present.

### Communication Blocks
- **Wio-E5 LoRaWAN**: chip select / reset / GPIO pins are listed in
  `Core/Inc/main.h`. The driver in `Core/Src/WioE5.c` handles OTAA join, time
  requests, confirmed/unconfirmed uplinks, and low-power sleep.
- **ESP32-C6 Matter/Thread bridge**: WAKE on PC6, READY on PC7, ASCII command set
  for `PING`, `STATUS`, `QR`, commissioning control, and data frames is in
  `Core/Src/esp32c6.c`. All commands are also exposed through the config console.

### Sensors and Power Tree
- **SCD41, SHT41, SGP40** share the PS1 rail plus I2C pull-ups
  (PULL_CNTR). CO2 recalibration is performed inside `STATE_RECALIBRATION`.
- **BQ25185** charger monitors USB presence (`USB_ON` pin) and charger status.
  See `Core/Src/BQ25185.c` for helper functions used by GUI and telemetry.
- **External EEPROM M24C02** stores `AppConfig_t`, `WioOtaaConfig_t`, and
  `EspC6Pairing_t`.
- **GPIO essentials**: LED (PB10), user button (PB11, EXTI4_15), PSx power-switch
  pins defined in `main.h`.

## Repository Layout
- `Core/Inc`, `Core/Src` - application code, drivers, state machine, GUI.
- `Drivers` - STM32U0 HAL delivered by Cube (do not edit manually).
- `Debug`, `Release` - STM32CubeIDE build outputs (regenerate locally).
- `.mxproject`, `.ioc`, `.project`, `.cproject` - STM32CubeIDE / CubeMX metadata.
- `.vscode` - optional editor helpers.

Large bitmap sources (`ImageData*.c`, `images.c`) are generated assets; avoid
editing them manually.

## Application State Machine
`App_Run()` (`Core/Src/app.c`) iterates a `switch(app.currentState)` state machine:
1. **STATE_INIT** - handles cold boot or STOP2 wake-up, loads EEPROM config,
   initializes sensors, GUI, radios, and bookkeeping.
2. **STATE_MEASURE** - gathers CO2/T/RH (per `measureMode`), optional VOC, builds
   the 6-byte payload, and updates the GUI cache.
3. **STATE_BAT_MEASURE** - samples batteries/USB/charger according to
   `interval_bat_sec`.
4. **STATE_SEND_DATA** - checks `ShouldTxNow()` and sends via LoRa or ESP while
   tracking ACK statistics.
5. **STATE_RECOVER** - rejoins LoRa or resets ESP32-C6 after repeated failures.
6. **STATE_TIME_REQ** - triggers a LoRaWAN time downlink to sync the RTC.
7. **STATE_GUI** - performs full or partial display refreshes (`GUI.c`).
8. **STATE_SLEEP** - enters STOP2 (unless USB+VOC is active or the button is held)
   using an RTC wake-up timer.
9. **STATE_RECALIBRATION** - long button press starts SCD41 400 ppm recal and UI
   feedback.
10. **STATE_CONFIG** - halts sensors, powers ESP for QR commands, shows the config
    icon, and runs the UART4 console until exit/inactivity.

`ButtonHandler` (`Core/Src/button_handler.c`) maps press duration:
- `<5 s` - reserved for future quick actions.
- `5..14 s` - toggles the CONFIG state (`app.confFlag++`).
- `15..29 s` - requests CO2 recalibration (`app.recalibFlag = 1`).
- `>=30 s` - triggers a system reset (`NVIC_SystemReset()`).

## Configuration Console
- **How to enter:** hold PB11 for ~5 s or call `ConfConsole_Start()` directly.
- **Connection:** UART4 @ 115200 8N1 (`DBG_TX`=PA0, `DBG_RX`=PA1). In CONFIG mode
  the ESP rail stays powered (WAKE held low) so QR/STATUS commands work.
- **Commands:** `HELP` prints the full list. Highlights:
  - `SHOW`, `SET <key> <value>` manipulate `AppConfig`.
  - `WIO ...` family for OTAA keys, JOIN, confirmed/unconfirmed SEND, TIME.
  - `ESP ...` for PING/STATUS/VERSION/QR/COMM START-STOP/FABRICS/factory reset,
    plus data test frames.
  - `JOIN` (LoRa), `RESET` (restore defaults in RAM), `SAVE` (write EEPROM),
    `EXIT`.
- Each command prints textual feedback, and optional E-Ink partial updates use
  `GUI_DrawConfIcon`, `GUI_DrawConfOk`, and `GUI_DrawConfError`.

Supported `SET <key> <value>` pairs (same parser as `Core/Src/conf_console.c`):

| Key (alias)                  | Allowed values / units                                                                 | Notes                                                                                     |
|-----------------------------|-----------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------|
| `measure_mode` (`mm`)       | `0` = CO2+T+RH, `1` = T+RH only                                                         | Enables/disables SCD41 sampling.                                                          |
| `voc_mode` (`voc`)          | `0` = disabled, `1` = VOC only while USB is present                                     | Continuous VOC draws more current; keep it off for pure battery.                          |
| `comms_mode` (`comms`)      | `0` = offline GUI, `1` = LoRaWAN (Wio-E5), `2` = Matter over Thread (ESP32-C6 build)    | Chooses which radio stack initializes in `STATE_INIT`.                                    |
| `interval_measure_sec` (`tm`)| Seconds                                                                                | Primary measurement cadence.                                                              |
| `interval_sleep_sec` (`ts`) | Seconds                                                                                 | STOP2 sleep gap between cycles when VOC is idle.                                          |
| `interval_time_req_sec` (`tt`)| Seconds (`0` disables)                                                                | LoRa network time-sync request interval.                                                  |
| `interval_bat_sec` (`tb`)   | Seconds                                                                                 | Battery/USB/charger measurement cadence.                                                  |
| `tx_min_interval_sec` (`txmin`)| Seconds                                                                              | Minimum spacing enforced between any TX events.                                           |
| `tx_max_interval_sec` (`txmax`)| Seconds (`0` disables heartbeat)                                                      | Forces a heartbeat if no delta-triggered TX happens before this timeout.                  |
| `th_temp_01C` (`dT`)        | Delta in 0.01 °C                                                                        | GUI/radio updates trigger once |ΔT| exceeds this.                                       |
| `th_rh_01pct` (`dRH`)       | Delta in 0.01 %RH                                                                       | Same principle for humidity.                                                              |
| `th_co2_ppm` (`dCO2`)       | Delta in ppm                                                                            | Same principle for CO₂.                                                                   |

> Always provide the numeric value in the console (e.g. `SET comms_mode 2` for Matter, `SET comms_mode 1` for LoRa).

## Communications and Payload Format
- **Payload layout**: 6 bytes total  
  `T` = int16 in 0.01 degC, `RH` = uint16 in 0.01 %RH, `CO2` = uint16 ppm.
  `build_payload_from_kv()` in `conf_console.c` mirrors the application format.
- **LoRaWAN**: `ShouldTxNow()` enforces minimum/maximum intervals (`tx_min/max`),
  delta thresholds (`th_temp_01C`, etc.), and heartbeats. `Wio_SendData()` (or
  `_SendData_Unconfirmed`) handles uplinks; ACK counters influence `STATE_RECOVER`.
- **Matter/Thread**: ESP32-C6 accepts `DATA HEX` or `DATA KV` ASCII frames with a
  WAKE/READY handshake. `EspC6_DataBat()` reports dual-battery status over Matter.
- **Offline mode**: set `app.cfg.commsMode = COMMS_OFFLINE` to keep UI/logging only.

## GUI Notes
- All rendering lives in `Core/Src/GUI.c`, `GUI_Paint.c`, and `Fonts/`.
- `GUI_Display()` draws the full dashboard (connectivity/time, CO2/T/RH/VOC grid,
  emoticon, battery strip, sleep state). `GUI_UpdateState()` and
  `GUI_UpdateVocLine()` perform partial refreshes to reduce ghosting.
- QR codes (used for Matter COMM pairing) are generated via `qrcodegen.c` and
  `GUI_DrawQr`.
- Before STOP2 the display is put to sleep via `EPD_PowerOff()`. After resume
  call `GUI_OnEPDResumed()` to rebuild the partial base image.

## Build and Flash
1. Install **STM32CubeIDE 1.15+** (or CubeMX 6.11+) with the STM32CubeU0 package.
2. `File -> Import -> Existing Projects into Workspace`, pick `firmware/stm32`.
3. When regenerating from `.ioc`, use CubeMX inside CubeIDE and keep USER CODE
   sections intact (`app.c`, `main.c`, etc.).
4. Select the `Debug` or `Release` configuration and run **Build**; binaries end
   up in the respective folders.
5. Flash via ST-LINK (`Run -> Debug`) or CLI  
   `STM32_Programmer_CLI -c port=SWD -d Debug/LocuSense_final.elf`.

> LoRaWAN/Matter features require valid OTAA credentials (set with the console)
> and a programmed ESP32-C6 firmware. Without them, `STATE_RECOVER` will keep
> trying to reconnect.

## EEPROM Defaults and Calibration
- `AppConfig_SetDefaults()` (in `app.c`) configures:
  - CO2+T+RH every 30 s (`interval_measure_sec`) with 30 s STOP2 gaps,
    LoRa heartbeat every 3600 s, delta thresholds of 0.2 degC / 1 %RH / 10 ppm.
  - `commsMode = COMMS_MATTER`, `vocMode = VOC_CONT_USB`.
- `EE_Config_Load()` / `EE_Config_Save()` use a simple magic+CRC layout that
  stores `AppConfig`, Wio OTAA keys, and ESP QR/manual strings. Invalid pages
  fall back to defaults.
- `State_Recalib()` executes `SCD41_ForceRecalibration(&hi2c1, 400)`. Make sure
  the device sits in a 400 ppm environment before triggering it.

## Development Tips
- **Adding sensors**: extend `STATE_MEASURE`, update `GUI.c`, and consider payload
  compatibility for both LoRa and Matter paths.
- **New console commands**: extend the dispatch table in `conf_console.c`.
- **Power tuning**: keep `KEEP_PINS` in `app.c` up to date so new peripherals
  survive STOP2 without leakage.
- **Testing**: stream logs over UART4 (`printf`) and watch the LED/E-Ink states.
  For LoRa debugging you can enable extra AT logs on Wio-E5 if needed.

Contributions are welcome. The most critical pieces are the application state
machine (`Core/Src/app.c`), configuration console (`Core/Src/conf_console.c`),
and GUI (`Core/Src/GUI.c`). Whenever CubeMX regeneration is required, double-check
USER CODE markers and commit the updated `.ioc`.
