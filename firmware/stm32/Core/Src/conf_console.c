/**
 * @file conf_console.c
 * @author Evzen Steif
 * @brief Minimal interactive configuration console over UART (huart4).
 *
 * @details
 *   This module implements a simple line-based configuration console running
 *   over a UART interface (typically UART4). It is used in the CONFIG state
 *   of the main application state machine to:
 *
 *   - Inspect and modify the runtime configuration (AppConfig_t).
 *   - Manage LoRaWAN (Wio-E5) OTAA credentials and quick test uplinks.
 *   - Interact with the ESP32-C6 (Matter bridge) for status, QR codes,
 *     commissioning control and test frames.
 *   - Trigger EEPROM save/restore of configuration and pairing data.
 *
 *   ESP32-C6 and Wio-E5 command families:
 *     - ESP ... : proxies to esp32c6.c wrappers (PING/STATUS/QR/COMM/...)
 *     - WIO ... : OTAA keys management (SHOW/SET), quick data send
 *                 (confirmed/unconfirmed), JOIN, TIME.
 *
 *   Design note:
 *     - Wio OTAA keys are no longer handled via generic "SET <key> <value>".
 *       Use "WIO SET ..." and "WIO SHOW" instead for LoRa credentials.
 */

#include "conf_console.h"
#include "GUI.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "WioE5.h"     /* Wio helpers (send/join/time + g_wio + Wio_IsHexLen) */
#include "esp32c6.h"   /* ESP32-C6 wrappers + g_esp */
#include "app.h"
#include "ee_config.h"

/* ================= Console state ================= */

static UART_HandleTypeDef* s_huart = NULL;
static AppConfig_t*        s_cfg   = NULL;
static uint32_t          (*s_sod_now)(void) = NULL;
static uint32_t            s_inact_s = 600; /* default 10 min */

static volatile uint8_t    s_active  = 0;
static volatile uint8_t    s_rx_ch   = 0;
static char                s_line[128];
static size_t              s_len     = 0;

static volatile uint8_t    s_wants_exit = 0;
static uint32_t            s_last_act_s = 0;

static volatile uint8_t s_cmd_ready = 0;
static char             s_cmd[128];

/* ================= Utilities ================= */

/**
 * @brief HAL UART error callback hook.
 *
 * In case of an overrun or similar error on the console UART, the error flag
 * is cleared and the RX interrupt is restarted so that the console remains
 * responsive.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_huart) {   // or &huart4
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(huart, (uint8_t*)&s_rx_ch, 1);
    }
}


/**
 * @brief Transmit a zero-terminated string over the console UART.
 */
static void tx(const char* s) {
    if (!s_huart || !s) return;
    HAL_UART_Transmit(s_huart, (uint8_t*)s, (uint16_t)strlen(s), 1000);
}

/**
 * @brief Update the last-activity timestamp to the current time.
 *
 * Used on each character received to implement inactivity auto-exit.
 */
static inline void touch(void) {
    if (s_sod_now) s_last_act_s = s_sod_now();
}

/**
 * @brief Case-insensitive key comparison.
 */
static int keyeq(const char* a, const char* b) {
    while (*a && *b) {
        char ca = (*a>='A'&&*a<='Z') ? *a - 'A' + 'a' : *a;
        char cb = (*b>='A'&&*b<='Z') ? *b - 'A' + 'a' : *b;
        if (ca != cb) return 0;
        ++a; ++b;
    }
    return (*a == '\0' && *b == '\0');
}

/**
 * @brief Parse an unsigned 32-bit integer from decimal string.
 * @return 1 on success, 0 on failure.
 */
static int parse_u32(const char* s, uint32_t* out){
    char* e = NULL; unsigned long v = strtoul(s, &e, 10);
    if (e == s || *e != '\0') return 0;
    *out = (uint32_t)v; return 1;
}

/**
 * @brief Parse an unsigned 16-bit integer from decimal string.
 * @return 1 on success, 0 on failure.
 */
static int parse_u16(const char* s, uint16_t* out){
    uint32_t t; if (!parse_u32(s, &t) || t > 0xFFFFu) return 0; *out = (uint16_t)t; return 1;
}

/**
 * @brief Parse an unsigned 8-bit integer from decimal string.
 * @return 1 on success, 0 on failure.
 */
static int parse_u8 (const char* s, uint8_t * out){
    uint32_t t; if (!parse_u32(s, &t) || t > 0xFFu)   return 0; *out = (uint8_t)t;   return 1;
}

/**
 * @brief Build a 6-byte measurement payload from T/RH/CO2 values.
 *
 * Encodes:
 *   - T:   int16, 0.01 °C units
 *   - RH:  uint16, 0.01 %RH units
 *   - CO2: uint16, ppm
 */
static void build_payload_from_kv(int16_t t_01C, uint16_t rh_01pct, uint16_t co2_ppm, uint8_t out6[6]) {
    out6[0] = (uint8_t)((t_01C   >> 8) & 0xFF);
    out6[1] = (uint8_t)((t_01C        ) & 0xFF);
    out6[2] = (uint8_t)((rh_01pct>> 8) & 0xFF);
    out6[3] = (uint8_t)((rh_01pct     ) & 0xFF);
    out6[4] = (uint8_t)((co2_ppm >> 8) & 0xFF);
    out6[5] = (uint8_t)((co2_ppm      ) & 0xFF);
}

/**
 * @brief Try to cache COMM QR and MANUAL strings from the ESP RX buffer.
 *
 * This helper scans esp_rx_buffer for "COMM QR " and "COMM MANUAL " prefixes
 * and copies the trailing text (until newline) into g_esp.pairing.qr and
 * g_esp.pairing.manual, respectively.
 */
static void EspC6_TryCacheQrFromBuffer(void)
{
    const char *pqr = strstr(esp_rx_buffer, "COMM QR ");
    if (pqr) {
        pqr += 8;
        size_t n = 0;
        while (pqr[n] && pqr[n] != '\r' && pqr[n] != '\n' && n + 1 < sizeof(g_esp.pairing.qr)) n++;
        memcpy(g_esp.pairing.qr, pqr, n); g_esp.pairing.qr[n] = '\0';
    }
    const char *pmn = strstr(esp_rx_buffer, "COMM MANUAL ");
    if (pmn) {
        pmn += 12;
        size_t n = 0;
        while (pmn[n] && pmn[n] != '\r' && pmn[n] != '\n' && n + 1 < sizeof(g_esp.pairing.manual)) n++;
        memcpy(g_esp.pairing.manual, pmn, n); g_esp.pairing.manual[n] = '\0';
    }
}

/* ================= Public API ================= */

/**
 * @brief Initialize the configuration console module.
 *
 * @param init Pointer to initialization structure with:
 *             - huart:      UART handle used for RX/TX.
 *             - cfg:        Pointer to application configuration to manipulate.
 *             - sod_now:    Callback returning current time in seconds-of-day.
 *             - inactivity_s: Auto-exit timeout in seconds (0 = default 600s).
 */
void ConfConsole_Init(const ConfConsole_Init_t* init) {
    s_huart   = init->huart;
    s_cfg     = init->cfg;
    s_sod_now = init->sod_now;
    s_inact_s = (init->inactivity_s ? init->inactivity_s : 600u);
}

/**
 * @brief Start the configuration console.
 *
 * Resets internal input buffers, marks the console as active and starts
 * asynchronous RX on the configured UART.
 */
void ConfConsole_Start(void) {
    s_len = 0; s_active = 1; s_wants_exit = 0; touch();
    if (s_huart) HAL_UART_Receive_IT(s_huart, (uint8_t*)&s_rx_ch, 1);
}

/**
 * @brief Stop the configuration console.
 *
 * Console becomes inactive; UART RX can still be used by other code.
 */
void ConfConsole_Stop(void) { s_active = 0; }

/**
 * @brief Query whether the console is currently active.
 */
uint8_t  ConfConsole_IsActive(void){ return s_active; }

/**
 * @brief Get the timestamp (seconds-of-day) of the last activity.
 */
uint32_t ConfConsole_LastActivityS(void){ return s_last_act_s; }

/**
 * @brief Print a short banner to indicate configuration mode.
 */
void ConfConsole_PrintBanner(void){
    tx("CONFIG MODE\r\n");
}

/**
 * @brief Print the current configuration and radio pairing state.
 *
 * Outputs:
 *   - AppConfig fields (measurement/interval/thresholds).
 *   - Wio-E5 OTAA keys and ADR flag.
 *   - ESP32-C6 status and cached QR/MANUAL values.
 */
void ConfConsole_PrintCfg(void) {
    char buf[256];
    (void)snprintf(buf, sizeof(buf),
      "CFG measure_mode=%u voc_mode=%u comms_mode=%u "
      "interval_measure_sec=%lu interval_sleep_sec=%lu interval_time_req_sec=%lu interval_bat_sec=%lu "
      "tx_min_interval_sec=%lu tx_max_interval_sec=%lu "
      "th_temp_01C=%u th_rh_01pct=%u th_co2_ppm=%u\r\n",
      (unsigned)s_cfg->measureMode, (unsigned)s_cfg->vocMode, (unsigned)s_cfg->commsMode,
      (unsigned long)s_cfg->interval_measure_sec,
      (unsigned long)s_cfg->interval_sleep_sec,
      (unsigned long)s_cfg->interval_time_req_sec,
      (unsigned long)s_cfg->interval_bat_sec,
      (unsigned long)s_cfg->tx_min_interval_sec,
      (unsigned long)s_cfg->tx_max_interval_sec,
      (unsigned)s_cfg->th_temp_01C,
      (unsigned)s_cfg->th_rh_01pct,
      (unsigned)s_cfg->th_co2_ppm
    );
    tx(buf);

    char w[256];
    snprintf(w, sizeof(w),
        "WIO: AppEui=%s DevEui=%s AppKey=%s ADR=%u\r\n",
        g_wio.app_eui[0]?g_wio.app_eui:"<empty>",
        g_wio.dev_eui[0]?g_wio.dev_eui:"<empty>",
        g_wio.app_key[0]?g_wio.app_key:"<empty>",
        (unsigned)g_wio.adr_on);
    tx(w);

    const char* st =
        (g_esp.status == ESP_STATUS_ONLINE)  ? "ONLINE"  :
        (g_esp.status == ESP_STATUS_OFFLINE) ? "OFFLINE" : "<unknown>";

    char e[320];
    snprintf(e, sizeof(e),
        "ESP: status=%s qr=%s manual=%s\r\n",
        st,
        g_esp.pairing.qr[0] ? g_esp.pairing.qr : "<na>",
        g_esp.pairing.manual[0] ? g_esp.pairing.manual : "<na>");
    tx(e);
}

/**
 * @brief Print the interactive HELP text describing all supported commands.
 */
static void help(void)
{
    tx(
        "CONFIG HELP\r\n"
        "============\r\n"
        "\r\n"
        "Global commands:\r\n"
        "  HELP\r\n"
        "    Show this help.\r\n"
        "\r\n"
        "  SHOW\r\n"
        "    Print current device config, Wio-E5 keys and ESP32-C6 status/QR/manual.\r\n"
        "\r\n"
        "  EXIT\r\n"
        "    Leave CONFIG mode (same effect as inactivity timeout or 2x long button press).\r\n"
        "\r\n"
        "  SET <key> <value>\r\n"
        "    Set device-level config field (NOT Wio-E5 keys; those use WIO SET ...).\r\n"
        "    Keys and values:\r\n"
        "      measure_mode | mm\r\n"
        "        0 = CO2+T+RH (SCD41 + SHT41)\r\n"
        "        1 = T+RH only (SHT41)\r\n"
        "\r\n"
        "      voc_mode | voc\r\n"
        "        0 = VOC_DISABLED      (VOC sensor off)\r\n"
        "        1 = VOC_CONT_USB      (continuous VOC only when USB is present)\r\n"
        "\r\n"
        "      comms_mode | comms\r\n"
        "        0 = COMMS_OFFLINE     (no radio, GUI only)\r\n"
        "        1 = COMMS_LORA        (LoRaWAN via Wio-E5)\r\n"
        "        2 = COMMS_MATTER      (Matter over Thread via ESP32-C6, if enabled in FW)\r\n"
        "\r\n"
        "      interval_measure_sec | tm\r\n"
        "        Measurement period in seconds (CO2/T/RH or T/RH only).\r\n"
        "\r\n"
        "      interval_sleep_sec | ts\r\n"
        "        STOP2 sleep duration between cycles in seconds (if VOC is not running).\r\n"
        "\r\n"
        "      interval_time_req_sec | tt\r\n"
        "        LoRa network time-sync period in seconds (0 = never).\r\n"
        "\r\n"
        "      interval_bat_sec | tb\r\n"
        "        Battery/USB/charger measurement period in seconds.\r\n"
        "\r\n"
        "      tx_min_interval_sec | txmin\r\n"
        "        Minimum spacing between uplinks in seconds.\r\n"
        "\r\n"
        "      tx_max_interval_sec | txmax\r\n"
        "        Heartbeat timeout in seconds (0 = disable heartbeat).\r\n"
        "\r\n"
        "      th_temp_01C | dT\r\n"
        "        Temperature change threshold for TX/GUI in 0.01 degC (e.g. 20 = 0.20°C).\r\n"
        "\r\n"
        "      th_rh_01pct | dRH\r\n"
        "        Humidity change threshold for TX/GUI in 0.01 %%RH (e.g. 100 = 1.00%%).\r\n"
        "\r\n"
        "      th_co2_ppm | dCO2\r\n"
        "        CO2 change threshold for TX/GUI in ppm (e.g. 50).\r\n"
        "\r\n"
        "  JOIN\r\n"
        "    Immediate LoRa OTAA join attempt using current Wio-E5 (g_wio) keys.\r\n"
        "\r\n"
        "ESP32-C6 commands:\r\n"
        "  ESP PING\r\n"
        "    Simple ping to ESP32-C6; checks if UART link is alive.\r\n"
        "\r\n"
        "  ESP STATUS\r\n"
        "    Ask ESP32-C6 for current status (ONLINE/OFFLINE) and cache it.\r\n"
        "\r\n"
        "  ESP VERSION\r\n"
        "    Print firmware/version info reported by ESP32-C6.\r\n"
        "\r\n"
        "  ESP QR\r\n"
        "    Request Matter COMM QR + MANUAL code from ESP32-C6,\r\n"
        "    store into g_esp.pairing.qr/manual and draw QR on E-Ink (if connected).\r\n"
        "\r\n"
        "  ESP FABRICS\r\n"
        "    List fabrics / commissioners stored in ESP32-C6.\r\n"
        "\r\n"
        "  ESP FABRIC REMOVE <idx>\r\n"
        "    Remove fabric with given index (from ESP32-C6 list).\r\n"
        "\r\n"
        "  ESP COMM START\r\n"
        "    Start ESP32-C6 commissioning/COMM session (e.g. Matter pairing window).\r\n"
        "    Also tries to extract COMM QR/MANUAL from response and draw QR on E-Ink.\r\n"
        "\r\n"
        "  ESP COMM STOP\r\n"
        "    Stop ESP32-C6 commissioning/COMM session.\r\n"
        "\r\n"
        "  ESP FACTORYRESET\r\n"
        "    Full factory reset on ESP32-C6 (clears fabrics/commissioning info).\r\n"
        "\r\n"
        "  ESP SEND HEX <12hex>\r\n"
        "    Send 6-byte payload given as 12 hex chars over ESP (Matter text frame).\r\n"
        "\r\n"
        "  ESP SEND KV  T=<i16> RH=<u16> CO2=<u16>\r\n"
        "    Build payload from T/RH/CO2 values and send via ESP as text frame.\r\n"
        "\r\n"
        "Wio-E5 (LoRaWAN) commands:\r\n"
        "  WIO SHOW\r\n"
        "    Show current OTAA config: AppEUI, DevEUI, AppKey, ADR flag.\r\n"
        "\r\n"
        "  WIO SET APP_EUI <16hex>\r\n"
        "    Set OTAA AppEUI (16 hex chars, no spaces).\r\n"
        "\r\n"
        "  WIO SET DEV_EUI <16hex>\r\n"
        "    Set OTAA DevEUI (16 hex chars, no spaces).\r\n"
        "\r\n"
        "  WIO SET APP_KEY <32hex>\r\n"
        "    Set OTAA AppKey (32 hex chars, no spaces).\r\n"
        "\r\n"
        "  WIO SET ADR <0|1>\r\n"
        "    Set Adaptive Data Rate: 0 = OFF, 1 = ON.\r\n"
        "\r\n"
        "  WIO JOIN\r\n"
        "    Perform OTAA join now using current keys (g_wio).\r\n"
        "\r\n"
        "  WIO SEND  HEX <12hex>\r\n"
        "    Send 6-byte payload (12 hex chars) as CONFIRMED uplink.\r\n"
        "    Returns OK(ACK) / OK(NOACK) / ERR.\r\n"
        "\r\n"
        "  WIO SEND  KV  T=<i16> RH=<u16> CO2=<u16>\r\n"
        "    Build 6-byte payload from T/RH/CO2 and send as CONFIRMED uplink.\r\n"
        "\r\n"
        "  WIO SENDU HEX <12hex>\r\n"
        "    Same as WIO SEND HEX, but UNCONFIRMED (no ACK expected).\r\n"
        "\r\n"
        "  WIO SENDU KV  T=<i16> RH=<u16> CO2=<u16>\r\n"
        "    Same as WIO SEND KV, but UNCONFIRMED uplink.\r\n"
        "\r\n"
        "  WIO TIME\r\n"
        "    Request network time via LoRaWAN downlink and set MCU RTC if received.\r\n"
        "\r\n"
        "EEPROM config commands:\r\n"
        "  RESET\r\n"
        "    Restore device-level config (AppConfig) to firmware defaults in RAM.\r\n"
        "    LoRa credentials (g_wio) and ESP pairing (QR/MANUAL) are kept in RAM.\r\n"
        "    Changes are NOT saved to EEPROM until SAVE is used.\r\n"
        "\r\n"
        "  SAVE\r\n"
        "    Store current AppConfig, Wio OTAA keys and ESP pairing (QR/MANUAL)\r\n"
        "    into M24C02 EEPROM. Next cold boot will load these values.\r\n"
        "\r\n"
    );
}

/**
 * @brief Set a single key/value pair in AppConfig from a console command.
 *
 * Supports short aliases (mm, voc, comms, tm, ts, tt, tb, txmin, txmax, dT, dRH, dCO2).
 *
 * @return 1 on success, 0 on invalid key or value.
 */
static int set_kv(const char* key, const char* val) {
    uint32_t u32; uint16_t u16; uint8_t u8;

    if (keyeq(key,"measure_mode") || keyeq(key,"mm")) {
        if (!parse_u8(val, &u8) || u8 > MEAS_TRH_ONLY) return 0;
        s_cfg->measureMode = (MeasureMode_t)u8; return 1;
    }
    if (keyeq(key,"voc_mode") || keyeq(key,"voc")) {
        if (!parse_u8(val, &u8) || u8 > VOC_CONT_USB) return 0;
        s_cfg->vocMode = (VocMode_t)u8; return 1;
    }
    if (keyeq(key,"comms_mode") || keyeq(key,"comms")) {
        if (!parse_u8(val, &u8) || u8 > COMMS_LORA) return 0;
        s_cfg->commsMode = (CommsMode_t)u8;
        return 1;
    }
    if (keyeq(key,"interval_measure_sec") || keyeq(key,"tm")) { if (!parse_u32(val, &u32)) return 0; s_cfg->interval_measure_sec = u32; return 1; }
    if (keyeq(key,"interval_sleep_sec")   || keyeq(key,"ts")) { if (!parse_u32(val, &u32)) return 0; s_cfg->interval_sleep_sec   = u32; return 1; }
    if (keyeq(key,"interval_time_req_sec")|| keyeq(key,"tt")) { if (!parse_u32(val, &u32)) return 0; s_cfg->interval_time_req_sec= u32; return 1; }
    if (keyeq(key,"interval_bat_sec")     || keyeq(key,"tb")) { if (!parse_u32(val, &u32)) return 0; s_cfg->interval_bat_sec     = u32; return 1; }

    if (keyeq(key,"tx_min_interval_sec")  || keyeq(key,"txmin")) { if (!parse_u32(val, &u32)) return 0; s_cfg->tx_min_interval_sec = u32; return 1; }
    if (keyeq(key,"tx_max_interval_sec")  || keyeq(key,"txmax")) { if (!parse_u32(val, &u32)) return 0; s_cfg->tx_max_interval_sec = u32; return 1; }

    if (keyeq(key,"th_temp_01C")          || keyeq(key,"dT"))  { if (!parse_u16(val, &u16)) return 0; s_cfg->th_temp_01C = u16; return 1; }
    if (keyeq(key,"th_rh_01pct")          || keyeq(key,"dRH")) { if (!parse_u16(val, &u16)) return 0; s_cfg->th_rh_01pct = u16; return 1; }
    if (keyeq(key,"th_co2_ppm")           || keyeq(key,"dCO2")){ if (!parse_u16(val, &u16)) return 0; s_cfg->th_co2_ppm = u16; return 1; }

    return 0;
}

/* ================= Command handler ================= */

/**
 * @brief Print current Wio-E5 OTAA configuration over the console.
 */
static void wio_show(void)
{
    char line[256];
    snprintf(line, sizeof(line),
        "WIO SHOW:\r\n"
        "  APP_EUI: %s\r\n"
        "  DEV_EUI: %s\r\n"
        "  APP_KEY: %s\r\n"
        "  ADR    : %u\r\n",
        g_wio.app_eui[0]?g_wio.app_eui:"<empty>",
        g_wio.dev_eui[0]?g_wio.dev_eui:"<empty>",
        g_wio.app_key[0]?g_wio.app_key:"<empty>",
        (unsigned)g_wio.adr_on);
    tx(line);
}

/**
 * @brief Handle "WIO SET ..." subcommands to modify OTAA credentials and ADR.
 */
static void wio_set(const char* key, const char* val)
{
    if (!key || !val) { tx("ERR: WIO SET <APP_EUI|DEV_EUI|APP_KEY|ADR> <value>\r\n"); return; }

    if (keyeq(key,"APP_EUI")) {
        if (!Wio_IsHexLen(val, 16)) { tx("ERR: APP_EUI must be 16 hex chars\r\n"); return; }
        strncpy(g_wio.app_eui, val, sizeof(g_wio.app_eui));
        g_wio.app_eui[sizeof(g_wio.app_eui)-1] = '\0';
        tx("OK\r\n");
        return;
    }
    if (keyeq(key,"DEV_EUI")) {
        if (!Wio_IsHexLen(val, 16)) { tx("ERR: DEV_EUI must be 16 hex chars\r\n"); return; }
        strncpy(g_wio.dev_eui, val, sizeof(g_wio.dev_eui));
        g_wio.dev_eui[sizeof(g_wio.dev_eui)-1] = '\0';
        tx("OK\r\n");
        return;
    }
    if (keyeq(key,"APP_KEY")) {
        if (!Wio_IsHexLen(val, 32)) { tx("ERR: APP_KEY must be 32 hex chars\r\n"); return; }
        strncpy(g_wio.app_key, val, sizeof(g_wio.app_key));
        g_wio.app_key[sizeof(g_wio.app_key)-1] = '\0';
        tx("OK\r\n");
        return;
    }
    if (keyeq(key,"ADR")) {
        uint32_t u=2;
        if (!parse_u32(val, &u) || u>1) { tx("ERR: ADR must be 0 or 1\r\n"); return; }
        g_wio.adr_on = (uint8_t)u;
        tx("OK\r\n");
        return;
    }

    tx("ERR: WIO SET unknown key\r\n");
}

/**
 * @brief Parse and execute a single console command line.
 */
static void handle_line(const char* line)
{
    char buf[128];
    strncpy(buf, line, sizeof(buf)-1); buf[sizeof(buf)-1] = '\0';

    char* cmd = strtok(buf, " \t");
    if (!cmd) return;

    if (keyeq(cmd,"HELP") || keyeq(cmd,"?"))          { help(); return; }
    if (keyeq(cmd,"SHOW") || keyeq(cmd,"GET"))        { ConfConsole_PrintCfg(); return; }

    if (keyeq(cmd,"SET")) {
        char* k = strtok(NULL, " \t"); char* v = strtok(NULL, " \t");
        if (!k || !v) { tx("\r\nERR: usage SET <key> <value>\r\n"); return; }
        tx(set_kv(k, v) ? "\r\nOK\r\n" : "\r\nERR: bad key or value\r\n");
        return;
    }

    if (keyeq(cmd,"EXIT") || keyeq(cmd,"QUIT")) {
        s_wants_exit = 1; tx("OK, exiting\r\n"); return;
    }

    if (keyeq(cmd,"JOIN")) {
        tx(Wio_JoinNow(30000) ? "OK: joined\r\n" : "ERR: join failed\r\n");
        return;
    }

    /* ========================= ESP32-C6 commands ========================= */
    if (keyeq(cmd,"ESP")) {
        char* sub = strtok(NULL, " \t");
        if (!sub) { tx("ERR: ESP <subcmd>\r\n"); return; }

        uint8_t ok = 0;

        /* Helper inline extractor for QR/MANUAL from esp_rx_buffer (local to this handler) */
        auto void extract_qr_from_buffer(void) {
            const char *pqr = strstr(esp_rx_buffer, "COMM QR ");
            if (pqr) {
                pqr += 8;
                size_t n = 0;
                while (pqr[n] && pqr[n] != '\r' && pqr[n] != '\n' && n + 1 < sizeof(g_esp.pairing.qr)) n++;
                memcpy(g_esp.pairing.qr, pqr, n); g_esp.pairing.qr[n] = '\0';
            }
            const char *pmn = strstr(esp_rx_buffer, "COMM MANUAL ");
            if (pmn) {
                pmn += 12;
                size_t n = 0;
                while (pmn[n] && pmn[n] != '\r' && pmn[n] != '\n' && n + 1 < sizeof(g_esp.pairing.manual)) n++;
                memcpy(g_esp.pairing.manual, pmn, n); g_esp.pairing.manual[n] = '\0';
            }
        };

        if      (keyeq(sub,"PING"))          ok = EspC6_Ping();
        else if (keyeq(sub,"STATUS"))        ok = EspC6_Status();
        else if (keyeq(sub,"VERSION"))       ok = EspC6_Version();
        else if (keyeq(sub,"QR")) {
            ok = EspC6_QR();                 /* fills g_esp.pairing.qr and g_esp.pairing.manual when available */
            /* Immediately draw QR on E-Ink if connected and data present */
            if (ok && g_esp.pairing.qr[0] && app.epd_link == EPD_LINK_CONNECTED) {
                GUI_DrawQR_TextFull(g_esp.pairing.qr, g_esp.pairing.manual[0] ? g_esp.pairing.manual : NULL);
            }
        }
        else if (keyeq(sub,"FABRICS"))	{
        	ok = EspC6_Fabrics();
        }
        else if (keyeq(sub,"FACTORYRESET")) {
            ok = EspC6_FactoryReset();
        }
        else if (keyeq(sub,"COMM")) {
            char* what = strtok(NULL, " \t");
            if (!what) { tx("ERR: ESP COMM <START|STOP>\r\n"); return; }

            if (keyeq(what,"START")) {
                ok = EspC6_CommStart();

                /* 1) Try to extract QR/MANUAL from the latest response */
                extract_qr_from_buffer();

                /* 2) If QR is still empty, request it explicitly */
                if (!g_esp.pairing.qr[0]) {
                    (void)EspC6_QR(); /* fills g_esp.pairing.qr / g_esp.pairing.manual if available */
                }

                /* 3) If we have a QR, show it on E-Ink */
                if (app.epd_link == EPD_LINK_CONNECTED && g_esp.pairing.qr[0]) {
                    GUI_DrawQR_TextFull(g_esp.pairing.qr, g_esp.pairing.manual[0] ? g_esp.pairing.manual : NULL);
                }
            }
            else if (keyeq(what,"STOP")) {
                ok = EspC6_CommStop();
            }
            else {
                tx("ERR: ESP COMM <START|STOP>\r\n");
                return;
            }
        }
        else if (keyeq(sub,"SEND")) {
            char* which = strtok(NULL, " \t");
            if (!which) { tx("ERR: ESP SEND <HEX|KV> ...\r\n"); return; }
            if (keyeq(which,"HEX")) {
                char* hex = strtok(NULL, " \t");
                if (!hex || strlen(hex)!=12) { tx("ERR: HEX needs 12 hex chars\r\n"); return; }
                uint8_t p6[6];
                for (int i=0;i<6;i++){ char b[3]={hex[i*2],hex[i*2+1],0}; p6[i]=(uint8_t)strtoul(b,NULL,16); }
                ok = EspC6_DataHex(p6);
            } else if (keyeq(which,"KV")) {
                char* t = strtok(NULL," \t"); char* rh = strtok(NULL," \t"); char* co2 = strtok(NULL," \t");
                int tt=0; unsigned rr=0, cc=0;
                if (!t || !rh || !co2 || sscanf(t,"T=%d",&tt)!=1 || sscanf(rh,"RH=%u",&rr)!=1 || sscanf(co2,"CO2=%u",&cc)!=1) {
                    tx("ERR: usage ESP SEND KV T=<i16> RH=<u16> CO2=<u16>\r\n"); return;
                }
                ok = EspC6_DataKV((int16_t)tt,(uint16_t)rr,(uint16_t)cc);
            } else {
                tx("ERR: ESP SEND <HEX|KV>\r\n"); return;
            }
        }
        else {
            tx("ERR: unknown ESP subcommand\r\n");
            return;
        }

        /* Consolidated console output (same pattern as before) */
        tx("ESP Response:\r\n");
        tx(esp_rx_buffer);
        if (esp_rx_buffer[0] && esp_rx_buffer[strlen(esp_rx_buffer)-1] != '\n') tx("\r\n");
        tx(ok ? "OK\r\n" : "ERR\r\n");
        return;
    }

    /* ========================== Wio-E5 commands ========================== */
    if (keyeq(cmd,"WIO")) {
        char* sub = strtok(NULL, " \t");
        if (!sub) { tx("ERR: WIO <subcmd>\r\n"); return; }

        if (keyeq(sub,"SHOW")) { wio_show(); return; }

        if (keyeq(sub,"SET")) {
            char* wkey = strtok(NULL, " \t");
            char* wval = strtok(NULL, " \t");
            wio_set(wkey, wval);
            return;
        }

        if (keyeq(sub,"JOIN")) {
            tx(Wio_JoinNow(30000) ? "OK: joined\r\n" : "ERR: join failed\r\n");
            return;
        }

        if (keyeq(sub,"TIME")) {
            tx(WioTimeReq() ? "OK\r\n" : "ERR\r\n");
            return;
        }

        if (keyeq(sub,"SEND") || keyeq(sub,"SENDU")) {
            const uint8_t unconfirmed = keyeq(sub,"SENDU") ? 1 : 0;

            char* which = strtok(NULL, " \t");
            if (!which) { tx("ERR: WIO SEND[U] <HEX|KV> ...\r\n"); return; }

            if (keyeq(which,"HEX")) {
                char* hex = strtok(NULL, " \t");
                if (!hex || strlen(hex)!=12) { tx("ERR: HEX needs 12 hex chars\r\n"); return; }

                uint8_t payload[6];
                for (int i=0;i<6;i++){ char b[3]={hex[i*2],hex[i*2+1],0}; payload[i]=(uint8_t)strtoul(b,NULL,16); }

                if (unconfirmed) {
                    WioSendUnconfResult_t r = Wio_SendData_Unconfirmed(payload, sizeof(payload));
                    tx((r == WIO_SEND_OK) ? "OK\r\n" : "ERR\r\n");
                } else {
                    WioSendResult_t r = Wio_SendData(payload, sizeof(payload));
                    if      (r == WIO_SEND_ACK_RECEIVED) tx("OK (ACK)\r\n");
                    else if (r == WIO_SEND_NO_ACK)       tx("OK (NOACK)\r\n");
                    else                                 tx("ERR\r\n");
                }
                return;
            }

            if (keyeq(which,"KV")) {
                char* t  = strtok(NULL," \t");
                char* rh = strtok(NULL," \t");
                char* c2 = strtok(NULL," \t");
                int tt=0; unsigned rr=0, cc=0;
                if (!t || !rh || !c2 ||
                    sscanf(t, "T=%d",     &tt) != 1 ||
                    sscanf(rh,"RH=%u",    &rr) != 1 ||
                    sscanf(c2,"CO2=%u",   &cc) != 1) {
                    tx("ERR: usage WIO SEND[U] KV T=<i16> RH=<u16> CO2=<u16>\r\n");
                    return;
                }

                uint8_t payload[6];
                build_payload_from_kv((int16_t)tt, (uint16_t)rr, (uint16_t)cc, payload);

                if (unconfirmed) {
                    WioSendUnconfResult_t r = Wio_SendData_Unconfirmed(payload, sizeof(payload));
                    tx((r == WIO_SEND_OK) ? "OK\r\n" : "ERR\r\n");
                } else {
                    WioSendResult_t r = Wio_SendData(payload, sizeof(payload));
                    if      (r == WIO_SEND_ACK_RECEIVED) tx("OK (ACK)\r\n");
                    else if (r == WIO_SEND_NO_ACK)       tx("OK (NOACK)\r\n");
                    else                                 tx("ERR\r\n");
                }
                return;
            }

            tx("ERR: WIO SEND[U] <HEX|KV>\r\n");
            return;
        }

        tx("ERR: unknown WIO subcommand\r\n");
        return;
    }

    if (keyeq(cmd,"RESET") || keyeq(cmd,"DEFAULTS")) {
        AppConfig_SetDefaults(s_cfg);
        tx("OK: config reset to firmware defaults (LoRa/Matter data unchanged, not saved yet)\r\n");
        return;
    }

    if (keyeq(cmd,"SAVE")) {
        App_Config_SaveToEeprom();
        tx("OK: config + WIO OTAA + ESP pairing saved to EEPROM\r\n");
        return;
    }



    tx("ERR: unknown command (try HELP)\r\n");
}


/* ================= ISRs & polling ================= */

/**
 * @brief Poll for a pending command and execute it.
 *
 * Must be called from the main loop while in CONFIG state. It processes one
 * complete line at a time that has been accumulated by ConfConsole_OnRxCplt().
 */
void ConfConsole_Poll(void) {
    if (!s_active) return;
    if (!s_cmd_ready) return;

    __disable_irq();
    char line[sizeof(s_cmd)];
    strncpy(line, s_cmd, sizeof(line));
    s_cmd_ready = 0;
    __enable_irq();

    handle_line(line);  // safely outside ISR
}

/**
 * @brief UART RX complete hook that feeds incoming characters into the console.
 *
 * Accumulates characters into a line buffer until CR/LF is received, then
 * marks a command as ready for ConfConsole_Poll() to process.
 */
void ConfConsole_OnRxCplt(UART_HandleTypeDef* huart)
{
    if (!s_active || huart != s_huart) return;

    uint8_t c = s_rx_ch;
    touch();

    if (c == '\r' || c == '\n') {
        if (s_len > 0 && !s_cmd_ready) {
            size_t n = (s_len < sizeof(s_cmd)-1) ? s_len : sizeof(s_cmd)-1;
            memcpy(s_cmd, s_line, n);
            s_cmd[n] = '\0';
            s_len = 0;
            s_cmd_ready = 1;     // only raise the flag; no TX/Delay inside ISR
        }
    } else {
        if (s_len + 1 < sizeof(s_line)) s_line[s_len++] = (char)c;
        else { s_len = 0; /* optional: signal overflow */ }
    }

    HAL_UART_Receive_IT(s_huart, (uint8_t*)&s_rx_ch, 1);
}

/**
 * @brief Check if the console should exit due to EXIT command or inactivity.
 *
 * @return 1 if console should exit CONFIG mode, 0 otherwise.
 */
uint8_t ConfConsole_ShouldExit(void)
{
    if (!s_active) return 0;
    if (s_wants_exit) return 1;
    if (!s_sod_now) return 0;
    uint32_t now = s_sod_now();
    return ((now - s_last_act_s) >= s_inact_s) ? 1 : 0;
}
