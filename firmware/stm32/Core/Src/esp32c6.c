/**
 * @file esp32c6.c
 * @brief UART/handshake client for ESP32-C6 text protocol (no init; uses main.h symbols).
 * @author Evzen Steif
 *
 * Core responsibilities:
 *  - Drive WAKE/READY handshake lines to the ESP32-C6.
 *  - Send ASCII commands over UART and collect responses into esp_rx_buffer.
 *  - Support token-based matching (e.g. "OK|ERROR|PONG") with a sliding timeout.
 *  - Provide small high-level helpers for PING/STATUS/QR/COMM/... commands.
 */

#include "esp32c6.h"
#include <string.h>
#include <stdio.h>
#include "main.h"  /* for huart1, RF_GPIO1/2_* */

/* Public RX buffer (similar to wio_rx_buffer). */
char esp_rx_buffer[512];
extern UART_HandleTypeDef huart1;

/* Global context with sensible defaults. */
EspC6Context_t g_esp = {
    .ready_wait_ms = ESPC6_READY_WAIT_MS,
    .def_timeout_ms = ESPC6_DEF_TIMEOUT_MS,
    .success_without_tokens = 1,
    .drain_silence_ms = 60,   // recommended: 50–100 ms
    .drain_cap_ms     = 400,  // recommended: 300–800 ms
    .status = ESP_STATUS_UNKNOWN,
    .pairing.qr = {0},
    .pairing.manual = {0},
};

/* --- Small helpers bound to main.h pins --- */
static inline void wake_high(void) { HAL_GPIO_WritePin(RF_GPIO1_GPIO_Port, RF_GPIO1_Pin, GPIO_PIN_SET); }
static inline void wake_low(void)  { HAL_GPIO_WritePin(RF_GPIO1_GPIO_Port, RF_GPIO1_Pin, GPIO_PIN_RESET); }

/**
 * @brief Wait until READY pin reaches the desired level or timeout expires.
 *
 * @param level      GPIO_PIN_SET or GPIO_PIN_RESET to wait for.
 * @param timeout_ms Maximum time to wait in milliseconds.
 * @return 1 if level was reached within timeout, 0 on timeout.
 */
static uint8_t wait_ready_level(GPIO_PinState level, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (HAL_GPIO_ReadPin(RF_GPIO2_GPIO_Port, RF_GPIO2_Pin) == level) return 1;
        HAL_Delay(1);
    }
    return 0;
}

/* ======================= ESP helper ======================= */
/* WAKE = RF_GPIO1, READY = RF_GPIO2; RF_GPIO0 is unused. */

/**
 * @brief Initialize WAKE/READY pins and bring ESP32-C6 into enabled state.
 *
 * Configures:
 *   - READY (RF_GPIO2) as input with pull-down (or NOPULL, depending on HW).
 *   - WAKE  (RF_GPIO1) as push-pull output, default LOW.
 *   - RF_RST as HIGH if it serves as enable for the RF block.
 *
 * UART is assumed to be configured elsewhere (huart1).
 */
void ESP_Init(void)
{
    /* 2) READY as input (pull-down or NOPULL per HW design) */
    HAL_GPIO_DeInit(RF_GPIO2_GPIO_Port, RF_GPIO2_Pin);
    GPIO_InitTypeDef gi = {0};
    gi.Pin   = RF_GPIO2_Pin;
    gi.Mode  = GPIO_MODE_INPUT;
    gi.Pull  = GPIO_PULLDOWN;          /* or GPIO_NOPULL per HW */
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RF_GPIO2_GPIO_Port, &gi);

    /* 3) WAKE as push-pull output, default LOW */
    HAL_GPIO_DeInit(RF_GPIO1_GPIO_Port, RF_GPIO1_Pin);
    GPIO_InitTypeDef go = {0};
    go.Pin   = RF_GPIO1_Pin;
    go.Mode  = GPIO_MODE_OUTPUT_PP;
    go.Pull  = GPIO_NOPULL;
    go.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RF_GPIO1_GPIO_Port, &go);
    HAL_GPIO_WritePin(RF_GPIO1_GPIO_Port, RF_GPIO1_Pin, GPIO_PIN_RESET); /* WAKE=LOW */

    /* 4) Optionally: if RF_RST is used as EN, pull it HIGH */
    HAL_GPIO_WritePin(RF_RST_GPIO_Port, RF_RST_Pin, GPIO_PIN_SET);
}

/**
 * @brief Core send/receive with WAKE/READY handshake and token scanning.
 *
 * Behavior:
 *  - Pulses WAKE high, waits for READY high (up to g_esp.ready_wait_ms).
 *  - Sends optional ASCII command (with CRLF appended).
 *  - Reads bytes with a sliding timeout (resets after each received byte).
 *  - If a non-empty token list is provided, scans esp_rx_buffer with strstr()
 *    for any pipe-separated token and treats a match as success.
 *  - After the first token match, it waits a small "post-match" delay to
 *    gather remaining response bytes, then returns.
 *
 * Design notes:
 *  - We always keep esp_rx_buffer NUL-terminated, so it is a valid C-string
 *    at all times and safe for strstr().
 *  - Sliding timeout ensures that slow or bursty output does not prematurely
 *    trigger a timeout as long as data keeps coming.
 *
 * @param cmd       ASCII command WITHOUT CRLF (may be NULL to "read only").
 * @param responses Pipe-separated success tokens, e.g. "OK|READY|PONG".
 *                  If NULL or empty, no token check is performed and success
 *                  is controlled by g_esp.success_without_tokens.
 * @param timeout_ms Sliding timeout window in milliseconds.
 * @return 1 on success (token matched, or no tokens and success_without_tokens == 1), 0 on failure.
 */
uint8_t EspC6_SendCommand(const char *cmd, const char *responses, uint32_t timeout_ms)
{
    memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));
    uint8_t  temp_byte;
    uint16_t index = 0;

    const uint32_t t_start = HAL_GetTick();
    uint32_t       t_last  = t_start;   // for sliding timeout

    uint8_t  matched    = 0;
    uint32_t t_matched  = 0;
    const uint32_t postmatch_ms = 150;  // slightly longer "post-breath" after first token match

    char matched_token[64] = {0};       // for logging which token was matched

    // 1) Start transaction: wake ESP up
    wake_high();

    // 2) Wait for READY high
    if (!wait_ready_level(GPIO_PIN_SET, g_esp.ready_wait_ms)) {
        wake_low();
        printf("ESP: READY timeout\r\n");
        return 0;
    }

    // 3) Transmit optional command
    if (cmd && cmd[0]) {
        HAL_UART_Transmit(&huart1, (uint8_t*)cmd, (uint16_t)strlen(cmd), HAL_MAX_DELAY);
        static const uint8_t crlf[2] = {'\r','\n'};
        HAL_UART_Transmit(&huart1, (uint8_t*)crlf, 2, HAL_MAX_DELAY);
    }

    // 4) RX loop: sliding timeout or match + post-match delay
    for (;;) {
        uint32_t now = HAL_GetTick();

        // sliding timeout window: measured from last received byte
        if ((now - t_last) >= timeout_ms) break;

        // once we have a match, wait a short post-match period, then stop
        if (matched && (now - t_matched) >= postmatch_ms) break;

        if (HAL_UART_Receive(&huart1, &temp_byte, 1, 20) == HAL_OK) {
            if (index < sizeof(esp_rx_buffer) - 1) {
                esp_rx_buffer[index++] = temp_byte;
                esp_rx_buffer[index]   = '\0';
            }
            t_last = now;  // extend timeout window

            if (!matched && responses && responses[0]) {
                char resp_copy[128];
                strncpy(resp_copy, responses, sizeof(resp_copy) - 1);
                resp_copy[sizeof(resp_copy) - 1] = '\0';

                char *token = strtok(resp_copy, "|");
                while (token) {
                    // trim leading spaces inside token (if any)
                    while (*token == ' ') token++;
                    size_t tn = strlen(token);
                    while (tn && token[tn-1] == ' ') token[--tn] = '\0';

                    if (strstr(esp_rx_buffer, token)) {
                        matched = 1;
                        t_matched = now;
                        // remember exact token for logging
                        strncpy(matched_token, token, sizeof(matched_token)-1);
                        matched_token[sizeof(matched_token)-1] = '\0';
                        break;
                    }
                    token = strtok(NULL, "|");
                }
            }
        }
    }

    // 5) End transaction: lower WAKE
    wake_low();

    // 6) Short guard period – read any trailing bytes that arrive right after WAKE low
    {
        const uint32_t guard_ms = 3;
        uint32_t t0 = HAL_GetTick();
        while ((HAL_GetTick() - t0) < guard_ms) {
            if (HAL_UART_Receive(&huart1, &temp_byte, 1, 1) == HAL_OK) {
                if (index < sizeof(esp_rx_buffer) - 1) {
                    esp_rx_buffer[index++] = temp_byte;
                    esp_rx_buffer[index]   = '\0';
                }
            }
        }
    }

    // 7) Friendly debug log to console
    const uint32_t elapsed = HAL_GetTick() - t_start;
    if (responses && responses[0]) {
        if (matched) {
            printf("ESP: matched '%s' in %lums (%u bytes)\r\n",
                   matched_token[0] ? matched_token : "<token>",
                   (unsigned long)elapsed, (unsigned)index);
        } else {
            printf("ESP: TIMEOUT waiting for '%s' after %lums (%u bytes)\r\n",
                   responses, (unsigned long)elapsed, (unsigned)index);
        }
    } else {
        printf("ESP: no-token mode, elapsed %lums (%u bytes)\r\n",
               (unsigned long)elapsed, (unsigned)index);
    }

    // 8) Return OK/ERR depending on token match (or no-token policy)
    if (responses && responses[0]) {
        return matched ? 1 : 0;
    } else {
        return g_esp.success_without_tokens ? 1 : 0;
    }
}


/* ===== High-level wrappers ===== */

/**
 * @brief Send PING and expect PONG; quick liveness check.
 */
uint8_t EspC6_Ping(void)
{
    return EspC6_SendCommand("PING", "PONG", g_esp.def_timeout_ms);
}

/**
 * @brief Query ESP32-C6 status (ONLINE/OFFLINE) and update g_esp.status.
 *
 * Expected response examples:
 *   - "STATUS ONLINE role=SED"
 *   - "STATUS OFFLINE"
 */
uint8_t EspC6_Status(void)
{
    uint8_t ok = EspC6_SendCommand("STATUS?", "STATUS ONLINE|STATUS OFFLINE", g_esp.def_timeout_ms);
    if (ok) {
        if      (strstr(esp_rx_buffer, "STATUS ONLINE"))  g_esp.status = ESP_STATUS_ONLINE;
        else if (strstr(esp_rx_buffer, "STATUS OFFLINE")) g_esp.status = ESP_STATUS_OFFLINE;
        else                                              g_esp.status = ESP_STATUS_UNKNOWN;
    }
    return ok;
}

/**
 * @brief Request version information from ESP32-C6.
 *
 * Example response: "VERSION app=1.0.0 idf=... matter=..."
 */
uint8_t EspC6_Version(void)
{
    return EspC6_SendCommand("VERSION?", "VERSION ", g_esp.def_timeout_ms);
}

/**
 * @brief Request Matter QR and manual code and cache them into g_esp.pairing.
 *
 * Device typically replies with lines like:
 *   "COMM QR <base38>"
 *   "COMM MANUAL <decimal>"
 *
 * Both QR and manual fields are extracted if present.
 */
uint8_t EspC6_QR(void)
{
    uint8_t ok = EspC6_SendCommand("QR?", "COMM QR", g_esp.def_timeout_ms);
    if (ok) {
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
    return ok;
}

/**
 * @brief List fabrics / commissioners stored in ESP32-C6.
 *
 * The console can parse content between "FABRICS BEGIN" and "FABRICS END".
 */
uint8_t EspC6_Fabrics(void)
{
    return EspC6_SendCommand("FABRICS?", "FABRICS END", 20000);
}

/**
 * @brief Start a commissioning (COMM) session on ESP32-C6.
 *
 * Typically opens a Matter pairing window and returns COMM-related info.
 */
uint8_t EspC6_CommStart(void)
{
    return EspC6_SendCommand("COMM START", "COMM MANUAL", 5000);
}

/**
 * @brief Stop an ongoing commissioning (COMM) session.
 */
uint8_t EspC6_CommStop(void)
{
    return EspC6_SendCommand("COMM STOP", "COMM CLOSED|COMM STATE Stopped", 5000);
}

/**
 * @brief Perform a factory reset on ESP32-C6 (clears fabrics / commissioning data).
 */
uint8_t EspC6_FactoryReset(void)
{
    return EspC6_SendCommand("FACTORYRESET", "OK", 1000);
}

/* === Data helpers === */

/**
 * @brief Send a 6-byte payload encoded as 12 hex characters via ESP (Matter data frame).
 *
 * Uses "DATA HEX <12hex>" format. ESP usually does not reply to DATA frames, so
 * token validation is disabled and only a short timeout is used.
 *
 * @param payload6 Pointer to 6 bytes of data.
 * @return 1 on successful send, 0 on UART / handshake failure.
 */
uint8_t EspC6_DataHex(const uint8_t payload6[6])
{
    char hex[13];
    static const char HX[] = "0123456789ABCDEF";
    for (int i = 0; i < 6; ++i) {
        hex[i*2+0] = HX[(payload6[i] >> 4) & 0xF];
        hex[i*2+1] = HX[(payload6[i]     ) & 0xF];
    }
    hex[12] = '\0';

    char line[32];
    snprintf(line, sizeof(line), "DATA HEX %s", hex);

    /* DATA usually gets no reply; don’t require tokens. */
    return EspC6_SendCommand(line, NULL, 800);
}

/**
 * @brief Send climate payload (T/RH/CO2) as key-value text line via ESP.
 *
 * Format:
 *   "DATA KV T=<i16> RH=<u16> CO2=<u16>"
 *
 * @param t_01C   Temperature in 0.01 °C.
 * @param rh_01pct Relative humidity in 0.01 %RH.
 * @param co2_ppm CO2 concentration in ppm.
 * @return 1 on successful send, 0 on error.
 */
uint8_t EspC6_DataKV(int16_t t_01C, uint16_t rh_01pct, uint16_t co2_ppm)
{
    char line[64];
    snprintf(line, sizeof(line), "DATA KV T=%d RH=%u CO2=%u",
             (int)t_01C, (unsigned)rh_01pct, (unsigned)co2_ppm);
    return EspC6_SendCommand(line, NULL, 800);
}

/**
 * @brief Send battery status to ESP32-C6 (for Matter Power Source or telemetry).
 *
 * Format:
 *   "DATA BAT KV LI=<mV> LS=<mV> USB=<0/1> CHG=<0..255>"
 *
 * @param vbat_liion_mV   Li-Ion battery voltage in millivolts.
 * @param vbat_lsocl2_mV  SOCl2 battery voltage in millivolts.
 * @param usb_on          1 if USB is present, otherwise 0.
 * @param charger_status  BQ25185 charger status, passed as an opaque 0..255 value.
 * @return 1 on successful send, 0 on error.
 */
uint8_t EspC6_DataBat(uint16_t vbat_liion_mV,
                      uint16_t vbat_lsocl2_mV,
                      uint8_t usb_on,
                      uint8_t charger_status)
{
    /* DATA BAT KV LI=<mV> LS=<mV> USB=<0/1> CHG=<0..255> */
    char line[96];
    snprintf(line, sizeof(line),
             "DATA BAT KV LI=%u LS=%u USB=%u CHG=%u",
             (unsigned)vbat_liion_mV,
             (unsigned)vbat_lsocl2_mV,
             (unsigned)usb_on,
             (unsigned)charger_status);

    /* Same approach as DATA HEX: no token expectations, short timeout. */
    return EspC6_SendCommand(line, NULL, 800);
}
