#pragma once
/**
 * @file esp32c6.h
 * @brief Simple UART + WAKE/READY helpers for ESP32-C6 “text protocol” (no init needed).
 * @author Evzen Steif
 *
 * Hardware binding is taken directly from main.h:
 *   - UART:         extern UART_HandleTypeDef huart1;
 *   - WAKE  -> ESP: RF_GPIO1_GPIO_Port / RF_GPIO1_Pin
 *   - READY <- ESP: RF_GPIO2_GPIO_Port / RF_GPIO2_Pin
 *
 * This mirrors the style of WioE5.c:
 *   - One EspC6_SendCommand() core with token list support.
 *   - A few high-level wrappers (PING/STATUS/QR/COMM/...) for convenience.
 */

#include <stdint.h>
#include "stm32u0xx_hal.h"

#define ESP_QR_MAX_LEN     48
#define ESP_MANUAL_MAX_LEN 16

/* Public RX buffer (like wio_rx_buffer) so console can print captured text. */
extern char esp_rx_buffer[512];

/* Default timings (you can tweak g_esp.* after link). */
#ifndef ESPC6_READY_WAIT_MS
#define ESPC6_READY_WAIT_MS   600u   /**< Wait for READY rising edge after WAKE is asserted. */
#endif

#ifndef ESPC6_DEF_TIMEOUT_MS
#define ESPC6_DEF_TIMEOUT_MS  1500u  /**< Default sliding timeout while reading UART (ms). */
#endif

/**
 * @brief High-level connection status cached from last STATUS/QR call.
 */
typedef enum {
    ESP_STATUS_UNKNOWN = 0,
    ESP_STATUS_OFFLINE = 1,
    ESP_STATUS_ONLINE  = 2,
} EspC6Status_t;

/**
 * @brief Cached pairing payloads used for Matter commissioning.
 *
 * qr     - Matter QR payload, e.g. "MT:..." (base38).
 * manual - Matter manual code, e.g. "34970112332".
 */
typedef struct {
    char qr[ESP_QR_MAX_LEN];           /**< Matter QR payload, e.g. "MT:...". */
    char manual[ESP_MANUAL_MAX_LEN];   /**< Matter manual code, e.g. "34970112332". */
} EspC6Pairing_t;

/**
 * @brief Runtime context (timings, status, pairing) for ESP32-C6 link.
 *
 * This struct does NOT contain GPIO/UART handles; those come from main.h.
 */
typedef struct {
    uint32_t ready_wait_ms;         /**< Time to wait for READY high after asserting WAKE. */
    uint32_t def_timeout_ms;        /**< Default sliding timeout for most commands. */
    uint8_t  success_without_tokens;/**< When no tokens are requested, treat as success if 1. */

    uint16_t drain_silence_ms;      /**< Optional guard-drain silence (not used in core API). */
    uint16_t drain_cap_ms;          /**< Optional guard-drain cap (not used in core API). */

    EspC6Status_t status;           /**< Last known status (ONLINE/OFFLINE/UNKNOWN). */

    EspC6Pairing_t pairing;         /**< Cached QR + manual pairing data. */
} EspC6Context_t;

/* Global instance (like g_wio). */
extern EspC6Context_t g_esp;

/* ===== Low-level core ===== */

/**
 * @brief Send one ASCII command with WAKE/READY handshake and look for any token.
 *
 * Typical call pattern:
 *   EspC6_SendCommand("PING", "PONG", 1000);
 *
 * The function:
 *   - Asserts WAKE, waits for READY.
 *   - Sends cmd (if non-NULL) plus CRLF.
 *   - Reads bytes until timeout or token match + small post-match delay.
 *   - Stores all received bytes into esp_rx_buffer (NUL-terminated).
 *
 * @param cmd       ASCII command WITHOUT CRLF (can be NULL to only read).
 * @param responses Pipe-separated tokens to accept, e.g. "OK|READY|PONG".
 *                  If NULL/empty, no validation is done (just capture).
 * @param timeout   Sliding timeout (ms) that resets after every received byte.
 * @return 1 if a token was found (or no tokens and success_without_tokens == 1), otherwise 0.
 */
uint8_t EspC6_SendCommand(const char *cmd, const char *responses, uint32_t timeout);

void ESP_Init(void);

/* ===== High-level helpers used by the console ===== */

/**
 * @brief Send PING and expect PONG from ESP32-C6.
 */
uint8_t EspC6_Ping(void);

/**
 * @brief Query ESP32-C6 status and update g_esp.status.
 */
uint8_t EspC6_Status(void);

/**
 * @brief Request version/firmware info from ESP32-C6.
 */
uint8_t EspC6_Version(void);

/**
 * @brief Request Matter COMM QR + MANUAL codes and cache into g_esp.pairing.
 */
uint8_t EspC6_QR(void);

/**
 * @brief List fabrics / commissioners stored inside ESP32-C6.
 */
uint8_t EspC6_Fabrics(void);

/**
 * @brief Remove a fabric by index (declared for completeness, implemented elsewhere).
 */
uint8_t EspC6_FabricRemove(uint8_t idx);

/**
 * @brief Start commissioning (COMM) session.
 */
uint8_t EspC6_CommStart(void);

/**
 * @brief Stop commissioning (COMM) session.
 */
uint8_t EspC6_CommStop(void);

/**
 * @brief Perform ESP32-C6 factory reset (clear fabrics / pairing data).
 */
uint8_t EspC6_FactoryReset(void);

/* ===== Data lines (one-way; ESP typically doesn’t reply to DATA) ===== */

/**
 * @brief Send 6-byte payload as 12 hex chars in "DATA HEX" frame.
 */
uint8_t EspC6_DataHex(const uint8_t payload6[6]);

/**
 * @brief Send key-value climate payload in "DATA KV" format.
 */
uint8_t EspC6_DataKV(int16_t t_01C, uint16_t rh_01pct, uint16_t co2_ppm);

/**
 * @brief Send battery status in "DATA BAT KV" format.
 */
uint8_t EspC6_DataBat(uint16_t vbat_liion_mV,
                      uint16_t vbat_lsocl2_mV,
                      uint8_t usb_on,
                      uint8_t charger_status);
