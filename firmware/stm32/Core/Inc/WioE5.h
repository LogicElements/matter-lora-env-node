/**
 * @file WioE5.h
 * @brief Public interface for Wio-E5 LoRaWAN helper functions.
 *
 * Author: Evzen Steif
 *
 * This layer wraps the Wio-E5 AT command set in a few convenience helpers:
 *  - One generic Wio_SendCommand() with token matching and shared RX buffer.
 *  - Minimal OTAA configuration and join using g_wio (AppEUI/DevEUI/AppKey/ADR).
 *  - Confirmed and unconfirmed uplink send helpers (hex payload).
 *  - Simple network time request + RTC update.
 */

#ifndef INC_WIOE5_H_
#define INC_WIOE5_H_

#include <stdint.h>

/* ===== Send result enums ===== */

/**
 * @brief Result of confirmed uplink (AT+CMSGHEX) send.
 */
typedef enum {
    WIO_SEND_ERROR       = 0, /**< Timeout or unexpected response. */
    WIO_SEND_NO_ACK      = 1, /**< Uplink OK, but no ACK indication. */
    WIO_SEND_ACK_RECEIVED= 2  /**< Uplink OK and "ACK Received" found. */
} WioSendResult_t;

/**
 * @brief Result of unconfirmed uplink (AT+MSGHEX) send.
 */
typedef enum {
    WIO_SEND_OK  = 0, /**< Uplink completed (no ACK expected). */
    WIO_SEND_ERR = 1  /**< Timeout or unexpected response.     */
} WioSendUnconfResult_t;

/* ===== OTAA runtime config ===== */

/**
 * @brief Minimal OTAA runtime config stored in module globals.
 *
 * All strings are plain ASCII hex without spaces:
 *  - app_eui: 16 hex chars + NUL terminator.
 *  - dev_eui: 16 hex chars + NUL.
 *  - app_key: 32 hex chars + NUL.
 */
typedef struct {
    char   app_eui[17]; /**< AppEUI (16 hex chars + NUL). */
    char   dev_eui[17]; /**< DevEUI (16 hex chars + NUL). */
    char   app_key[33]; /**< AppKey (32 hex chars + NUL). */
    uint8_t adr_on;     /**< 0/1: Adaptive Data Rate flag. */
} WioOtaaConfig_t;

/** @brief Global instance used by WioE5.c functions. */
extern WioOtaaConfig_t g_wio;

/* ===== Core AT helper ===== */

/**
 * @brief Send AT command and block until one of the expected tokens appears or timeout elapses.
 *
 * @param cmd       AT command string without CRLF (NULL = only read).
 * @param responses Pipe-separated list of accepted tokens (e.g. "OK|ERROR").
 *                  May be NULL/empty if only capturing response.
 * @param timeout   Overall timeout in milliseconds.
 * @return 1 if any token was matched, 0 on timeout or no match.
 */
uint8_t Wio_SendCommand(const char *cmd, const char *responses, uint32_t timeout);

/* ===== Uplink helpers ===== */

WioSendUnconfResult_t Wio_SendData_Unconfirmed(const uint8_t *data, uint8_t length);
WioSendResult_t       Wio_SendData(uint8_t *data, uint8_t length);

/* ===== OTAA / ABP init ===== */

/**
 * @brief (Optional) ABP init placeholder, if used in the project.
 * Not implemented in the provided source; keep prototype for backward compatibility.
 */
uint8_t Wio_Init_ABP(void);

/**
 * @brief Configure modem for OTAA using g_wio (AppEUI/DevEUI/AppKey/ADR).
 */
uint8_t Wio_Init_OTAA(void);

/* ===== Network time support ===== */

uint8_t WioTimeReq(void);
uint8_t Wio_ParseTimeDownlink(const char *uart_response);

/**
 * @brief Optional read helper (not implemented in the provided source).
 * Prototype kept for API compatibility.
 */
uint8_t Wio_ReadResponse(char *out_buffer, uint16_t max_len, uint32_t timeout_ms);

/* ===== Modem readiness / low-power ===== */

uint8_t Wio_EnsureReady(uint32_t overall_ms);
void    Wio_SleepEnter(void);
uint8_t Wio_Wake(uint32_t overall_ms);

/* ===== Utility helpers ===== */

/**
 * @brief Fill g_wio with compile-time defaults and ADR ON.
 */
void Wio_SetOtaaDefaults(void);

/**
 * @brief Check if s is exactly expect_len hex chars [0-9A-Fa-f].
 */
int Wio_IsHexLen(const char* s, int expect_len);

/**
 * @brief Convenience: attempt OTAA join now with g_wio keys.
 */
uint8_t Wio_JoinNow(uint32_t timeout_ms);

#endif /* INC_WIOE5_H_ */
