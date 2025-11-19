/**
 * @file WioE5.c
 * @brief Minimal helper layer for Seeed Wio-E5 LoRaWAN module (AT commands over UART).
 *
 * Author: Evzen Steif
 *
 * Responsibilities:
 *  - Keep a global OTAA config (g_wio) for AppEUI / DevEUI / AppKey / ADR.
 *  - Provide Wio_SendCommand() to send AT command and wait for any of the
 *    expected response tokens.
 *  - Provide simple wrappers to configure OTAA, join the network, send
 *    confirmed / unconfirmed uplinks and request network time.
 *
 * Hardware binding:
 *  - Uses extern UART_HandleTypeDef huart1 (declared in main.c / main.h).
 *  - RTC integration uses Set_RTC_From_Epoch() from RTC.c.
 */

#include "stm32u0xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <ctype.h>

#include "WioE5.h"
#include "main.h"
#include "RTC.h"

/* RX buffer for all Wio-E5 responses (for logging / parsing). */
char wio_rx_buffer[512];

/* Compile-time defaults (can be overridden later via g_wio). */
#define DEF_APP_EUI "1122334455667788"
#define DEF_DEV_EUI "70B3D57ED006F192"
#define DEF_APP_KEY "7149209878EE40FD79CEE047DBF5C3E4"

extern RTC_HandleTypeDef  hrtc;
extern UART_HandleTypeDef huart1;

/* ===== Global OTAA runtime config ===== */

WioOtaaConfig_t g_wio = {
    .app_eui = DEF_APP_EUI,
    .dev_eui = DEF_DEV_EUI,
    .app_key = DEF_APP_KEY,
    .adr_on  = 1,
};

/* ------------------------------------------------------------------------- */
/* Small utilities                                                           */
/* ------------------------------------------------------------------------- */

/**
 * @brief Check that string is exactly expect_len hex chars [0-9A-Fa-f].
 *
 * @param s          Zero-terminated C string (may be NULL).
 * @param expect_len Required length in characters.
 * @return 1 if string is valid hex of the required length, otherwise 0.
 */
int Wio_IsHexLen(const char* s, int expect_len)
{
    if (!s) return 0;
    int n = 0;
    while (*s) {
        unsigned char c = (unsigned char)*s++;
        if (!isxdigit(c)) return 0;
        n++;
    }
    return (n == expect_len);
}

/**
 * @brief Fill g_wio with sane defaults (compile-time keys and ADR ON).
 *
 * - AppEUI/DevEUI/AppKey are set to DEF_* macros.
 * - adr_on is set to 1 (ADR enabled).
 */
void Wio_SetOtaaDefaults(void)
{
    memset(&g_wio, 0, sizeof(g_wio));
    strncpy(g_wio.app_eui, DEF_APP_EUI, sizeof(g_wio.app_eui) - 1);
    strncpy(g_wio.dev_eui, DEF_DEV_EUI, sizeof(g_wio.dev_eui) - 1);
    strncpy(g_wio.app_key, DEF_APP_KEY, sizeof(g_wio.app_key) - 1);
    g_wio.adr_on = 1;
}

/* ------------------------------------------------------------------------- */
/* Core AT command helper                                                    */
/* ------------------------------------------------------------------------- */

/**
 * @brief Send one AT command and search for any of the expected response tokens.
 *
 * Behavior:
 *  - Clears wio_rx_buffer and then sends cmd over huart1, appending CRLF.
 *  - Reads characters until timeout.
 *  - After each new character, keeps wio_rx_buffer NUL-terminated so it can
 *    be safely used with strstr().
 *  - responses is a '|'-separated list, e.g. "OK|ERROR|Network joined".
 *    If any token is found as substring in the received buffer, the function
 *    returns success (1).
 *
 * @param cmd       AT command string without CRLF; if NULL, only read.
 * @param responses Pipe-separated list of expected tokens; must not be NULL
 *                  when non-empty matching is desired.
 * @param timeout   Overall timeout in ms for receiving a matching token.
 * @return 1 if any token was found, 0 if timeout / no match.
 */
uint8_t Wio_SendCommand(const char *cmd, const char *responses, uint32_t timeout)
{
    memset(wio_rx_buffer, 0, sizeof(wio_rx_buffer));
    uint8_t  temp_byte = 0;
    uint16_t index     = 0;

    uint32_t start_time = HAL_GetTick();

    /* Transmit command, if provided */
    if (cmd != NULL) {
        HAL_UART_Transmit(&huart1, (const uint8_t*)cmd, (uint16_t)strlen(cmd), HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, (const uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    }

    while ((HAL_GetTick() - start_time) < timeout) {
        if (HAL_UART_Receive(&huart1, &temp_byte, 1, 200) == HAL_OK) {
            if (index < sizeof(wio_rx_buffer) - 1) {
                wio_rx_buffer[index++] = (char)temp_byte;
                wio_rx_buffer[index]   = '\0';
            } else {
                /* Buffer full, stop receiving */
                break;
            }

            if (responses && responses[0]) {
                /* Check all possible '|'-separated tokens */
                char resp_copy[128];
                strncpy(resp_copy, responses, sizeof(resp_copy) - 1);
                resp_copy[sizeof(resp_copy) - 1] = '\0';

                char *token = strtok(resp_copy, "|");
                while (token != NULL) {
                    if (strstr(wio_rx_buffer, token) != NULL) {
                        printf("WIO Response:\r\n%s\r\n", wio_rx_buffer);
                        printf("Response \"%s\" found!\r\n", token);
                        return 1; /* success */
                    }
                    token = strtok(NULL, "|");
                }
            }
        }
    }

    printf("WIO Response:\r\n%s\r\n", wio_rx_buffer);
    printf("No expected response found!\r\n");
    return 0;
}

/* ------------------------------------------------------------------------- */
/* OTAA config + join                                                        */
/* ------------------------------------------------------------------------- */

/**
 * @brief Configure Wio-E5 for OTAA using runtime keys in g_wio.
 *
 * Sequence:
 *  - Validate AppEUI (16 hex chars), DevEUI (16 hex), AppKey (32 hex).
 *  - Set IDs via AT+ID and AT+KEY.
 *  - Configure EU868 DR, ADR ON/OFF, channels 0-2, Class A.
 *  - Switch to LWOTAA mode (resets LoRaWAN stack).
 *
 * This function only configures the modem; it does not perform a join.
 *
 * @return 1 on success, 0 if any step fails or keys are invalid.
 */
uint8_t Wio_Init_OTAA(void)
{
    uint8_t res;
    const uint32_t T = 1500;

    /* Validate keys first */
    if (!Wio_IsHexLen(g_wio.app_eui, 16) ||
        !Wio_IsHexLen(g_wio.dev_eui, 16) ||
        !Wio_IsHexLen(g_wio.app_key, 32)) {
        printf("OTAA keys invalid (len/hex).\r\n");
        return 0;
    }

    char cmd[96];

    snprintf(cmd, sizeof(cmd), "AT+ID=AppEui,\"%s\"", g_wio.app_eui);
    res = Wio_SendCommand(cmd, "+ID", T); if (!res) return 0;

    snprintf(cmd, sizeof(cmd), "AT+ID=DevEui,\"%s\"", g_wio.dev_eui);
    res = Wio_SendCommand(cmd, "+ID", T); if (!res) return 0;

    snprintf(cmd, sizeof(cmd), "AT+KEY=APPKEY,\"%s\"", g_wio.app_key);
    res = Wio_SendCommand(cmd, "+KEY", T); if (!res) return 0;

    /* EU868 band plan + ADR + join channels 0-2 + Class A */
    res = Wio_SendCommand("AT+DR=EU868", "+DR",   T); if (!res) return 0;
    res = Wio_SendCommand(g_wio.adr_on ? "AT+ADR=ON" : "AT+ADR=OFF", "+ADR", T); if (!res) return 0;
    res = Wio_SendCommand("AT+CH=NUM,0-2", "+CH", T); if (!res) return 0;
    res = Wio_SendCommand("AT+CLASS=A", "+CLASS", T); if (!res) return 0;

    /* Enable OTAA mode (resets internal LW stack) */
    res = Wio_SendCommand("AT+MODE=LWOTAA", "+MODE", T); if (!res) return 0;

    return 1;
}

/**
 * @brief Attempt an OTAA join immediately using keys in g_wio.
 *
 * Useful mainly from the configuration console or debug shell.
 *
 * @param timeout_ms Overall timeout to wait for "Network joined" / "Joined already".
 * @return 1 on successful join or "Joined already", otherwise 0.
 */
uint8_t Wio_JoinNow(uint32_t timeout_ms)
{
    if (!Wio_EnsureReady(1500)) return 0;
    if (!Wio_Init_OTAA())       return 0;
    return Wio_SendCommand("AT+JOIN", "Network joined|Joined already", timeout_ms);
}

/* ------------------------------------------------------------------------- */
/* Uplink helpers                                                            */
/* ------------------------------------------------------------------------- */

/**
 * @brief Send confirmed LoRaWAN uplink as hex payload (AT+CMSGHEX).
 *
 * @param data   Pointer to payload bytes.
 * @param length Number of bytes in payload.
 * @return WIO_SEND_ACK_RECEIVED if ACK Received is seen in response,
 *         WIO_SEND_NO_ACK       if no ACK is indicated but +CMSGHEX: Done was seen,
 *         WIO_SEND_ERROR        on timeout or unexpected response.
 */
WioSendResult_t Wio_SendData(uint8_t *data, uint8_t length)
{
    char buffer[60];
    char command[80];

    for (uint8_t i = 0; i < length; i++) {
        sprintf(&buffer[i * 2], "%02X", data[i]);
    }

    sprintf(command, "AT+CMSGHEX=%s", buffer);
    uint8_t res = Wio_SendCommand(command, "+CMSGHEX: Done", 30000);

    if (!res) return WIO_SEND_ERROR;

    if (strstr(wio_rx_buffer, "ACK Received") != NULL)
        return WIO_SEND_ACK_RECEIVED;
    else
        return WIO_SEND_NO_ACK;
}

/**
 * @brief Send unconfirmed LoRaWAN uplink as hex payload (AT+MSGHEX).
 *
 * @param data   Pointer to payload bytes.
 * @param length Number of bytes in payload.
 * @return WIO_SEND_OK on success, WIO_SEND_ERR on timeout or error.
 */
WioSendUnconfResult_t Wio_SendData_Unconfirmed(const uint8_t *data, uint8_t length)
{
    char buffer[60];
    char command[80];

    for (uint8_t i = 0; i < length; i++) {
        sprintf(&buffer[i * 2], "%02X", data[i]);
    }

    sprintf(command, "AT+MSGHEX=%s", buffer);
    uint8_t res = Wio_SendCommand(command, "+MSGHEX: Done", 50000);

    if (!res) return WIO_SEND_ERR;
    return WIO_SEND_OK;
}

/* ------------------------------------------------------------------------- */
/* Network time request (LoRaWAN downlink)                                   */
/* ------------------------------------------------------------------------- */

/**
 * @brief Request time from network via LoRaWAN uplink (AT+MSGHEX=11).
 *
 * Wio-E5 is expected to receive a downlink containing a UNIX timestamp,
 * which we parse and apply via Set_RTC_From_Epoch().
 *
 * @retval 1 if uplink and downlink parsing succeed, otherwise 0.
 */
uint8_t WioTimeReq(void)
{
    if (!Wio_SendCommand("AT+MSGHEX=11", "+MSGHEX: Done", 20000))
        return 0;

    return Wio_ParseTimeDownlink(wio_rx_buffer);
}

/**
 * @brief Parse time downlink from an AT response and set MCU RTC.
 *
 * Expected format somewhere in uart_response:
 *   +MSGHEX: PORT:<x>; RX: "<8 hex chars>" ...
 *
 * The 8 hex chars represent a UNIX timestamp (seconds). Driver currently
 * adds +2 hours (simple DST offset) before passing to Set_RTC_From_Epoch().
 *
 * @param uart_response Full text capture from modem (wio_rx_buffer).
 * @return 1 on success, 0 if pattern is not found or malformed.
 */
uint8_t Wio_ParseTimeDownlink(const char *uart_response)
{
    const char *prefix = "+MSGHEX: PORT:";
    const char *start  = strstr(uart_response, prefix);
    if (!start) return 0;

    const char *rx_ptr = strstr(start, "RX: \"");
    if (!rx_ptr) return 0;

    rx_ptr += strlen("RX: \"");

    char hex_string[9] = {0};
    strncpy(hex_string, rx_ptr, 8);

    /* HEX → UNIX timestamp */
    uint32_t unix_time = (uint32_t)strtoul(hex_string, NULL, 16);
    unix_time += 2 * 3600;  /* simple +2h offset (DST) */

    Set_RTC_From_Epoch(unix_time);

    return 1;
}

/* ------------------------------------------------------------------------- */
/* Modem readiness + low-power helpers                                      */
/* ------------------------------------------------------------------------- */

/**
 * @brief Keep sending "AT" until we get "OK" or overall timeout expires.
 *
 * Useful as a generic "is modem alive and responsive?" probe. Also used
 * to wake the module up from internal low-power mode.
 *
 * @param overall_ms Overall time in ms to keep retrying.
 * @return 1 if "OK" was received, 0 otherwise.
 */
uint8_t Wio_EnsureReady(uint32_t overall_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < overall_ms) {
        if (Wio_SendCommand("AT", "OK", 300)) return 1;
        HAL_Delay(100);
    }
    return 0;
}

/**
 * @brief Put modem into internal low-power mode (RF rail stays ON).
 *
 * Implementation uses AT+LOWPOWER. If your firmware uses a different
 * command, this is the place to adjust it.
 */
void Wio_SleepEnter(void)
{
    (void)Wio_SendCommand("AT+LOWPOWER", "+LOWPOWER", 500);
}

/**
 * @brief Wake modem from internal low-power mode and ensure it responds to AT.
 *
 * Internally calls Wio_EnsureReady().
 *
 * @param overall_ms Overall time in ms to wait for Wio-E5 to wake.
 * @return 1 if modem responded with OK, 0 otherwise.
 */
uint8_t Wio_Wake(uint32_t overall_ms)
{
    return Wio_EnsureReady(overall_ms);
}
