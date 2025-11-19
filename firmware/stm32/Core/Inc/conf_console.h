#pragma once
/**
 * @file conf_console.h
 * @author Evzen Steif
 * @brief Public interface for the minimal interactive configuration console.
 *
 * @details
 *   The configuration console provides a simple line-based interface over UART
 *   to inspect and modify the device configuration at runtime. It is typically
 *   used when the main application enters the CONFIG state, allowing:
 *
 *   - Reading and updating AppConfig_t fields (measurement modes, intervals,
 *     TX thresholds, etc.).
 *   - Managing LoRaWAN credentials (Wio-E5 OTAA keys and ADR flag).
 *   - Interacting with the ESP32-C6 (Matter bridge) for diagnostics and
 *     commissioning flows.
 *   - Saving and restoring configuration and pairing data from EEPROM.
 */

#include <stdint.h>
#include "stm32u0xx_hal.h"
#include "app.h"  /* for AppConfig_t / enums used in set_kv() */

/**
 * @brief Initialization structure for the configuration console.
 */
typedef struct {
    UART_HandleTypeDef* huart;      /**< UART handle used for console I/O. */
    AppConfig_t*        cfg;        /**< Pointer to application configuration to manipulate. */
    uint32_t          (*sod_now)(void); /**< Callback returning current time in seconds-of-day. */
    uint32_t            inactivity_s;   /**< Auto-exit timeout after inactivity (seconds). */
} ConfConsole_Init_t;

/**
 * @brief Initialize the configuration console module.
 */
void     ConfConsole_Init(const ConfConsole_Init_t* init);

/**
 * @brief Activate the console and start UART RX.
 */
void     ConfConsole_Start(void);

/**
 * @brief Deactivate the console (no more command processing).
 */
void     ConfConsole_Stop(void);

/**
 * @brief Check if the console is currently active.
 */
uint8_t  ConfConsole_IsActive(void);

/**
 * @brief Get timestamp (seconds-of-day) of the last user activity.
 */
uint32_t ConfConsole_LastActivityS(void);

/**
 * @brief Print a short banner indicating that CONFIG mode is active.
 */
void     ConfConsole_PrintBanner(void);

/**
 * @brief Print current AppConfig, Wio-E5 and ESP32-C6 config/state.
 */
void     ConfConsole_PrintCfg(void);

/**
 * @brief Poll for pending commands and execute them (call from main loop).
 */
void     ConfConsole_Poll(void);

/**
 * @brief ISR hook: feed received byte to console when the UART matches.
 *
 * Should be called from HAL_UART_RxCpltCallback().
 */
void     ConfConsole_OnRxCplt(UART_HandleTypeDef* huart);

/**
 * @brief Query whether main loop should exit CONFIG state.
 *
 * Returns non-zero when EXIT was requested explicitly or when inactivity
 * timeout has been reached.
 */
uint8_t  ConfConsole_ShouldExit(void);
