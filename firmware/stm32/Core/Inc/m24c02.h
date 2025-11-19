/*
 * m24c02.h
 *
 *  Created on: Nov 12, 2025
 *      Author: Evzen Steif
 *
 *  Minimalist wrapper around HAL I2C for STMicroelectronics + M24C02 EEPROM.
 *  Device: 2 kbit I2C EEPROM → 256 bytes total, 8-byte page size.
 */

#ifndef SRC_M24C02_H_
#define SRC_M24C02_H_

#include "stm32u0xx_hal.h"

// M24C02 geometry: 2 kbit = 256 bytes total, 8-byte pages.
#define M24C02_TOTAL_SIZE_BYTES   (256U)
#define M24C02_PAGE_SIZE_BYTES    (8U)

// HAL uses 7-bit address left-shifted by 1 → 0x50 << 1 = 0xA0.
#define M24C02_HAL_ADDR_FIXED     ((uint16_t)(0x50U << 1))

/**
 * @brief Simple handle describing a single M24C02 device on a given I2C bus.
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;   /**< Pointer to I2C HAL handle (e.g., &hi2c1). */
    uint16_t           devAddr;/**< 8-bit I2C device address (0xA0 for M24C02 with A2=A1=A0=0). */
    uint32_t           timeout_ms; /**< Timeout in milliseconds for HAL I2C operations. */
} M24C02_HandleTypeDef;

/**
 * @brief Initialize the M24C02 handle (fixed address 0xA0 when A2=A1=A0=0).
 *
 * This function does not touch the I2C peripheral itself, only sets up the
 * EEPROM handle with the given I2C instance, device address and timeout.
 *
 * @param hee        Pointer to EEPROM handle to be filled.
 * @param hi2c       Pointer to I2C HAL handle.
 * @param timeout_ms Timeout in ms for I2C access (0 → default 100 ms).
 */
void M24C02_Init(M24C02_HandleTypeDef *hee,
                 I2C_HandleTypeDef *hi2c,
                 uint32_t timeout_ms);

/**
 * @brief Check whether the EEPROM is ready (e.g. after a write).
 *
 * Uses HAL_I2C_IsDeviceReady() with the given number of trials.
 *
 * @param hee    Pointer to EEPROM handle.
 * @param trials Number of readiness polling trials (0 → default 50).
 * @return HAL_OK if ready, otherwise HAL error code.
 */
HAL_StatusTypeDef M24C02_IsReady(M24C02_HandleTypeDef *hee, uint32_t trials);

/**
 * @brief Read len bytes starting at memAddr (0..255).
 *
 * Bounds are checked against M24C02_TOTAL_SIZE_BYTES. If the call would
 * exceed device capacity, HAL_ERROR is returned without touching the bus.
 *
 * @param hee     Pointer to EEPROM handle.
 * @param memAddr Start byte address (0..255).
 * @param pData   Pointer to destination buffer.
 * @param len     Number of bytes to read.
 * @return HAL_OK on success, HAL_ERROR or HAL I2C error code on failure.
 */
HAL_StatusTypeDef M24C02_Read(M24C02_HandleTypeDef *hee,
                              uint16_t memAddr,
                              uint8_t *pData,
                              uint16_t len);

/**
 * @brief Write len bytes starting at memAddr using page-aware write.
 *
 * The function automatically splits the data into 8-byte page chunks so no
 * write crosses a page boundary, and waits for the internal write cycle to
 * finish after each page.
 *
 * @param hee     Pointer to EEPROM handle.
 * @param memAddr Start byte address (0..255).
 * @param pData   Pointer to source data.
 * @param len     Number of bytes to write.
 * @return HAL_OK on success, HAL_ERROR or HAL I2C error code on failure.
 */
HAL_StatusTypeDef M24C02_Write(M24C02_HandleTypeDef *hee,
                               uint16_t memAddr,
                               const uint8_t *pData,
                               uint16_t len);

/**
 * @brief Erase the entire 256-byte EEPROM by writing 0xFF to all pages.
 *
 * Writes one full page at a time (M24C02_PAGE_SIZE_BYTES) and waits for the
 * device to finish its internal write cycle between pages.
 *
 * @param hee Pointer to EEPROM handle.
 * @return HAL_OK on success, HAL_ERROR or HAL I2C error code on failure.
 */
HAL_StatusTypeDef M24C02_EraseAll(M24C02_HandleTypeDef *hee);

/**
 * @brief Simple built-in self-test of the EEPROM.
 *
 * Writes a fixed test pattern into a safe region (starting at 0xC0), reads it
 * back and compares. Intended as a quick sanity check during bring-up.
 *
 * @param hee Pointer to EEPROM handle.
 * @return HAL_OK if test passes, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef M24C02_SelfTest(M24C02_HandleTypeDef *hee);

#ifdef __cplusplus
}
#endif

#endif /* SRC_M24C02_H_ */
