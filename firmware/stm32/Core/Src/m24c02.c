/**
 * @file m24c02.c
 * @brief Minimal HAL-based driver for M24C02 I2C EEPROM (2 kbit / 256 bytes).
 *
 * Provides:
 *  - Simple initialization bound to a given I2C handle.
 *  - Byte/array read with bounds checks.
 *  - Page-aware write (8-byte pages) with standby polling after each page.
 *  - Full-chip erase helper (fill with 0xFF).
 *  - Simple self-test using a fixed pattern in a safe block.
 *
 * Author: Evzen Steif
 */

#include "m24c02.h"
#include <string.h>

/**
 * @brief Wait until the EEPROM is ready (standby) after a write operation.
 *
 * Uses HAL_I2C_IsDeviceReady() which repeatedly addresses the device until it
 * responds with ACK or the number of trials is exhausted.
 *
 * @param hee    Pointer to EEPROM handle.
 * @param trials Number of readiness polling trials.
 * @return HAL_OK if the device is ready, otherwise HAL error code.
 */
static HAL_StatusTypeDef m24c02_wait_standby(M24C02_HandleTypeDef *hee, uint32_t trials)
{
    return HAL_I2C_IsDeviceReady(hee->hi2c, hee->devAddr, trials, hee->timeout_ms);
}

/**
 * @brief Initialize an M24C02 handle with I2C handle, fixed address, and timeout.
 *
 * The device address is fixed to M24C02_HAL_ADDR_FIXED (0x50 << 1 = 0xA0)
 * assuming A2=A1=A0 are tied to GND.
 *
 * @param hee        Pointer to EEPROM handle to be filled.
 * @param hi2c       Pointer to I2C HAL handle (e.g., &hi2c1).
 * @param timeout_ms Timeout in milliseconds for I2C operations (0 → default 100 ms).
 */
void M24C02_Init(M24C02_HandleTypeDef *hee,
                 I2C_HandleTypeDef *hi2c,
                 uint32_t timeout_ms)
{
    hee->hi2c       = hi2c;
    hee->devAddr    = M24C02_HAL_ADDR_FIXED;   // 0xA0
    hee->timeout_ms = (timeout_ms != 0U) ? timeout_ms : 100U;
}

/**
 * @brief Check whether the EEPROM is ready (e.g. after a write cycle).
 *
 * Wraps the internal standby wait helper with a default number of trials.
 *
 * @param hee    Pointer to EEPROM handle.
 * @param trials Number of readiness polling trials (0 → default 50).
 * @return HAL_OK if device is ready, otherwise HAL error code.
 */
HAL_StatusTypeDef M24C02_IsReady(M24C02_HandleTypeDef *hee, uint32_t trials)
{
    return m24c02_wait_standby(hee, trials ? trials : 50U);
}

/**
 * @brief Read a block of data from EEPROM starting at memAddr.
 *
 * Performs simple bounds checking: memAddr must be within 0..255 and the
 * (memAddr + len) must not exceed M24C02_TOTAL_SIZE_BYTES.
 *
 * @param hee     Pointer to EEPROM handle.
 * @param memAddr Byte address inside EEPROM (0..255).
 * @param pData   Destination buffer.
 * @param len     Number of bytes to read.
 * @return HAL_OK on success, HAL_ERROR or HAL I2C error code on failure.
 */
HAL_StatusTypeDef M24C02_Read(M24C02_HandleTypeDef *hee,
                              uint16_t memAddr,
                              uint8_t *pData,
                              uint16_t len)
{
    if (!hee || !pData) return HAL_ERROR;
    if (memAddr >= M24C02_TOTAL_SIZE_BYTES) return HAL_ERROR;
    if ((memAddr + len) > M24C02_TOTAL_SIZE_BYTES) return HAL_ERROR;

    return HAL_I2C_Mem_Read(hee->hi2c,
                            hee->devAddr,
                            (uint16_t)memAddr,
                            I2C_MEMADD_SIZE_8BIT,
                            pData,
                            len,
                            hee->timeout_ms);
}

/**
 * @brief Write a block of data into EEPROM with automatic page handling.
 *
 * The M24C02 uses 8-byte pages. This function splits the write into chunks so
 * that no write crosses a page boundary, and after each chunk it polls the
 * device until the internal programming cycle is finished.
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
                               uint16_t len)
{
    if (!hee || !pData) return HAL_ERROR;
    if (memAddr >= M24C02_TOTAL_SIZE_BYTES) return HAL_ERROR;
    if ((memAddr + len) > M24C02_TOTAL_SIZE_BYTES) return HAL_ERROR;

    HAL_StatusTypeDef st;
    uint16_t offset = 0;

    while (offset < len) {
        uint16_t currAddr   = (uint16_t)(memAddr + offset);
        uint16_t pageRemain = (uint16_t)(M24C02_PAGE_SIZE_BYTES - (currAddr % M24C02_PAGE_SIZE_BYTES));
        uint16_t chunk      = (uint16_t)((len - offset) < pageRemain ? (len - offset) : pageRemain);

        st = HAL_I2C_Mem_Write(hee->hi2c,
                               hee->devAddr,
                               currAddr,
                               I2C_MEMADD_SIZE_8BIT,
                               (uint8_t const*)(pData + offset),
                               chunk,
                               hee->timeout_ms);
        if (st != HAL_OK) return st;

        // Internal programming time (typically ~5 ms).
        st = m24c02_wait_standby(hee, 50U);
        if (st != HAL_OK) return st;

        offset += chunk;
    }
    return HAL_OK;
}

/**
 * @brief Erase the entire EEPROM by writing 0xFF to all pages.
 *
 * Writes one full page (M24C02_PAGE_SIZE_BYTES) at a time and waits for the
 * device to become ready after each page write.
 *
 * @param hee Pointer to EEPROM handle.
 * @return HAL_OK on success, HAL_ERROR or HAL I2C error code on failure.
 */
HAL_StatusTypeDef M24C02_EraseAll(M24C02_HandleTypeDef *hee)
{
    uint8_t ff[M24C02_PAGE_SIZE_BYTES];
    memset(ff, 0xFF, sizeof(ff));

    for (uint16_t addr = 0; addr < M24C02_TOTAL_SIZE_BYTES; addr += M24C02_PAGE_SIZE_BYTES) {
        HAL_StatusTypeDef st = HAL_I2C_Mem_Write(hee->hi2c,
                                                 hee->devAddr,
                                                 addr,
                                                 I2C_MEMADD_SIZE_8BIT,
                                                 ff,
                                                 M24C02_PAGE_SIZE_BYTES,
                                                 hee->timeout_ms);
        if (st != HAL_OK) return st;
        st = m24c02_wait_standby(hee, 50U);
        if (st != HAL_OK) return st;
    }
    return HAL_OK;
}

/**
 * @brief Simple self-test: write a fixed pattern into a safe block and read it back.
 *
 * Uses a test block starting at address 0xC0 to avoid overlapping with typical
 * configuration storage. The pattern is written with M24C02_Write() and then
 * read back with M24C02_Read(); both data blocks are compared with memcmp().
 *
 * @param hee Pointer to EEPROM handle.
 * @return HAL_OK if the pattern round-trip matches exactly, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef M24C02_SelfTest(M24C02_HandleTypeDef *hee)
{
    static const uint8_t pattern[] = {
        0x55,0xAA,0x12,0x34,0x00,0xFF,0x99,0x66,0xDE,0xAD,0xBE,0xEF,0x10,0x20,0x30,0x40
    };
    uint8_t rd[sizeof(pattern)];
    uint16_t base = 0xC0; // safe test block near the end of EEPROM

    if (M24C02_Write(hee, base, pattern, sizeof(pattern)) != HAL_OK) return HAL_ERROR;
    if (M24C02_Read (hee, base, rd, sizeof(rd))            != HAL_OK) return HAL_ERROR;

    return (memcmp(pattern, rd, sizeof(pattern)) == 0) ? HAL_OK : HAL_ERROR;
}
