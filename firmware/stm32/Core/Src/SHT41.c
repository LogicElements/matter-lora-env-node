/*
 * SHT41.c
 *
 *  Created on: Mar 27, 2025
 *      Author: Evzen Steif
 *
 *  @brief Minimal driver for Sensirion SHT41 temperature and humidity sensor.
 *
 *  Provides:
 *    - Sensor initialization and soft reset.
 *    - Single high-precision T/RH measurement.
 *    - CRC check for received raw data words.
 */

#include "SHT41.h"

SHT41_Data SHT_data;

/**
 * @brief Initialize SHT41 sensor (currently soft reset only).
 *
 * Call this once after power-up. Uses SHTSoftReset() to ensure a known state.
 */
void SHTInit(I2C_HandleTypeDef *hi2c)
{
    SHTSoftReset(hi2c);
}

/**
 * @brief Issue SHT41 soft reset command.
 *
 * @param hi2c I2C handle used to communicate with the sensor.
 */
void SHTSoftReset(I2C_HandleTypeDef *hi2c)
{
    uint8_t cmd = SHT41_SOFT_RESET;
    HAL_I2C_Master_Transmit(hi2c, SHT41_ADRESS << 1, &cmd, 1, HAL_MAX_DELAY);
}

/**
 * @brief Perform one high-precision T/RH measurement and update SHT_data.
 *
 * Sequence:
 *   1) Send measurement command.
 *   2) Wait for conversion to finish.
 *   3) Read 6 bytes (T[0..1], CRC_T[2], RH[3..4], CRC_RH[5]).
 *   4) Validate each word using SHT41_CheckCRC().
 *   5) Convert valid raw words to °C and %RH.
 *
 * On CRC failure, the corresponding field in SHT_data is left unchanged.
 *
 * @param hi2c I2C handle used to communicate with the sensor.
 */
void SHTMeasure(I2C_HandleTypeDef *hi2c)
{
    uint8_t cmd = MEASURE_COMD;
    uint8_t response[6];  // temperature [0-1], CRC_t [2], rh [3-4], CRC_rh [5]

    HAL_I2C_Master_Transmit(hi2c, SHT41_ADRESS << 1, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(10);
    HAL_I2C_Master_Receive(hi2c, SHT41_ADRESS << 1, response, 6, HAL_MAX_DELAY);

    uint16_t temp_raw = (response[0] << 8) | response[1];
    uint8_t t_checksum = response[2];
    uint16_t rh_raw = (response[3] << 8) | response [4];
    uint8_t rh_checksum = response[5];

    if (SHT41_CheckCRC(temp_raw, t_checksum))
    {
        SHT_data.temperature = -45.0 + 175.0*(float)temp_raw/65535.0;
    }

    if (SHT41_CheckCRC(rh_raw, rh_checksum))
    {
        SHT_data.rh = -6.0 + 125.0*(float)rh_raw/65535.0;
    }
}

/**
 * @brief Verify SHT4x CRC for a 16-bit data word.
 *
 * Polynomial 0x31, initial value 0xFF, as per Sensirion datasheet.
 *
 * @param raw_data 16-bit word received from the sensor.
 * @param checksum 8-bit CRC byte received from the sensor.
 * @return 1 if CRC matches, 0 otherwise.
 */
uint8_t SHT41_CheckCRC(uint16_t raw_data, uint8_t checksum)
{
    uint8_t crc = 0xFF;
    uint8_t data[2] = { raw_data >> 8, raw_data & 0xFF };

    for (uint8_t i = 0; i < 2; i++) {  // 2 data bytes
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {  // 8 bits
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return (crc == checksum) ? 1 : 0;
}
