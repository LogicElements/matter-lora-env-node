/*
 * SHT41.h
 *
 *  Created on: Mar 27, 2025
 *      Author: Evzen Steif
 *
 *  @brief Public interface for the SHT41 temperature and humidity sensor driver.
 */

#include "stm32u0xx_hal.h"

#ifndef INC_SHT41_H_
#define INC_SHT41_H_

#define SHT41_ADRESS      0x44   /**< 7-bit I2C address of SHT41 (datasheet value). */
#define MEASURE_COMD      0xFD   /**< High-precision T/RH measurement command. */
#define SHT41_SOFT_RESET  0x94   /**< Soft reset command. */

/**
 * @brief Latest SHT41 temperature and humidity measurement.
 *
 * temperature - in degrees Celsius.
 * rh          - relative humidity in percent.
 */
typedef struct
{
    float temperature;
    float rh;
} SHT41_Data;

extern SHT41_Data SHT_data;

/**
 * @brief Verify SHT4x CRC for a 16-bit data word.
 */
uint8_t SHT41_CheckCRC(uint16_t raw_data, uint8_t checksum);

/**
 * @brief Perform one T/RH measurement and update SHT_data.
 */
void SHTMeasure(I2C_HandleTypeDef *hi2c);

/**
 * @brief Initialize SHT41 (soft reset).
 */
void SHTInit(I2C_HandleTypeDef *hi2c);

/**
 * @brief Issue SHT41 soft reset command.
 */
void SHTSoftReset(I2C_HandleTypeDef *hi2c);

#endif /* INC_SHT41_H_ */
