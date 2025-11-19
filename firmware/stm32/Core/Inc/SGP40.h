/*
 * SGP40.h
 *
 *  VOC sensing + Sensirion VOC index wrapper.
 *      Author: Evzen Steif
 *
 *  @brief Public interface for SGP40 VOC sensor helpers and VOC index engine.
 */

#ifndef INC_SGP40_H_
#define INC_SGP40_H_

#include <stdint.h>
#include "stm32u0xx_hal.h"

#define SGP40_I2C_ADDR (0x59 << 1)

/**
 * @brief Latest SGP40 VOC measurement.
 *
 * voc_raw   - Raw SGP40 16-bit output word.
 * voc_index - Sensirion VOC index, calculated from voc_raw using the
 *             GasIndexAlgorithm. The algorithm's conceptual range is 0..500+;
 *             this driver clamps to 0..65535 (16-bit) when storing.
 */
typedef struct {
    uint16_t voc_raw;   /* raw SGP40 word */
    uint16_t voc_index; /* Sensirion VOC index (0..500+ scaled to 0..65535 cap in driver) */
} SGP40_Data;

extern SGP40_Data sgp40_data;

/* VOC algorithm bootstrap */
/**
 * @brief Initialize Sensirion VOC Index algorithm for use with SGP40.
 *
 * @param sampling_interval_seconds Sampling interval in seconds; if <= 0,
 *                                  the library default (1 s) is used.
 */
void SGP40_VOC_Init(float sampling_interval_seconds);

/* Continuous low-power VOC mode (USB-only use in this project) */
/**
 * @brief Start continuous VOC mode, configuring the VOC index algorithm.
 *
 * The caller must periodically call SGP40_VOC_ContStep() at the configured
 * sampling interval.
 */
HAL_StatusTypeDef SGP40_VOC_ContStart(I2C_HandleTypeDef *hi2c,
                                      uint32_t timeout_ms,
                                      float sampling_interval_seconds);

/**
 * @brief Stop continuous VOC mode (does not touch the sensor directly).
 */
void SGP40_VOC_ContStop(void);

/**
 * @brief Perform one low-power continuous VOC step (measurement + index).
 */
HAL_StatusTypeDef SGP40_VOC_ContStep(I2C_HandleTypeDef *hi2c, uint32_t timeout_ms);

/* Optional single compensated sample into sgp40_data */
/**
 * @brief Take one compensated VOC sample and update sgp40_data (raw + index).
 */
HAL_StatusTypeDef SGP40_MeasureOnce(I2C_HandleTypeDef *hi2c, uint32_t timeout_ms);

/**
 * @brief Read one compensated raw SGP40 sample with heater turned OFF afterwards.
 */
HAL_StatusTypeDef SGP40_ReadRawComp(I2C_HandleTypeDef *hi2c,
                                    uint32_t timeout_ms,
                                    uint16_t *out_raw);

/**
 * @brief Read one compensated raw SGP40 sample with heater kept ON (steady-state).
 *
 * First call performs warm-up and discards the first reading. Subsequent calls
 * are power-optimized for 1 Hz operation.
 */
HAL_StatusTypeDef SGP40_ReadRawComp_HeaterOn(I2C_HandleTypeDef *hi2c,
                                             uint32_t timeout_ms,
                                             uint16_t *out_raw);

#endif /* INC_SGP40_H_ */
