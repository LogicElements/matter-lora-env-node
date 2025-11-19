/*
 * SGP40.c
 *
 *  VOC sensing + Sensirion VOC index wrapper.
 *  Created on: 2025
 *      Author: Evzen Steif
 *
 *  @brief Driver helpers for Sensirion SGP40 VOC sensor + VOC index algorithm.
 *
 *  This module provides:
 *    - RH/T compensation and conversion to SGP40 ticks.
 *    - Single compensated raw measurements (with/without heater-off).
 *    - A low-power continuous VOC mode feeding the Sensirion VOC Index algorithm.
 *    - A simple one-shot helper that updates sgp40_data.voc_raw / voc_index.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "SGP40.h"
#include "SHT41.h"
#include "sensirion_gas_index_algorithm.h"

/* ===== VOC engine state ===== */
static GasIndexAlgorithmParams g_voc;
static uint8_t  g_voc_inited = 0;
static float    g_voc_sampling_interval_s = 1.0f;
static uint8_t  g_voc_running = 0;  /* true after ContStart() */

SGP40_Data sgp40_data = {0};

/* ===== Local helpers ===== */

/**
 * @brief Calculate Sensirion CRC-8 for SGP40 command/response frames.
 *
 * Polynomial: 0x31, initial value: 0xFF (per Sensirion datasheet).
 */
static uint8_t sensirion_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x31u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/**
 * @brief Convert relative humidity [%] and temperature [°C] into SGP4x ticks.
 *
 * Values are clamped to SGP40 limits and scaled into 0..65535 domain
 * for humidity and temperature compensation words.
 */
static void convRHTtoTicks(float rh, float temp, uint16_t *rh_ticks, uint16_t *temp_ticks) {
    if (rh   <   0.0f) rh   =   0.0f;
    if (rh   > 100.0f) rh   = 100.0f;
    if (temp < -45.0f) temp = -45.0f;
    if (temp > 130.0f) temp = 130.0f;

    uint32_t rh_i   = (uint32_t)(rh * 65535.0f / 100.0f + 0.5f);
    uint32_t temp_i = (uint32_t)((temp + 45.0f) * 65535.0f / 175.0f + 0.5f);

    if (rh_i   > 65535u) rh_i   = 65535u;
    if (temp_i > 65535u) temp_i = 65535u;

    *rh_ticks   = (uint16_t)rh_i;
    *temp_ticks = (uint16_t)temp_i;
}

/**
 * @brief Issue SGP40 "Turn heater off" command.
 *
 * Best-effort helper to reduce power after a low-power step.
 */
static HAL_StatusTypeDef SGP40_TurnHeaterOff(I2C_HandleTypeDef *hi2c, uint32_t timeout_ms) {
    uint8_t cmd[2] = { 0x36, 0x15 };
    return HAL_I2C_Master_Transmit(hi2c, SGP40_I2C_ADDR, cmd, sizeof(cmd), timeout_ms);
}

/**
 * @brief One compensated raw measurement; heater is turned off after reading.
 *
 * This performs:
 *   1) RH/T measurement from SHT41 and conversion to ticks.
 *   2) Compensated SGP40 command with ignition + warm-up.
 *   3) Readout with CRC check.
 *   4) Heater-off command (best-effort).
 *
 * @param hi2c      I2C handle.
 * @param timeout_ms HAL I2C timeout in ms.
 * @param out_raw   Pointer to store raw SGP40 value.
 * @return HAL_OK on success, HAL_ERROR on any failure.
 */
HAL_StatusTypeDef SGP40_ReadRawComp(I2C_HandleTypeDef *hi2c, uint32_t timeout_ms, uint16_t *out_raw)
{
    if (!hi2c || !out_raw) return HAL_ERROR;

    /* 1) Measure RH/T for compensation */
    SHTMeasure(hi2c);
    float rh = SHT_data.rh;
    float tp = SHT_data.temperature;

    /* 2) Convert to SGP40 "ticks" */
    uint16_t rh_ticks, t_ticks;
    convRHTtoTicks(rh, tp, &rh_ticks, &t_ticks);

    uint8_t rh_bytes[2] = { (uint8_t)(rh_ticks >> 8), (uint8_t)(rh_ticks & 0xFF) };
    uint8_t t_bytes[2]  = { (uint8_t)(t_ticks  >> 8), (uint8_t)(t_ticks  & 0xFF) };

    uint8_t tx[8] = {
        0x26, 0x0F,
        rh_bytes[0], rh_bytes[1], sensirion_crc8(rh_bytes, 2),
        t_bytes[0],  t_bytes[1],  sensirion_crc8(t_bytes, 2)
    };

    /* 3) "Ignition" + warm-up (minimized average power) */
    if (HAL_I2C_Master_Transmit(hi2c, SGP40_I2C_ADDR, tx, sizeof(tx), timeout_ms) != HAL_OK) return HAL_ERROR;
    HAL_Delay(170);

    if (HAL_I2C_Master_Transmit(hi2c, SGP40_I2C_ADDR, tx, sizeof(tx), timeout_ms) != HAL_OK) return HAL_ERROR;
    HAL_Delay(30);

    uint8_t rx[3] = {0};
    if (HAL_I2C_Master_Receive(hi2c, SGP40_I2C_ADDR, rx, sizeof(rx), timeout_ms) != HAL_OK) return HAL_ERROR;
    if (sensirion_crc8(rx, 2) != rx[2]) return HAL_ERROR;

    (void)SGP40_TurnHeaterOff(hi2c, timeout_ms); // best effort

    *out_raw = (uint16_t)((rx[0] << 8) | rx[1]);
    return HAL_OK;
}

/**
 * @brief Compensated raw measurement with heater left ON between calls.
 *
 * First call performs warm-up and discards the first reading. Subsequent calls
 * assume a 1 Hz sampling pattern: one command, 30 ms delay, then read.
 *
 * @param hi2c       I2C handle.
 * @param timeout_ms HAL I2C timeout in ms.
 * @param out_raw    Pointer to store raw SGP40 value.
 * @return HAL_OK on success, HAL_ERROR on any failure.
 */
HAL_StatusTypeDef SGP40_ReadRawComp_HeaterOn(I2C_HandleTypeDef *hi2c,
                                             uint32_t timeout_ms,
                                             uint16_t *out_raw)
{
    if (!hi2c || !out_raw) return HAL_ERROR;

    static uint8_t warmed = 0;

    /* 1) Current RH/T for compensation */
    SHTMeasure(hi2c);
    float rh = SHT_data.rh;
    float tp = SHT_data.temperature;

    uint16_t rh_ticks, t_ticks;
    convRHTtoTicks(rh, tp, &rh_ticks, &t_ticks);

    uint8_t rh_bytes[2] = { (uint8_t)(rh_ticks >> 8), (uint8_t)(rh_ticks & 0xFF) };
    uint8_t t_bytes[2] = { (uint8_t)(t_ticks >> 8), (uint8_t)(t_ticks & 0xFF) };

    uint8_t tx[8] = {
        0x26, 0x0F,
        rh_bytes[0], rh_bytes[1], sensirion_crc8(rh_bytes, 2),
        t_bytes[0], t_bytes[1], sensirion_crc8(t_bytes, 2)
    };

    if (!warmed) {
        /* First cycle: ignite and warm up (discard reading) */
        if (HAL_I2C_Master_Transmit(hi2c, SGP40_I2C_ADDR, tx, sizeof(tx), timeout_ms) != HAL_OK) return HAL_ERROR;
        HAL_Delay(170);
        if (HAL_I2C_Master_Transmit(hi2c, SGP40_I2C_ADDR, tx, sizeof(tx), timeout_ms) != HAL_OK) return HAL_ERROR;
        HAL_Delay(30);

        uint8_t rx_warm[3] = {0};
        if (HAL_I2C_Master_Receive(hi2c, SGP40_I2C_ADDR, rx_warm, sizeof(rx_warm), timeout_ms) != HAL_OK) return HAL_ERROR;
        if (sensirion_crc8(rx_warm, 2) != rx_warm[2]) return HAL_ERROR;

        warmed = 1; // from now on we assume "steady-state" (heater stays on)
    }

    /* Steady-state 1 Hz: 1x command -> 30 ms delay -> read (heater remains hot) */
    if (HAL_I2C_Master_Transmit(hi2c, SGP40_I2C_ADDR, tx, sizeof(tx), timeout_ms) != HAL_OK) return HAL_ERROR;
    HAL_Delay(30);

    uint8_t rx[3] = {0};
    if (HAL_I2C_Master_Receive(hi2c, SGP40_I2C_ADDR, rx, sizeof(rx), timeout_ms) != HAL_OK) return HAL_ERROR;
    if (sensirion_crc8(rx, 2) != rx[2]) return HAL_ERROR;

    *out_raw = (uint16_t)((rx[0] << 8) | rx[1]);
    return HAL_OK;
}

/* One compensated measurement with minimized average power */
static HAL_StatusTypeDef SGP40_LowPowerStep(I2C_HandleTypeDef *hi2c, uint32_t timeout_ms)
{
    if (!hi2c || !g_voc_inited) return HAL_ERROR;

    SHTMeasure(hi2c);
    float rh = SHT_data.rh;
    float tp = SHT_data.temperature;

    uint16_t rh_ticks, t_ticks;
    convRHTtoTicks(rh, tp, &rh_ticks, &t_ticks);

    uint8_t rh_bytes[2] = { (uint8_t)(rh_ticks >> 8), (uint8_t)(rh_ticks & 0xFF) };
    uint8_t t_bytes[2]  = { (uint8_t)(t_ticks >> 8),  (uint8_t)(t_ticks & 0xFF) };

    uint8_t tx[8] = {
        0x26, 0x0F,
        rh_bytes[0], rh_bytes[1], sensirion_crc8(rh_bytes, 2),
        t_bytes[0],  t_bytes[1],  sensirion_crc8(t_bytes, 2)
    };

    /* ignition (ignore readout), then warm-up */
    if (HAL_I2C_Master_Transmit(hi2c, SGP40_I2C_ADDR, tx, sizeof(tx), timeout_ms) != HAL_OK) return HAL_ERROR;
    HAL_Delay(170);

    if (HAL_I2C_Master_Transmit(hi2c, SGP40_I2C_ADDR, tx, sizeof(tx), timeout_ms) != HAL_OK) return HAL_ERROR;
    HAL_Delay(30);

    uint8_t rx[3] = {0};
    if (HAL_I2C_Master_Receive(hi2c, SGP40_I2C_ADDR, rx, sizeof(rx), timeout_ms) != HAL_OK) return HAL_ERROR;
    if (sensirion_crc8(rx, 2) != rx[2]) return HAL_ERROR;

    (void)SGP40_TurnHeaterOff(hi2c, timeout_ms); /* best effort */

    uint16_t raw = (uint16_t)((rx[0] << 8) | rx[1]);
    int32_t voc_index32 = 0;
    GasIndexAlgorithm_process(&g_voc, (int32_t)raw, &voc_index32);
    if (voc_index32 < 0)      voc_index32 = 0;
    if (voc_index32 > 65535)  voc_index32 = 65535;

    sgp40_data.voc_raw   = raw;
    sgp40_data.voc_index = (uint16_t)voc_index32;

    return HAL_OK;
}

/* ===== Public API ===== */

/**
 * @brief Initialize Sensirion VOC Index algorithm for SGP40.
 *
 * @param sampling_interval_seconds Algorithm sampling interval in seconds.
 *                                  If <= 0, default 1 s is used.
 */
void SGP40_VOC_Init(float sampling_interval_seconds)
{
    if (sampling_interval_seconds <= 0.0f)
        sampling_interval_seconds = GasIndexAlgorithm_DEFAULT_SAMPLING_INTERVAL; /* 1 s */
    GasIndexAlgorithm_init_with_sampling_interval(
        &g_voc, GasIndexAlgorithm_ALGORITHM_TYPE_VOC, sampling_interval_seconds);
    g_voc_sampling_interval_s = sampling_interval_seconds;
    g_voc_inited = 1;
}

/**
 * @brief Start continuous VOC mode (low-power step driven externally).
 *
 * This only initializes the VOC algorithm (if needed) and marks the internal
 * engine as running. The caller must periodically call SGP40_VOC_ContStep()
 * at the configured sampling interval.
 */
HAL_StatusTypeDef SGP40_VOC_ContStart(I2C_HandleTypeDef *hi2c,
                                      uint32_t timeout_ms,
                                      float sampling_interval_seconds)
{
    (void)timeout_ms;
    if (!g_voc_inited || sampling_interval_seconds != g_voc_sampling_interval_s) {
        SGP40_VOC_Init(sampling_interval_seconds > 0.0f ? sampling_interval_seconds : 1.0f);
    }
    g_voc_running = 1;
    return HAL_OK;
}

/**
 * @brief Stop continuous VOC mode (does not issue any I2C commands).
 *
 * If you want an explicit heater-off at stop, extend the API to pass hi2c
 * and call SGP40_TurnHeaterOff() here.
 */
void SGP40_VOC_ContStop(void)
{
    g_voc_running = 0;
    /* If you want an explicit heater-off here, pass hi2c to this function and call SGP40_TurnHeaterOff(). */
}

/**
 * @brief Perform one low-power VOC step in continuous mode.
 *
 * If continuous mode was not started, this will auto-start it with a 1 s
 * sampling interval before performing the low-power step.
 */
HAL_StatusTypeDef SGP40_VOC_ContStep(I2C_HandleTypeDef *hi2c, uint32_t timeout_ms)
{
    if (!g_voc_running) {
        (void)SGP40_VOC_ContStart(hi2c, timeout_ms, 1.0f);
    }
    return SGP40_LowPowerStep(hi2c, timeout_ms);
}

/**
 * @brief Single compensated VOC measurement into sgp40_data.
 *
 * This helper:
 *   - Ensures VOC algorithm is initialized (1 s interval if not).
 *   - Takes one compensated SGP40 sample.
 *   - Feeds it into the VOC index algorithm.
 *   - Updates sgp40_data.voc_raw and sgp40_data.voc_index.
 */
HAL_StatusTypeDef SGP40_MeasureOnce(I2C_HandleTypeDef *hi2c, uint32_t timeout_ms)
{
    if (!hi2c) return HAL_ERROR;
    if (!g_voc_inited) SGP40_VOC_Init(1.0f);

    SHTMeasure(hi2c);
    float rh = SHT_data.rh;
    float tp = SHT_data.temperature;

    uint16_t rh_ticks, t_ticks;
    convRHTtoTicks(rh, tp, &rh_ticks, &t_ticks);

    uint8_t rh_bytes[2] = { (uint8_t)(rh_ticks >> 8), (uint8_t)(rh_ticks & 0xFF) };
    uint8_t t_bytes[2]  = { (uint8_t)(t_ticks >> 8),  (uint8_t)(t_ticks & 0xFF) };

    uint8_t tx[8] = {
        0x26, 0x0F,
        rh_bytes[0], rh_bytes[1], sensirion_crc8(rh_bytes, 2),
        t_bytes[0],  t_bytes[1],  sensirion_crc8(t_bytes, 2)
    };

    if (HAL_I2C_Master_Transmit(hi2c, SGP40_I2C_ADDR, tx, sizeof(tx), timeout_ms) != HAL_OK) return HAL_ERROR;
    HAL_Delay(30);
    uint8_t rx[3] = {0};
    if (HAL_I2C_Master_Receive(hi2c, SGP40_I2C_ADDR, rx, sizeof(rx), timeout_ms) != HAL_OK) return HAL_ERROR;
    if (sensirion_crc8(rx, 2) != rx[2]) return HAL_ERROR;

    uint16_t raw = (uint16_t)((rx[0] << 8) | rx[1]);
    int32_t voc_index32 = 0;
    GasIndexAlgorithm_process(&g_voc, (int32_t)raw, &voc_index32);
    if (voc_index32 < 0)      voc_index32 = 0;
    if (voc_index32 > 65535)  voc_index32 = 65535;

    sgp40_data.voc_raw   = raw;
    sgp40_data.voc_index = (uint16_t)voc_index32;

    return HAL_OK;
}
