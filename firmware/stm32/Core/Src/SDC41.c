/*
 * SDC41.c
 *
 *  Created on: Oct 24, 2024
 *      Author: Evzen Steif
 *
 *  @brief Driver helpers for the Sensirion SCD41 CO2 sensor.
 *
 *  This module provides:
 *    - I2C command helpers and raw data reads.
 *    - Single-shot and averaged single-shot measurements.
 *    - CRC8 calculation/verification for SCD41 frames.
 *    - Automatic self-calibration (ASC) enable/disable helpers.
 *    - Factory reset, serial read, and FRC (forced recalibration) flow
 *      with progress reporting on the E-Ink display.
 */

#include "stm32u0xx_hal.h"
#include "SDC41.h"
#include "SHT41.h"
#include "stdio.h"
#include <string.h>
#include "GUI.h"

SCD41_Data SCD_data;
SCD41_CalibrationStatus SCD_calib = {0};

/**
 * @brief Initialize SCD41 into a known state.
 *
 * Stops periodic measurement, wakes the device and disables ASC so that the
 * application can operate with a fixed calibration or explicit FRC.
 */
void SCD41_Init(I2C_HandleTypeDef *hi2c)
{
    printf("SCD41 init...\r\n");

    SCD41_StopPeriodicMeasurement(hi2c);
    HAL_Delay(500);

    SCD41_WakeUp(hi2c);
    HAL_Delay(30);

    SCD41_ASC(hi2c, ASC_DISABLE);
    HAL_Delay(1);
}

/**
 * @brief Send a 16-bit command to SCD41 via I2C.
 */
void SCD41_SendCommand(I2C_HandleTypeDef *hi2c, uint16_t command)
{
    uint8_t cmd[2];
    cmd[0] = (command >> 8) & 0xFF;
    cmd[1] = command & 0xFF;

    HAL_I2C_Master_Transmit(hi2c, SCD41_I2C_ADDRESS << 1, cmd, 2, HAL_MAX_DELAY);
}

/**
 * @brief Read raw bytes from SCD41 via I2C.
 *
 * @param data  Pointer to buffer for received data.
 * @param len   Number of bytes to read.
 */
void SCD41_ReadData(I2C_HandleTypeDef *hi2c, uint8_t *data, uint16_t len)
{

	  HAL_I2C_Master_Receive(hi2c, SCD41_I2C_ADDRESS << 1, data, len, HAL_MAX_DELAY);

}

/**
 * @brief Query SCD41 for data-ready status.
 *
 * @return 1 if new data is available, 0 otherwise.
 */
uint8_t DataReadyStatus(I2C_HandleTypeDef *hi2c)
{
    uint8_t data[3];
    SCD41_SendCommand(hi2c, SCD41_CMD_DATA_READY);
    HAL_Delay(1);
    SCD41_ReadData(hi2c, data, 3);

    uint8_t crc = sensirion_common_generate_crc(data, 2);
    if (crc != data[2]) return 0;

    uint16_t report = ((uint16_t)data[0] << 8 | data[1]) & 0x07FF;
    return (report != 0);
}

/**
 * @brief Enable or disable SCD41 Automatic Self-Calibration (ASC).
 */
void SCD41_ASC(I2C_HandleTypeDef *hi2c, ASC_Status status)
{
    uint8_t cmd[5];
    cmd[0] = (SCD41_CMD_ASC_ENABLE >> 8) & 0xFF;
    cmd[1] = SCD41_CMD_ASC_ENABLE & 0xFF;
    cmd[2] = 0x00;
    cmd[3] = (status == ASC_EN) ? 0x01 : 0x00;
    cmd[4] = sensirion_common_generate_crc(&cmd[2], 2);

    HAL_I2C_Master_Transmit(hi2c, SCD41_I2C_ADDRESS << 1, cmd, 5, HAL_MAX_DELAY);
}

/**
 * @brief Perform a single-shot measurement with basic retry and CRC checking.
 *
 * Tries up to MAX_ATTEMPTS times if data-ready timeout or CRC error occurs.
 * On success, updates SCD_data. On repeated failure, SCD_data is zeroed.
 */
void SCD41_SingleMeasurement(I2C_HandleTypeDef *hi2c)
{
    uint8_t data[9];
    uint8_t attempts = 0;
    const uint8_t MAX_ATTEMPTS = 3;   // how many times to retry on timeout/CRC error

    while (attempts < MAX_ATTEMPTS)
    {
        attempts++;

        // Start single-shot measurement
        SCD41_SendCommand(hi2c, SCD41_CMD_MEASURE_SINGLE_SHOT);

        // Wait for DataReady (same logic as in averaged version)
        uint8_t wait_count = 0;
        while (!DataReadyStatus(hi2c) && wait_count < SCD41_MAX_WAIT_COUNT)
        {
            HAL_Delay(100);
            wait_count++;
        }

        if (wait_count >= SCD41_MAX_WAIT_COUNT)
        {
            printf("Timeout: data not ready (attempt %u/%u)\r\n", attempts, MAX_ATTEMPTS);
            continue;   // retry from the beginning
        }

        // Read measurement result
        SCD41_SendCommand(hi2c, SCD41_CMD_READ_MEASUREMENT);
        HAL_Delay(1);
        SCD41_ReadData(hi2c, data, 9);

        // CRC calculation and verification (same as in averaged version)
        uint8_t co2_crc_calc  = sensirion_common_generate_crc(&data[0], 2);
        uint8_t temp_crc_calc = sensirion_common_generate_crc(&data[3], 2);
        uint8_t rh_crc_calc   = sensirion_common_generate_crc(&data[6], 2);

        uint8_t crc_ok = verify_crc(co2_crc_calc, data[2]) &&
                         verify_crc(temp_crc_calc, data[5]) &&
                         verify_crc(rh_crc_calc, data[8]);

        if (!crc_ok)
        {
            printf("CRC error (attempt %u/%u)\r\n", attempts, MAX_ATTEMPTS);
            continue;   // retry entire measurement
        }

        // Decode raw values
        uint16_t co2_raw         = (data[0] << 8) | data[1];
        uint16_t temperature_raw = (data[3] << 8) | data[4];
        uint16_t humidity_raw    = (data[6] << 8) | data[7];

        // Convert to engineering units
        SCD_data.co2         = (float)co2_raw;
        SCD_data.temperature = -45.0f + 175.0f * ((float)temperature_raw / 65535.0f);
        SCD_data.humidity    = 100.0f * ((float)humidity_raw / 65535.0f);

        // Success – leave function
        return;
    }

    // If all attempts fail, use zero values (or you could keep previous readings)
    printf("SingleMeasurement failed after %u attempts. Using zeroes.\r\n", MAX_ATTEMPTS);
    SCD_data.co2 = 0.0f;
    SCD_data.temperature = 0.0f;
    SCD_data.humidity = 0.0f;
}

/**
 * @brief Perform multiple single-shot measurements and average valid samples.
 *
 * Takes SCD41_AVG_COUNT samples. Each sample waits for data-ready and passes
 * through CRC checks. Only valid samples are accumulated; on success the
 * average is saved to SCD_data. If none are valid, SCD_data is zeroed.
 */
void SCD41_SingleMeasurement_Averaged(I2C_HandleTypeDef *hi2c)
{
    float co2_sum = 0.0f;
    float temp_sum = 0.0f;
    float rh_sum = 0.0f;
    uint8_t valid_count = 0;

    for (uint8_t i = 0; i < SCD41_AVG_COUNT; i++)
    {
        SCD41_SendCommand(hi2c, SCD41_CMD_MEASURE_SINGLE_SHOT);

        // Wait for DataReady (max 5 s)
        uint8_t wait_count = 0;
        while (!DataReadyStatus(hi2c) && wait_count < SCD41_MAX_WAIT_COUNT )
        {
            HAL_Delay(100);
            wait_count++;
        }

        if (wait_count >= SCD41_MAX_WAIT_COUNT )
        {
            printf("Timeout: data not ready for sample %u\r\n", i + 1);
            continue;
        }

       // HAL_Delay(5000);

        uint8_t data[9];
        SCD41_SendCommand(hi2c, SCD41_CMD_READ_MEASUREMENT);
        HAL_Delay(1);
        SCD41_ReadData(hi2c, data, 9);

        // CRC checks
        uint8_t co2_crc_calc  = sensirion_common_generate_crc(&data[0], 2);
        uint8_t temp_crc_calc = sensirion_common_generate_crc(&data[3], 2);
        uint8_t rh_crc_calc   = sensirion_common_generate_crc(&data[6], 2);

        uint8_t crc_ok = verify_crc(co2_crc_calc, data[2]) &&
                         verify_crc(temp_crc_calc, data[5]) &&
                         verify_crc(rh_crc_calc, data[8]);

        if (crc_ok)
        {
            uint16_t co2_raw        = (data[0] << 8) | data[1];
            uint16_t temperature_raw = (data[3] << 8) | data[4];
            uint16_t humidity_raw    = (data[6] << 8) | data[7];

            float co2 = (float)co2_raw;
            float temperature = -45.0f + 175.0f * ((float)temperature_raw / 65535.0f);
            float humidity    = 100.0f * ((float)humidity_raw / 65535.0f);

            co2_sum  += co2;
            temp_sum += temperature;
            rh_sum   += humidity;
            valid_count++;
        }
        else
        {
            printf("CRC error on sample %u\r\n", i + 1);
        }
    }

    if (valid_count > 0)
    {
        SCD_data.co2        = co2_sum  / valid_count;
        SCD_data.temperature = temp_sum / valid_count;
        SCD_data.humidity    = rh_sum   / valid_count;
    }
    else
    {
        printf("No valid samples. Using zeroes.\r\n");
        SCD_data.co2 = 0;
        SCD_data.temperature = 0;
        SCD_data.humidity = 0;
    }
}

/**
 * @brief Wake the SCD41 from power-down/sleep.
 */
void SCD41_WakeUp(I2C_HandleTypeDef *hi2c)
{
	SCD41_SendCommand(hi2c, SCD41_CMD_WAKE_UP);
}

/**
 * @brief Put the SCD41 into power-down mode.
 */
void SCD41_PowerDown(I2C_HandleTypeDef *hi2c)
{
	SCD41_SendCommand(hi2c, SCD41_CMD_POWER_DOWN);
}

/**
 * @brief Calculate Sensirion CRC-8 for given data buffer.
 *
 * Polynomial and init value follow SCD41 documentation.
 */
uint8_t sensirion_common_generate_crc(const uint8_t* data, uint16_t count)
{
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;

    for (current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

/**
 * @brief Compare calculated and received CRC bytes.
 *
 * @return 1 if CRC matches, 0 otherwise.
 */
uint8_t verify_crc(uint8_t calculated_crc, uint8_t received_crc)
{
	return (calculated_crc == received_crc);
}

/**
 * @brief Read and print the 3-word SCD41 serial number.
 */
void SCD41_ReadSerial(I2C_HandleTypeDef *hi2c)
{
    uint8_t data[9];
    SCD41_SendCommand(hi2c, SCD41_CMD_READ_SERIAL_NUMBER);
    HAL_Delay(1);
    SCD41_ReadData(hi2c, data, 9);

    uint16_t serial1 = (data[0]<<8) | data[1];
    uint16_t serial2 = (data[3]<<8) | data[4];
    uint16_t serial3 = (data[6]<<8) | data[7];

    printf("Serial: %04X %04X %04X\r\n", serial1, serial2, serial3);
}

/**
 * @brief Start SCD41 periodic measurement mode.
 */
void SCD41_StartPeriodicMeasurement(I2C_HandleTypeDef *hi2c) {
    SCD41_SendCommand(hi2c, 0x21B1);  // Start periodic measurement
    HAL_Delay(1);
}

/**
 * @brief Stop SCD41 periodic measurement mode.
 */
void SCD41_StopPeriodicMeasurement(I2C_HandleTypeDef *hi2c) {
    SCD41_SendCommand(hi2c, 0x3F86);  // Stop periodic measurement
}

/**
 * @brief Perform SCD41 factory reset and wait until sensor restarts.
 */
void SCD41_PerformFactoryReset(I2C_HandleTypeDef *hi2c)
{
    printf("Performing factory reset...\r\n");
    SCD41_SendCommand(hi2c, 0x3632);  // Factory reset command
    HAL_Delay(1200);
    printf("Factory reset done. Sensor restarted.\r\n");
}

/**
 * @brief Run SCD41 Forced Recalibration (FRC) to a known reference concentration.
 *
 * Shows progress on the E-Ink display for the 3-minute stabilization period,
 * then sends FRC command, reads back offset and updates SCD_calib with the
 * result (including a human-readable result_msg).
 */
void SCD41_ForceRecalibration(I2C_HandleTypeDef *hi2c, uint16_t co2_reference_ppm)
{
    SCD_calib.reference_ppm = co2_reference_ppm;
    SCD_calib.progress_percent = 0;
    memset(SCD_calib.result_msg, 0, sizeof(SCD_calib.result_msg));
    SCD_calib.frc_success = 0;
    SCD_calib.frc_offset = 0xFFFF;

    printf("FRC start: waiting 3 min in periodic mode...\r\n");
    // Start periodic measurement (if not already started)
    SCD41_SendCommand(hi2c, 0x21B1);
    HAL_Delay(1000);

    EPDClear();
    GUI_UpdateState(app.currentState);

    SCD_calib.progress_percent = 1;

    for (int i = 0; i <= 180; i++) {
        SCD_calib.progress_percent = (i * 100) / 180;
        GUI_DrawCalibrationProgressFromStruct();
        HAL_Delay(1000);
    }

    // After the 3-minute run:
    SCD41_SendCommand(hi2c, 0x3F86); // Stop periodic
    HAL_Delay(500);

    uint8_t cmd[5];
    cmd[0] = (SCD41_CMD_FRC >> 8) & 0xFF;
    cmd[1] = SCD41_CMD_FRC & 0xFF;
    cmd[2] = (co2_reference_ppm >> 8) & 0xFF;
    cmd[3] = co2_reference_ppm & 0xFF;
    cmd[4] = sensirion_common_generate_crc(&cmd[2], 2);

    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(hi2c, SCD41_I2C_ADDRESS << 1, cmd, 5, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        snprintf(SCD_calib.result_msg, sizeof(SCD_calib.result_msg), "FRC send failed");
        SCD_calib.frc_success = 0;
        GUI_DrawCalibrationProgressFromStruct();
        return;
    }

    HAL_Delay(400);

    uint8_t data[3];
    SCD41_SendCommand(hi2c, 0xE020);
    HAL_Delay(1);
    SCD41_ReadData(hi2c, data, 3);

    uint8_t crc = sensirion_common_generate_crc(data, 2);
    if (crc != data[2]) {
        snprintf(SCD_calib.result_msg, sizeof(SCD_calib.result_msg), "FRC CRC error");
        SCD_calib.frc_success = 0;
        GUI_DrawCalibrationProgressFromStruct();
        return;
    }

    uint16_t frc_offset = (data[0] << 8) | data[1];
    SCD_calib.frc_offset = frc_offset;

    if (frc_offset == 0xFFFF) {
        snprintf(SCD_calib.result_msg, sizeof(SCD_calib.result_msg), "FRC failed");
        SCD_calib.frc_success = 0;
    } else {
        snprintf(SCD_calib.result_msg, sizeof(SCD_calib.result_msg), "OK: offset %u", frc_offset);
        SCD_calib.frc_success = 1;
    }

    SCD_calib.progress_percent = 100;
    EPDClear();
    GUI_DrawCalibrationProgressFromStruct();
    HAL_Delay(2500);
    EPDClear();
}

/**
 * @brief Persist current SCD41 configuration to EEPROM (ASC, etc.).
 */
void SCD41_PersistSettings(I2C_HandleTypeDef *hi2c)
{
    SCD41_SendCommand(hi2c, SCD41_CMD_PERSIST_SETTINGS);
    HAL_Delay(5);
}

/**
 * @brief Disable ASC and persist this setting to SCD41 non-volatile memory.
 *
 * Ensures the device is awake and out of periodic mode before modifying and
 * persisting the ASC configuration.
 */
void SCD41_PersistASCDisable(I2C_HandleTypeDef *hi2c)
{
    /* Ensure device is awake */
    SCD41_WakeUp(hi2c);
    HAL_Delay(30);

    /* Enter idle: stop periodic measurement (required before some commands) */
    SCD41_StopPeriodicMeasurement(hi2c);
    HAL_Delay(500);

    /* Disable ASC and persist to EEPROM */
    SCD41_ASC(hi2c, ASC_DISABLE);
    HAL_Delay(2);
    SCD41_PersistSettings(hi2c);

    printf("SCD41: ASC disabled permanently (persisted, periodic stopped).\r\n");
}
