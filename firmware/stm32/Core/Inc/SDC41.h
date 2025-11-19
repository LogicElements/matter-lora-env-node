/*
 * SDC41.h
 *
 *  Created on: Oct 24, 2024
 *      Author: Evzen Steif
 *
 *  @brief Public interface for Sensirion SCD41 CO2 sensor helpers.
 *
 *  This header defines:
 *    - SCD41 register/command constants.
 *    - Data structures for latest measurement and calibration status.
 *    - APIs for single-shot/averaged measurements, ASC control,
 *      factory reset, FRC, CRC helpers and persistence of settings.
 */

#ifndef INC_SDC41_H_
#define INC_SDC41_H_

#define SCD41_AVG_COUNT        3
#define SCD41_MAX_WAIT_COUNT   50

#define SCD41_I2C_ADDRESS              0x62
#define SCD41_CMD_MEASURE_SINGLE_SHOT  0x219d
#define SCD41_CMD_STOP_MEASUREMENT     0x3f86
#define SCD41_CMD_READ_MEASUREMENT     0xEC05
#define SCD41_CMD_WAKE_UP              0x36f6
#define SCD41_CMD_POWER_DOWN           0x36e0
#define SCD41_CMD_DATA_READY           0xe4b8
#define SCD41_CMD_ASC_ENABLE           0x2416
#define CRC8_POLYNOMIAL                0x31
#define CRC8_INIT                      0xFF
#define SCD41_CMD_READ_SERIAL_NUMBER   0x3682
#define SCD41_CMD_FRC                  0x362F
#define SCD41_CMD_PERSIST_SETTINGS     0x3615

/**
 * @brief Automatic Self-Calibration (ASC) mode selection.
 */
typedef enum{
	ASC_EN,
	ASC_DISABLE
} ASC_Status;

/**
 * @brief Latest SCD41 measurement in engineering units.
 *
 * co2         - CO2 concentration in ppm (raw integer from sensor).
 * temperature - Temperature in °C.
 * humidity    - Relative humidity in %RH.
 */
typedef struct {
    uint16_t co2;
    float    temperature;
    float    humidity;
} SCD41_Data;

/**
 * @brief State of a running/finished SCD41 Forced Recalibration (FRC).
 *
 * progress_percent - 0..100% progress for UI.
 * reference_ppm    - CO2 reference used for FRC.
 * frc_offset       - Offset reported by sensor (0xFFFF indicates failure).
 * frc_success      - 1 if FRC was successful, 0 otherwise.
 * result_msg       - Short human-readable status message.
 */
typedef struct {
    uint8_t  progress_percent;
    uint16_t reference_ppm;
    uint16_t frc_offset;
    uint8_t  frc_success;
    char     result_msg[32];
} SCD41_CalibrationStatus;

extern SCD41_Data             SCD_data;
extern SCD41_CalibrationStatus SCD_calib;

/* Basic control and measurement */
void SCD41_Init(I2C_HandleTypeDef *hi2c);
void SCD41_StopPeriodicMeasurement(I2C_HandleTypeDef *hi2c);
void SCD41_SendCommand(I2C_HandleTypeDef *hi2c, uint16_t command);
void SCD41_ReadData(I2C_HandleTypeDef *hi2c, uint8_t *data, uint16_t len);

void SCD41_SingleMeasurement(I2C_HandleTypeDef *hi2c);
void SCD41_SingleMeasurement_Averaged(I2C_HandleTypeDef *hi2c);

/* ASC and power control */
void SCD41_ASC(I2C_HandleTypeDef *hi2c, ASC_Status status);
void SCD41_WakeUp(I2C_HandleTypeDef *hi2c);
void SCD41_PowerDown(I2C_HandleTypeDef *hi2c);

/* Device information and maintenance */
void SCD41_ReadSerial(I2C_HandleTypeDef *hi2c);
void SCD41_ForceRecalibration(I2C_HandleTypeDef *hi2c, uint16_t co2_reference_ppm);
void SCD41_PerformFactoryReset(I2C_HandleTypeDef *hi2c);

/* CRC helpers */
uint8_t sensirion_common_generate_crc(const uint8_t* data, uint16_t count);
uint8_t verify_crc(uint8_t calculated_crc, uint8_t received_crc);

/* Non-volatile settings */
void SCD41_PersistSettings(I2C_HandleTypeDef *hi2c);
void SCD41_PersistASCDisable(I2C_HandleTypeDef *hi2c);

#endif /* INC_SDC41_H_ */
