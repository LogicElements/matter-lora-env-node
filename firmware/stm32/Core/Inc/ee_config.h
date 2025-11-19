/*
 * ee_config.h
 *
 *  Created on: Nov 14, 2025
 *      Author: evzen
 */

#ifndef INC_EE_CONFIG_H_
#define INC_EE_CONFIG_H_

#include "stm32u0xx_hal.h"
#include "app.h"
#include "WioE5.h"
#include "esp32c6.h"
#include "m24c02.h"

/**
 * @brief Načti konfiguraci z EEPROM (M24C02).
 *
 * Načítá:
 *  - AppConfig_t (device-level config)
 *  - WioOtaaConfig_t (LoRa OTAA klíče)
 *  - EspC6Pairing_t (QR + manual)
 *
 * Pokud jsou data nevalidní (magic/version/length/CRC),
 * naplní:
 *  - cfg -> firmware defaulty (AppConfig_SetDefaults)
 *  - wio -> nuly
 *  - esp_pairing -> vynuluje
 *
 * @param hi2c I2C handle pro EEPROM (M24C02).
 */
HAL_StatusTypeDef EE_Config_Load(I2C_HandleTypeDef *hi2c,
                                 AppConfig_t        *cfg,
                                 WioOtaaConfig_t    *wio,
                                 EspC6Pairing_t     *esp_pairing);

/**
 * @brief Ulož aktuální konfiguraci do EEPROM.
 *
 * Ukládá:
 *  - AppConfig_t
 *  - WioOtaaConfig_t
 *  - EspC6Pairing_t
 *
 * @param hi2c I2C handle pro EEPROM (M24C02).
 */
HAL_StatusTypeDef EE_Config_Save(I2C_HandleTypeDef      *hi2c,
                                 const AppConfig_t      *cfg,
                                 const WioOtaaConfig_t  *wio,
                                 const EspC6Pairing_t   *esp_pairing);

#endif /* INC_EE_CONFIG_H_ */
