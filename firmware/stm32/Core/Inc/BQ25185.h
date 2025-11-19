/*
 * BQ25185.h
 *
 *  Created on: Sep 10, 2025
 *      Author: Evzen Steif
 */

#ifndef __BQ25185_H
#define __BQ25185_H

#include "stm32u0xx_hal.h"
#include <stdint.h>
#include <stdio.h>

// Charger status enumeration
typedef enum {
    BQ25185_STATUS_USB_NOT_CONNECTED = 0,
    BQ25185_STATUS_CHARGE_DONE,
    BQ25185_STATUS_CHARGING,
    BQ25185_STATUS_RECOVERABLE_FAULT,
    BQ25185_STATUS_FATAL_FAULT,
    BQ25185_STATUS_BATTERY_MISSING,
    BQ25185_STATUS_UNKNOWN
} BQ25185_Status_t;

// Reads STAT1/STAT2 and updates app.charger_status
BQ25185_Status_t BQ25185_GetStatus(void);

// Converts enum status to a human-readable string
const char* BQ25185_StatusToString(BQ25185_Status_t status);

// Prints current status using printf
void BQ25185_PrintStatus(void);

void BQ25185_GPIO_Config(uint8_t enable);

#endif // __BQ25185_H
