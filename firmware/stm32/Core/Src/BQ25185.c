/*
 * BQ25185.c
 *
 *  Created on: Sep 10, 2025
 *      Author: Evzen Steif
 */


#include "bq25185.h"
#include "app.h"

/**
 * @brief Detect if battery is missing.
 * STAT1 must be HIGH, while STAT2 toggles HIGH/LOW periodically.
 * Blocking version using HAL_Delay.
 * @retval 1 if battery missing, 0 otherwise.
 */
static uint8_t BQ25185_IsBatteryMissing(void)
{
    if (HAL_GPIO_ReadPin(STAT1_GPIO_Port, STAT1_Pin) == GPIO_PIN_RESET) {
        return 0;
    }

    uint8_t prev = HAL_GPIO_ReadPin(STAT2_GPIO_Port, STAT2_Pin);
    for (int i = 0; i < 5; i++) {
        HAL_Delay(10);
        uint8_t cur = HAL_GPIO_ReadPin(STAT2_GPIO_Port, STAT2_Pin);
        if (cur != prev) {
            return 1; // STAT2 toggles -> battery missing
        }
    }
    return 0;
}

/**
 * @brief Reads charger status and updates global app context.
 * @retval Current charger status (enum).
 */
BQ25185_Status_t BQ25185_GetStatus(void)
{
    BQ25185_Status_t status;

    if (app.usb_on == 0) {
        status = BQ25185_STATUS_USB_NOT_CONNECTED;
    }
    else if (BQ25185_IsBatteryMissing()) {
        status = BQ25185_STATUS_BATTERY_MISSING;
    }
    else {
        uint8_t stat1 = HAL_GPIO_ReadPin(STAT1_GPIO_Port, STAT1_Pin);
        uint8_t stat2 = HAL_GPIO_ReadPin(STAT2_GPIO_Port, STAT2_Pin);

        if (stat1 && stat2) {
            status = BQ25185_STATUS_CHARGE_DONE;
        } else if (stat1 && !stat2) {
            status = BQ25185_STATUS_CHARGING;
        } else if (!stat1 && stat2) {
            status = BQ25185_STATUS_RECOVERABLE_FAULT;
        } else if (!stat1 && !stat2) {
            status = BQ25185_STATUS_FATAL_FAULT;
        } else {
            status = BQ25185_STATUS_UNKNOWN;
        }
    }

    app.charger_status = status;
    return status;
}

/**
 * @brief Convert charger status enum to string.
 * @param status Charger status enum.
 * @retval Pointer to string literal.
 */
const char* BQ25185_StatusToString(BQ25185_Status_t status)
{
    switch (status) {
		case BQ25185_STATUS_USB_NOT_CONNECTED: return "No USB";
		case BQ25185_STATUS_CHARGE_DONE:       return "Charged";
		case BQ25185_STATUS_CHARGING:          return "Charging";
		case BQ25185_STATUS_RECOVERABLE_FAULT: return "Recov. fault";
		case BQ25185_STATUS_FATAL_FAULT:       return "Fatal fault";
		case BQ25185_STATUS_BATTERY_MISSING:   return "Missing";
        case BQ25185_STATUS_UNKNOWN:
        default:                               return "Unknown";
    }
}

/**
 * @brief Print current charger status using printf.
 */
void BQ25185_PrintStatus(void)
{
    printf("BQ25185 status: %s\r\n", BQ25185_StatusToString(app.charger_status));
}

/**
 * @brief Configure STAT1/STAT2 GPIOs depending on USB presence.
 * enable=1  -> Input floating (no internal pull)  [external pull-ups/LEDs present]
 * enable=0  -> Analog/Hi-Z (lowest leakage for sleep)
 */
void BQ25185_GPIO_Config(uint8_t enable)
{
    GPIO_InitTypeDef gi = {0};

    if (enable) {
        /* Input FLOATING: use external pull-ups/LED network */
        gi.Mode  = GPIO_MODE_INPUT;
        gi.Pull  = GPIO_NOPULL;           // << no internal pull-ups
        gi.Speed = GPIO_SPEED_FREQ_LOW;

        gi.Pin = STAT1_Pin;
        HAL_GPIO_Init(STAT1_GPIO_Port, &gi);

        gi.Pin = STAT2_Pin;
        HAL_GPIO_Init(STAT2_GPIO_Port, &gi);
    } else {
        /* Analog/Hi-Z for minimum leakage in sleep */
        gi.Mode  = GPIO_MODE_ANALOG;
        gi.Pull  = GPIO_NOPULL;

        gi.Pin = STAT1_Pin;
        HAL_GPIO_Init(STAT1_GPIO_Port, &gi);

        gi.Pin = STAT2_Pin;
        HAL_GPIO_Init(STAT2_GPIO_Port, &gi);
    }
    HAL_Delay(1);
}

