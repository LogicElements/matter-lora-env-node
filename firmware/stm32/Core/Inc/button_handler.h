#pragma once
#include <stdint.h>

/* Thresholds (seconds) */
#define RESET_TIME_S                   30u  /* >=30s: system reset */
#define FORCE_RECALIBRATION_TIME_S     15u  /* 15..29s: sensor recalibration */
#define CONFIG_TIME_S                   5u  /* 5..14s: enter/exit CONFIG */

/* Init EXTI on the button pin. */
void ButtonHandler_Init(void);

/* Called once at boot: if the button is already held, start timing now. */
void ButtonHandler_BootCapture(void);

/* Returns 1 if button is currently released (inactive), 0 if pressed. */
uint8_t IsButtonReleased(void);

/* Call from both HAL_GPIO_EXTI_Rising_Callback/Falling for BUTTON_Pin. */
void ButtonHandler_EXTI_Callback(void);
