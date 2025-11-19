#include "button_handler.h"
#include "stm32u0xx_hal.h"
#include "main.h"
#include "app.h"
#include "RTC.h"

/* External handles */
extern RTC_HandleTypeDef hrtc;

/* Button wiring (active-LOW) */
#define BUTTON_GPIO_PORT   BUTTON_GPIO_Port
#define BUTTON_GPIO_PIN    BUTTON_Pin

/* Internal state */
static uint32_t press_start_time = 0u;
static uint8_t  button_down      = 0u;  /* 1 = currently pressed (active-low) */

/* ---------------- Public API ---------------- */

void ButtonHandler_Init(void)
{
    /* Configure EXTI on the button (rising & falling) */
    GPIO_InitTypeDef gi = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE(); /* adjust if your button is not on port C */

    gi.Pin  = BUTTON_GPIO_PIN;
    gi.Mode = GPIO_MODE_IT_RISING_FALLING;
    gi.Pull = GPIO_NOPULL;                /* use external PU if needed */
    HAL_GPIO_Init(BUTTON_GPIO_PORT, &gi);

    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* If the device woke up due to WKUP7 (or the user is simply holding the button),
   start measuring immediately at boot. Works even after Shutdown because we only
   ACT on release (rising edge). */
void ButtonHandler_BootCapture(void)
{
    GPIO_PinState s = HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN);
    if (s == GPIO_PIN_RESET) {                 /* active-LOW → pressed */
        press_start_time = RTC_GetSecondsOfDay();
        button_down      = 1u;
    } else {
        button_down      = 0u;
    }
}

uint8_t IsButtonReleased(void)
{
    return (HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN) == GPIO_PIN_SET) ? 1u : 0u;
}

/* Call from both rising & falling EXTI callbacks for BUTTON_Pin */
void ButtonHandler_EXTI_Callback(void)
{
    GPIO_PinState s = HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN);

    if (s == GPIO_PIN_RESET) {
        /* Falling edge → pressed */
        press_start_time = RTC_GetSecondsOfDay();
        button_down      = 1u;
        return;
    }

    /* Rising edge → released; only here we evaluate duration & act */
    if (button_down) {
        uint32_t end = RTC_GetSecondsOfDay();
        button_down  = 0u;

        /* Handle wrap around midnight */
        uint32_t duration = (end >= press_start_time)
                          ? (end - press_start_time)
                          : (end + 86400u - press_start_time);

        if (duration >= RESET_TIME_S) {
            NVIC_SystemReset();
        } else if (duration >= FORCE_RECALIBRATION_TIME_S) {
            App_UserButtonIRQ_15s();
        } else if (duration >= CONFIG_TIME_S) {
            App_UserButtonIRQ_5s();   /* toggles CONFIG (confFlag++) */
        } else {
            /* short press: ignore or map to a feature */
        }
    }
}
