#include "gpio_sleep.h"
#include "stm32u0xx_hal.h"
#include "main.h"

static const KeepPin* s_keep = NULL;
static size_t s_keep_count = 0;

void GPIO_Sleep_SetKeepPins(const KeepPin* pins, size_t count)
{
    s_keep = pins;
    s_keep_count = count;
}

void GPIO_EnableAllClocks(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
}

uint32_t build_keep_mask(GPIO_TypeDef* port)
{
    uint32_t mask = 0u;
    if (s_keep && s_keep_count) {
        for (size_t i = 0; i < s_keep_count; ++i) {
            if (s_keep[i].port == port) mask |= s_keep[i].pin;
        }
    }
    /* Auto-keep LSE pins if LSE is running (PC14/PC15 on STM32U0) */
    if (port == GPIOC && __HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY)) {
        mask |= GPIO_PIN_14 | GPIO_PIN_15;
    }
    return mask;
}

void port_to_analog_except(GPIO_TypeDef* port, uint32_t keep_mask)
{
    GPIO_InitTypeDef gi = {0};
    gi.Mode  = GPIO_MODE_ANALOG;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;

    for (uint32_t pin = 0x0001u; pin <= GPIO_PIN_15; pin <<= 1u) {
        if ((keep_mask & pin) == 0u) {
            gi.Pin = pin;
            HAL_GPIO_Init(port, &gi);
        }
    }
}

void GPIO_PrepareForSleep_Simple(void)
{
    GPIO_EnableAllClocks();

    port_to_analog_except(GPIOA, build_keep_mask(GPIOA));
    port_to_analog_except(GPIOB, build_keep_mask(GPIOB));
    port_to_analog_except(GPIOC, build_keep_mask(GPIOC));
    port_to_analog_except(GPIOD, build_keep_mask(GPIOD));
    port_to_analog_except(GPIOE, build_keep_mask(GPIOE));
    port_to_analog_except(GPIOF, build_keep_mask(GPIOF));

    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOD_CLK_DISABLE();
    __HAL_RCC_GPIOF_CLK_DISABLE();
}

void GPIO_RestoreAfterWake_Simple(void)
{
    MX_GPIO_Init();  // CubeMX vrátí původní konfigurace pinů
}

static inline void GPIO_WriteIfNotKeep(const KeepPin* keep, size_t keep_len,
                                       GPIO_TypeDef* port, uint16_t pin, GPIO_PinState st)
{
    if (!GPIO_IsKeep(keep, keep_len, port, pin)) {
        HAL_GPIO_WritePin(port, pin, st);
    }
}

static inline void GPIO_InitIfNotKeep(const KeepPin* keep, size_t keep_len,
                                      GPIO_TypeDef* port, uint16_t pin,
                                      uint32_t mode, uint32_t pull,
                                      uint32_t speed, uint32_t alternate)
{
    if (GPIO_IsKeep(keep, keep_len, port, pin)) return;
    GPIO_InitTypeDef i = {0};
    i.Pin       = pin;
    i.Mode      = mode;
    i.Pull      = pull;
    i.Speed     = speed;
    i.Alternate = alternate;
    HAL_GPIO_Init(port, &i);
}

int GPIO_IsKeep(const KeepPin* keep, size_t keep_len, GPIO_TypeDef* port, uint16_t pin)
{
    for (size_t i = 0; i < keep_len; ++i) {
        if (keep[i].port == port && keep[i].pin == pin) return 1;
    }
    return 0;
}

// ---- selective re-init (clone of MX_GPIO_Init but skipping KEEP pins) -----
void MX_GPIO_ReInit_Except(const KeepPin* keep, size_t keep_len)
{
    // Clocks (bezpečné volat opakovaně)
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // --- Výstupní defaultní stavy (nepřepisuj KEEP) ------------------------
    // Port C group
    GPIO_WriteIfNotKeep(keep, keep_len, GPIOC, BAT_CTRL1_Pin, GPIO_PIN_RESET);
    GPIO_WriteIfNotKeep(keep, keep_len, GPIOC, BAT_CTRL2_Pin, GPIO_PIN_RESET);
    GPIO_WriteIfNotKeep(keep, keep_len, GPIOC, EPD_DC_Pin,    GPIO_PIN_RESET);
    GPIO_WriteIfNotKeep(keep, keep_len, GPIOC, EPD_RST_Pin,   GPIO_PIN_RESET);
    GPIO_WriteIfNotKeep(keep, keep_len, GPIOC, RF_GPIO1_Pin,  GPIO_PIN_RESET);
    GPIO_WriteIfNotKeep(keep, keep_len, GPIOC, RF_GPIO2_Pin,  GPIO_PIN_RESET);
    GPIO_WriteIfNotKeep(keep, keep_len, GPIOC, RF_GPIO0_Pin,  GPIO_PIN_RESET);
    GPIO_WriteIfNotKeep(keep, keep_len, GPIOC, RF_RST_Pin,    GPIO_PIN_RESET);
    GPIO_WriteIfNotKeep(keep, keep_len, GPIOC, PS1_Pin,       GPIO_PIN_RESET);
    GPIO_WriteIfNotKeep(keep, keep_len, GPIOC, PS2_Pin,       GPIO_PIN_RESET);
    GPIO_WriteIfNotKeep(keep, keep_len, GPIOC, PS3_Pin,       GPIO_PIN_RESET);

    // Port A group
    GPIO_WriteIfNotKeep(keep, keep_len, GPIOA, EPD_CS_Pin,    GPIO_PIN_RESET);
    GPIO_WriteIfNotKeep(keep, keep_len, GPIOA, RF_CS_Pin,     GPIO_PIN_RESET);

    // Port B group
    GPIO_WriteIfNotKeep(keep, keep_len, GPIOB, LED_Pin,       GPIO_PIN_RESET);
    GPIO_WriteIfNotKeep(keep, keep_len, GPIOB, PULL_CNTR_Pin, GPIO_PIN_RESET);

    // PS4
    GPIO_WriteIfNotKeep(keep, keep_len, PS4_GPIO_Port, PS4_Pin, GPIO_PIN_RESET);

    // --- Konfigurace pinů (skip KEEP) --------------------------------------
    // PC13 analog
    GPIO_InitIfNotKeep(keep, keep_len, GPIOC, GPIO_PIN_13, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);

    // PF0..PF3 analog
    GPIO_InitIfNotKeep(keep, keep_len, GPIOF, GPIO_PIN_0, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOF, GPIO_PIN_1, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOF, GPIO_PIN_2, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOF, GPIO_PIN_3, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);

    // Port C outputs
    GPIO_InitIfNotKeep(keep, keep_len, GPIOC, BAT_CTRL1_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOC, BAT_CTRL2_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOC, EPD_DC_Pin,    GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOC, EPD_RST_Pin,   GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOC, RF_GPIO1_Pin,  GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOC, RF_GPIO2_Pin,  GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOC, RF_GPIO0_Pin,  GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOC, RF_RST_Pin,    GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOC, PS1_Pin,       GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOC, PS2_Pin,       GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOC, PS3_Pin,       GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);

    // USB_ON input
    GPIO_InitIfNotKeep(keep, keep_len, USB_ON_GPIO_Port, USB_ON_Pin, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);

    // STAT1, STAT2 inputs
    GPIO_InitIfNotKeep(keep, keep_len, GPIOA, STAT1_Pin, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOA, STAT2_Pin, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);

    // PA3,4,11,12,15 analog
    GPIO_InitIfNotKeep(keep, keep_len, GPIOA, GPIO_PIN_3,  GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOA, GPIO_PIN_4,  GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOA, GPIO_PIN_11, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOA, GPIO_PIN_12, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOA, GPIO_PIN_15, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);

    // EPD_CS, RF_CS outputs
    GPIO_InitIfNotKeep(keep, keep_len, GPIOA, EPD_CS_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOA, RF_CS_Pin,  GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);

    // EPD_BUSY input
    GPIO_InitIfNotKeep(keep, keep_len, EPD_BUSY_GPIO_Port, EPD_BUSY_Pin, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);

    // PB1, PB2, PB12, PB3, PB5, PB9 analog
    GPIO_InitIfNotKeep(keep, keep_len, GPIOB, GPIO_PIN_1,  GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOB, GPIO_PIN_2,  GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOB, GPIO_PIN_12, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOB, GPIO_PIN_3,  GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOB, GPIO_PIN_5,  GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOB, GPIO_PIN_9,  GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);

    // LED, PULL_CNTR outputs
    GPIO_InitIfNotKeep(keep, keep_len, GPIOB, LED_Pin,       GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);
    GPIO_InitIfNotKeep(keep, keep_len, GPIOB, PULL_CNTR_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);

    // BUTTON: VARIANTA A – jen když není v KEEP
    GPIO_InitIfNotKeep(keep, keep_len, BUTTON_GPIO_Port, BUTTON_Pin,
                       GPIO_MODE_IT_RISING_FALLING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);

    // PS4 output
    GPIO_InitIfNotKeep(keep, keep_len, PS4_GPIO_Port, PS4_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);

    // GPO input with EXTI rising
    GPIO_InitIfNotKeep(keep, keep_len, GPO_GPIO_Port, GPO_Pin, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);

    // EXTI NVIC (společné – jen povolit; pokud je BUTTON v KEEP a chceš EXTI, použij variantu B)
    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

