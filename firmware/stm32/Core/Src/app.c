/* app.c
 *
 * Author: Evzen Steif
 *
 * Description:
 *   Main application state machine for the air quality node.
 *   Handles measurements, communication (LoRa / Matter), VOC processing,
 *   power management (STOP2 / rails), GUI updates on the E-Ink display,
 *   configuration mode over UART, and battery monitoring.
 */

#include "app.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>


/* Peripherals / drivers */
#include "SDC41.h"
#include "SGP40.h"
#include "WioE5.h"
#include "SHT41.h"
#include "GUI.h"
#include "button_handler.h"
#include "RTC.h"
#include "EPD_2in13_V4.h"
#include "bq25185.h"
#include "gpio_sleep.h"
#include "conf_console.h"
#include "esp32c6.h"
#include "m24c02.h"

#include "stm32u0xx_hal_gpio.h"
#include "stm32u0xx_hal_adc.h"
#include "stm32u0xx_hal_lptim.h"


/* Algorithms */
#include "sensirion_gas_index_algorithm.h"

/* ---------------- External handles ---------------- */
extern void SystemClock_Config(void);
extern void MX_ADC1_Init(void);
extern void MX_I2C1_Init(void);
extern void MX_SPI1_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_SPI2_Init(void);
extern void MX_USART1_UART_Init(void);

extern ADC_HandleTypeDef   hadc1;
extern I2C_HandleTypeDef   hi2c1;
extern RTC_HandleTypeDef   hrtc;
extern SPI_HandleTypeDef   hspi1;
extern SPI_HandleTypeDef   hspi2;
extern UART_HandleTypeDef  huart1;
extern UART_HandleTypeDef  huart4;
extern LPTIM_HandleTypeDef hlptim2;

extern WioOtaaConfig_t   g_wio;
extern EspC6Context_t    g_esp;

/* ---------------- Power switch control ---------------- */
typedef struct { GPIO_TypeDef* port; uint16_t pin; } PowerSwitch_t;
static const PowerSwitch_t PS_SENSORS = { PS1_GPIO_Port, PS1_Pin };
static const PowerSwitch_t PS_RF      = { PS2_GPIO_Port, PS2_Pin };
static const PowerSwitch_t PS_EINK    = { PS4_GPIO_Port, PS4_Pin };

#define PS_ON(ps)  HAL_GPIO_WritePin((ps).port, (ps).pin, GPIO_PIN_SET)
#define PS_OFF(ps) HAL_GPIO_WritePin((ps).port, (ps).pin, GPIO_PIN_RESET)

/* Pins to keep active during sleep (everything else -> Analog/High-Z) */
static const KeepPin KEEP_PINS[] = {
    { USB_ON_GPIO_Port, USB_ON_Pin },
    { BUTTON_GPIO_Port, BUTTON_Pin },
    { RF_TX_GPIO_Port, RF_TX_Pin },
	{ RF_RX_GPIO_Port, RF_RX_Pin },
    { RF_RST_GPIO_Port, RF_RST_Pin },
    { RF_GPIO1_GPIO_Port, RF_GPIO1_Pin },
	{ RF_GPIO1_GPIO_Port, RF_GPIO1_Pin }, // ESP_WAKE pin
	{ RF_GPIO2_GPIO_Port, RF_GPIO2_Pin }, // ESP_READY pin
    { PS2_GPIO_Port, PS2_Pin }, /* RF rail */
    { GPIOA, GPIO_PIN_13 }, // SWDIO (disable in production)
    { GPIOA, GPIO_PIN_14 }, // SWCLK
};

/* ---------------- Logging helpers ---------------- */
#ifndef LOG_ENABLE
#define LOG_ENABLE 1
#endif

static const char* App_StateName(SystemState_t s);

#if LOG_ENABLE
#define LOGI(fmt, ...) do { printf("[I][STATE=%s] " fmt "\r\n", App_StateName(app.currentState), ##__VA_ARGS__); } while(0)
#define LOGW(fmt, ...) do { printf("[W][STATE=%s] " fmt "\r\n", App_StateName(app.currentState), ##__VA_ARGS__); } while(0)
#define LOGE(fmt, ...) do { printf("[E][STATE=%s] " fmt "\r\n", App_StateName(app.currentState), ##__VA_ARGS__); } while(0)
#define LOGN(fmt, ...) do { printf("[I] " fmt "\r\n", ##__VA_ARGS__); } while(0)
#define LOGB(title)   do { printf("\r\n================ %s ================\r\n", (title)); } while(0)
#else
#define LOGI(...) do{}while(0)
#define LOGW(...) do{}while(0)
#define LOGE(...) do{}while(0)
#define LOGN(...) do{}while(0)
#define LOGB(...) do{}while(0)
#endif

static void LOG_Hex(const uint8_t* buf, size_t len) {
#if LOG_ENABLE
    printf("[I][STATE=%s] Payload: ", App_StateName(app.currentState));
    for (size_t i = 0; i < len; ++i) printf("%02X%s", buf[i], (i + 1U < len) ? " " : "");
    printf("\r\n");
#else
    (void)buf; (void)len;
#endif
}

/* ---------------- ADC sampling parameters ---------------- */
#define VREFINT_SAMPLES 8
#define BAT_SAMPLES     4

/* LPTIM2 ARR to get 10 s at LSE/128 = 256 Hz: 10 s * 256 - 1 */
#define LPTIM2_10S_ARR  (2560U - 1U)

/* ---------------- VOC / algorithm ---------------- */
static GasIndexAlgorithmParams voc_algo;
static uint8_t voc_algo_inited = 0;


/* ---------------- App context (global) ---------------- */
AppContext_t app = {
    /* basic state and flags */
    .currentState        = STATE_INIT,
    .wakeUpFlag          = 0,
    .recalibFlag         = 0,
    .confFlag            = 0,

    /* configuration (later filled from EEPROM / AppConfig_SetDefaults) */
    .cfg                 = {0},   /* intentionally zeroed; real defaults applied via AppConfig_SetDefaults() */

    /* LoRa / ESP state */
    .Connected           = 0,
    .loraNoAckCount      = 0,
    .espFailCount        = 0,

    /* time sync / simple schedulers */
    .time_sync_ok        = 0,
    .lastTimeSyncSeconds = 0xFFFFFFFFU,
    .lastMeasureSeconds  = 0U,
    .lastBatSeconds      = 0U,

    /* E-Ink link */
    .epd_link            = EPD_LINK_CONNECTED, /* default; set to DISCONNECTED to skip GUI */

    /* VOC */
    .voc_last_time       = {0},
    .voc_last_value      = 0,
    .voc_tick            = 0,
    .voc_running         = 0,
    .voc_last_raw        = 0,
    .voc_index_ready     = 0,

    /* boot / TX / GUI bookkeeping */
    .boot_done           = 0,
	.cfg_loaded          = 0,
    .last_sent_seconds   = 0U,
    .baseline_ready      = 0U,
    .gui_drawn_once      = 0U,
};


void AppConfig_SetDefaults(AppConfig_t *cfg)
{
    if (!cfg) return;

    *cfg = (AppConfig_t){
        .measureMode           = MEAS_CO2_TRH,
        .vocMode               = VOC_CONT_USB,
        .commsMode             = COMMS_MATTER,
        .interval_measure_sec  = 30U,
        .interval_sleep_sec    = 30U,
        .interval_time_req_sec = 3600U,
        .interval_bat_sec      = 1U * 60U,

        .tx_min_interval_sec   = 30U,
        .tx_max_interval_sec   = 3600U,
        .th_temp_01C           = 20U,
        .th_rh_01pct           = 100U,
        .th_co2_ppm            = 10U,
    };
}

void App_Config_SaveToEeprom(void)
{
    /* Here you can control sensor power rails / pull-ups
       if the EEPROM shares power with PS_SENSORS or similar. */

    GPIO_PinState prev_pull = HAL_GPIO_ReadPin(PULL_CNTR_GPIO_Port, PULL_CNTR_Pin);
    HAL_GPIO_WritePin(PULL_CNTR_GPIO_Port, PULL_CNTR_Pin, GPIO_PIN_RESET); // enable pull-ups
    PS_ON(PS_SENSORS);
    HAL_Delay(2);

    HAL_StatusTypeDef st = EE_Config_Save(&hi2c1, &app.cfg, &g_wio, &g_esp.pairing);

    HAL_GPIO_WritePin(PULL_CNTR_GPIO_Port, PULL_CNTR_Pin, prev_pull);

    if (st == HAL_OK) {
        LOGI("EEPROM: config + WIO OTAA + ESP pairing saved");
    } else {
        LOGE("EEPROM: save failed");
    }
}

/* ======================= State prototypes ======================= */
static void State_Measure(void);
static void State_BatMeasure(void);
static void State_SendData(void);
static void State_Recover(void);
static void State_GUI(void);
static void State_Sleep(void);
static void State_Recalib(void);
static void State_Config(void);
static void State_TimeReq(void);
static void EPD_PowerOn(void);
static void EPD_PowerOff(void);
static void EPD_BootDetect(void);
static void ESP_ResetPulse(void);
static void App_SendBatteryToEsp(void);



/* ======================= VOC helpers ======================= */
static void VOC_Mode_Ensure(void);
static void VOC_LPTIM_Start(void);
static void VOC_LPTIM_Stop(void);
static inline uint8_t VOC_Active(void) {
    return (app.cfg.vocMode == VOC_CONT_USB) && (app.usb_on != 0);
}
/* ----------------------------------------------------------------------- */

/* ---------------- Helpers ---------------- */
static const char* App_StateName(SystemState_t s) {
    switch (s) {
        case STATE_INIT:           return "INIT";
        case STATE_MEASURE:        return "MEASURE";
        case STATE_BAT_MEASURE:    return "BAT_MEASURE";
        case STATE_SEND_DATA:      return "SEND_DATA";
        case STATE_RECOVER:   	   return "RECOVER";
        case STATE_TIME_REQ:       return "TIME_REQ";
        case STATE_GUI:            return "GUI";
        case STATE_SLEEP:          return "SLEEP";
        case STATE_RECALIBRATION:  return "RECALIBRATION";
        case STATE_CONFIG:		   return "CONFIG";
        default:                   return "UNKNOWN";
    }
}

/* Seconds-of-day helpers (wrap-safe) */
static inline uint32_t sod_now(void) { return RTC_GetSecondsOfDay(); }
static inline uint32_t sod_diff(uint32_t now, uint32_t then) {
    return (then == 0U) ? 0xFFFFFFFFU : (now >= then ? (now - then) : (now + 86400U - then));
}

/* ======================= TX policy ======================= */
/*
 * Decide if we should transmit the current payload now.
 * Common policy for both LoRa and Matter:
 *  - Never send in COMMS_OFFLINE mode.
 *  - Require a valid baseline.
 *  - Always allow the very first uplink.
 *  - Respect minimum TX interval.
 *  - Force heartbeat after tx_max_interval_sec.
 *  - Otherwise, send only if T/RH/CO2 changed above configured thresholds.
 */
static int ShouldTxNow(void)
{
    /* Offline mode never transmits */
    if (app.cfg.commsMode == COMMS_OFFLINE)
        return 0;

    /* Common policy for both LoRa and Matter (ESP) */
    if (!app.baseline_ready) return 0;

    /* --- FIRST UPLINK: always allowed --- */
    if (app.last_sent_seconds == 0U) return 1;

    const uint32_t now = sod_now();
    const uint32_t dt  = sod_diff(now, app.last_sent_seconds);

    /* Minimum spacing after first TX */
    if (app.last_sent_seconds != 0U && dt < app.cfg.tx_min_interval_sec) return 0;

    /* Heartbeat */
    if (app.cfg.tx_max_interval_sec != 0U &&
        app.last_sent_seconds != 0U &&
        dt >= app.cfg.tx_max_interval_sec) return 1;

    /* Change detection relative to last successfully sent payload */
    const uint16_t prev_t   = (uint16_t)((app.last_sent_payload[0] << 8) | app.last_sent_payload[1]);
    const uint16_t prev_rh  = (uint16_t)((app.last_sent_payload[2] << 8) | app.last_sent_payload[3]);
    const uint16_t prev_co2 = (uint16_t)((app.last_sent_payload[4] << 8) | app.last_sent_payload[5]);

    const uint16_t dT   = (app.temp_data > prev_t)  ? (app.temp_data - prev_t)  : (prev_t  - app.temp_data);
    const uint16_t dRH  = (app.hum_data  > prev_rh) ? (app.hum_data  - prev_rh) : (prev_rh - app.hum_data);
    const uint16_t dCO2 = (app.co2_data  > prev_co2)? (app.co2_data  - prev_co2): (prev_co2- app.co2_data);

    if (dT  >= app.cfg.th_temp_01C)  return 1;
    if (dRH >= app.cfg.th_rh_01pct)  return 1;
    if (dCO2>= app.cfg.th_co2_ppm)   return 1;

    return 0;
}


/* === GUI meta snapshot (values that the GUI shows besides T/RH/CO2) === */
typedef struct {
    uint8_t  Connected;
    uint8_t  usb_on;
    uint32_t sleep_interval_sec;

    /* battery presence (OFF/ON) + voltage in mV */
    uint8_t  socl2_present;
    uint16_t vbat_lsocl2_mV;

    uint8_t  liion_present;
    uint16_t vbat_liion_mV;

    /* charger status (copy as-is) */
    BQ25185_Status_t charger_status;
} GuiMeta_t;

static GuiMeta_t s_gui_meta_prev;
static uint8_t   s_gui_meta_prev_valid = 0;

/* voltage delta to trigger redraw (hysteresis) */
#ifndef GUI_VBAT_DELTA_MV
#define GUI_VBAT_DELTA_MV 50U
#endif

static inline uint8_t diff_bool(uint8_t a, uint8_t b) { return (a != b); }
static inline uint8_t diff_u32(uint32_t a, uint32_t b){ return (a != b); }
static inline uint8_t diff_mv(uint16_t now, uint16_t prev, uint16_t thr)
{
    if (now > prev) return (now - prev) >= thr;
    return (prev - now) >= thr;
}

static GuiMeta_t BuildGuiMetaSnapshot(void)
{
    GuiMeta_t m;
    m.Connected      = app.Connected ? 1 : 0;
    m.usb_on             = app.usb_on ? 1 : 0;
    m.sleep_interval_sec = app.cfg.interval_sleep_sec;

    m.vbat_lsocl2_mV     = app.vbat_lsocl2_mV;
    m.socl2_present      = (app.vbat_lsocl2_mV >= 1000U) ? 1 : 0;

    m.vbat_liion_mV      = app.vbat_liion_mV;
    m.liion_present      = (app.vbat_liion_mV  >= 1000U) ? 1 : 0;

    m.charger_status     = app.charger_status;
    return m;
}

/* Optional: emoji bucket (coarse category) so GUI updates when crossing thresholds even for small deltas */
static uint8_t emoji_bucket(uint16_t co2_ppm, float tempC) {
    /* CO2 bands: <800, <1200, <2500, >=2500 ; Temp bands: <15, 15..35, >35 */
    uint8_t cb = (co2_ppm < 800) ? 0 : (co2_ppm < 1200) ? 1 : (co2_ppm < 2500) ? 2 : 3;
    uint8_t tb = (tempC   < 15.0f) ? 0 : (tempC   > 35.0f) ? 2 : 1;
    return (uint8_t)((tb << 2) | cb); /* 0..11 */
}

/*
 * Decide whether to perform a full-screen GUI refresh.
 * First refresh is forced, then only when:
 *  - GUI meta (connection, USB, sleep interval, charger state, batteries) changes
 *  - payload changes above thresholds (same as TX)
 *  - emoji bucket changes (coarse air quality categorization)
 */
static int ShouldGUIRefreshNow(void)
{
    if (app.epd_link != EPD_LINK_CONNECTED) return 0;   /* no display */

    /* 1) Always do the very first full-screen draw so the panel isn't blank */
    if (!app.gui_drawn_once) return 1;

    /* 2) Meta states: LoRa, USB, sleep interval, charger status, battery presence/voltage */
    GuiMeta_t cur = BuildGuiMetaSnapshot();
    if (!s_gui_meta_prev_valid) return 1;

    if (diff_bool(cur.Connected, s_gui_meta_prev.Connected)) return 1;
    if (diff_bool(cur.usb_on,        s_gui_meta_prev.usb_on))        return 1;
    if (diff_u32 (cur.sleep_interval_sec, s_gui_meta_prev.sleep_interval_sec)) return 1;

    if (memcmp(&cur.charger_status, &s_gui_meta_prev.charger_status, sizeof(BQ25185_Status_t)) != 0)
        return 1;

    if (diff_bool(cur.socl2_present, s_gui_meta_prev.socl2_present)) return 1;
    if (diff_bool(cur.liion_present, s_gui_meta_prev.liion_present)) return 1;

    if (cur.socl2_present && diff_mv(cur.vbat_lsocl2_mV, s_gui_meta_prev.vbat_lsocl2_mV, GUI_VBAT_DELTA_MV)) return 1;
    if (cur.liion_present && diff_mv(cur.vbat_liion_mV,  s_gui_meta_prev.vbat_liion_mV,  GUI_VBAT_DELTA_MV)) return 1;

    /* 3) Main payload change thresholds (same as TX) */
    const uint16_t prev_t   = (uint16_t)((app.last_gui_payload[0] << 8) | app.last_gui_payload[1]);
    const uint16_t prev_rh  = (uint16_t)((app.last_gui_payload[2] << 8) | app.last_gui_payload[3]);
    const uint16_t prev_co2 = (uint16_t)((app.last_gui_payload[4] << 8) | app.last_gui_payload[5]);

    const uint16_t dT   = (app.temp_data > prev_t)  ? (app.temp_data - prev_t)  : (prev_t  - app.temp_data);
    const uint16_t dRH  = (app.hum_data  > prev_rh) ? (app.hum_data  - prev_rh) : (prev_rh - app.hum_data);
    const uint16_t dCO2 = (app.co2_data  > prev_co2)? (app.co2_data  - prev_co2): (prev_co2- app.co2_data);

    if (dT  >= app.cfg.th_temp_01C)  return 1;
    if (dRH >= app.cfg.th_rh_01pct)  return 1;
    if (dCO2>= app.cfg.th_co2_ppm)   return 1;

    /* 4) Optional: redraw if we crossed emoji bucket thresholds (CO2 800/1200/2500; T 15/35 °C) */
    {
        float curT  = app.temp_data / 100.0f;
        float prevT = prev_t / 100.0f;
        uint8_t buck_cur  = emoji_bucket(app.co2_data, curT);
        uint8_t buck_prev = emoji_bucket(prev_co2,     prevT);
        if (buck_cur != buck_prev) return 1;
    }

    /* 5) Otherwise, no full redraw needed */
    return 0;
}

/* ======================= IRQ handlers ======================= */
void App_WakeUpIRQ(void)
{
	app.wakeUpFlag = 1;
}
void App_UserButtonIRQ_15s(void){ app.recalibFlag = 1; }
void App_UserButtonIRQ_5s(void){  app.confFlag++; }
void App_VOC_TickIRQ(void)  { app.voc_tick = 1; }

void EPD_OnFatalError(void)
{
    static uint8_t once = 0;
    if (once) return;
    once = 1;

    LOGW("EPD fatal/timeout -> disabling GUI");
    app.epd_link = EPD_LINK_DISCONNECTED;
}


/* ======================= Application Init ======================= */
/**
 * Initialize or resume the application.
 *
 * - On wake-up from STOP2: re-initializes clocks and peripherals,
 *   recomputes USB state and optionally refreshes GUI.
 * - On cold boot (first run): performs full hardware and radio initialization,
 *   loads configuration from EEPROM, probes E-Ink presence, and brings up
 *   communication (LoRa or Matter) according to configuration.
 */
/* ======================= App_Init ======================= */
void App_Init(void)
{
    app.confFlag = 0;

    if (app.wakeUpFlag) {
        /* Re-init path after STOP2 */
        app.wakeUpFlag = 0;
        RTC_UpdateTimeData();

        GPIO_Sleep_SetKeepPins(KEEP_PINS, sizeof(KEEP_PINS)/sizeof(KEEP_PINS[0]));
        MX_GPIO_ReInit_Except(KEEP_PINS, sizeof(KEEP_PINS)/sizeof(KEEP_PINS[0]));

        HAL_UART_DeInit(&huart4); MX_USART4_UART_Init();
        HAL_UART_DeInit(&huart1); MX_USART1_UART_Init();
        HAL_SPI_DeInit(&hspi1);   MX_SPI1_Init();
        HAL_SPI_DeInit(&hspi2);   MX_SPI2_Init();
        HAL_I2C_DeInit(&hi2c1);   MX_I2C1_Init();
        HAL_ADC_DeInit(&hadc1);   MX_ADC1_Init();

        app.usb_on = (HAL_GPIO_ReadPin(USB_ON_GPIO_Port, USB_ON_Pin) == GPIO_PIN_SET);

        LOGB("WAKE-UP");
        LOGI("Wake-up path complete (clocks/peripherals re-initialized)");


        if ((app.epd_link == EPD_LINK_CONNECTED) && app.usb_on)
        {
				EPD_PowerOn();
				GUI_UpdateState(app.currentState);
				EPD_PowerOff();
		}
        return;
    }

    if (app.boot_done) {
        /* Skip heavy cold boot if not slept. */
        LOGI("Cold boot already done -> light refresh only");
        GPIO_Sleep_SetKeepPins(KEEP_PINS, sizeof(KEEP_PINS)/sizeof(KEEP_PINS[0]));
        return;
    }

    /* ===== Cold boot path (run once) ===== */
    GPIO_Sleep_SetKeepPins(KEEP_PINS, sizeof(KEEP_PINS)/sizeof(KEEP_PINS[0]));
    ButtonHandler_Init();

    PS_ON(PS_SENSORS);
    HAL_Delay(50);

    /* READY pin (RF_GPIO2) as input with pull-down (or NOPULL depending on HW) */
    HAL_GPIO_DeInit(GPIOC, RF_GPIO2_Pin);
    GPIO_InitTypeDef gi = {0};
    gi.Pin  = RF_GPIO2_Pin;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_PULLDOWN;   // or GPIO_NOPULL depending on HW
    HAL_GPIO_Init(GPIOC, &gi);

    app.usb_on = (HAL_GPIO_ReadPin(USB_ON_GPIO_Port, USB_ON_Pin) == GPIO_PIN_SET);

    /* ---- EEPROM: load config + LoRa + ESP pairing ---- */
    if (!app.cfg_loaded) {
        GPIO_PinState prev_pull = HAL_GPIO_ReadPin(PULL_CNTR_GPIO_Port, PULL_CNTR_Pin);
        HAL_GPIO_WritePin(PULL_CNTR_GPIO_Port, PULL_CNTR_Pin, GPIO_PIN_RESET); // I2C pull-ups ON
        PS_ON(PS_SENSORS);
        PS_ON(PS_RF);
        HAL_Delay(20);

        if (EE_Config_Load(&hi2c1, &app.cfg, &g_wio, &g_esp.pairing) != HAL_OK) {
            LOGW("EEPROM: invalid/missing -> defaults");
        } else {
            LOGI("EEPROM: config loaded");
        }

        HAL_GPIO_WritePin(PULL_CNTR_GPIO_Port, PULL_CNTR_Pin, prev_pull);

        app.cfg_loaded = 1;
    }

    LOGI("Config: measure=%d voc=%d comms=%d",
         (int)app.cfg.measureMode, (int)app.cfg.vocMode, (int)app.cfg.commsMode);

    EPD_BootDetect();

    if (app.epd_link == EPD_LINK_CONNECTED) {
        EPD_PowerOn();
        LOGI("GUI: init");
        GUI_Init();
        if(app.usb_on) GUI_UpdateState(app.currentState);
        EPD_PowerOff();
    } else {
        LOGW("E-Ink disconnected -> skipping GUI init");
    }

    /* === Radio init according to comms mode === */
    if (app.cfg.commsMode == COMMS_MATTER) {
        /* ESP/Matter: do not start WioE5; only prepare ESP */
        PS_ON(PS_RF);
        ESP_Init();
        LOGI("ESP init: PS_RF ON, READY=input PD, WAKE=LOW");
    } else if (app.cfg.commsMode == COMMS_LORA) {
        /* LoRa: start WioE5 + OTAA join */
        PS_ON(PS_RF);
        HAL_GPIO_WritePin(RF_RST_GPIO_Port, RF_RST_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RF_GPIO1_GPIO_Port, RF_GPIO1_Pin, GPIO_PIN_SET); /* WAKE high does not disturb WioE5 */
        HAL_Delay(3000);

        LOGI("Initializing WIO-E5 (OTAA)...");
        if (!Wio_EnsureReady(1500)) {
            LOGE("Modem unresponsive!");
        }
        if (!Wio_Init_OTAA()) {
            LOGE("Wio_Init_OTAA failed.");
            app.Connected = 0;
        } else {
            if (Wio_SendCommand("AT+JOIN", "Network joined", 30000)) {
                app.Connected = 1;
                app.loraNoAckCount = 0;
                LOGI("OTAA JOIN OK.");
            } else {
                app.Connected = 0;
                LOGW("OTAA JOIN failed.");
            }
        }
    } else {
        /* Offline: radio off */
        PS_OFF(PS_RF);
    }

    LOGB("BOOT");
    LOGI("Modes: measure=%d, voc=%d, comms=%d",
         (int)app.cfg.measureMode, (int)app.cfg.vocMode, (int)app.cfg.commsMode);

    app.boot_done = 1; /* mark cold boot completed */
}

/* ======================= Application Run ======================= */
/**
 * Main application state machine step.
 *
 * - Reacts to user button flags (recalibration, configuration).
 * - Executes one state handler per call, then transitions to the next state.
 * - Maintains continuous VOC mode, including 10 s raw sampling via LPTIM2.
 * - Updates GUI status line and VOC index when E-Ink and USB are active.
 */
void App_Run(void)
{
    if (app.recalibFlag) app.currentState = STATE_RECALIBRATION;
    if (app.confFlag == 1)    app.currentState = STATE_CONFIG;

    /* Log state header only when the state actually changes */
    static SystemState_t s_prev_logged_state = (SystemState_t)(-1);
    if (app.currentState != s_prev_logged_state) {
        LOGB(App_StateName(app.currentState));
        s_prev_logged_state = app.currentState;
    }

    SystemState_t next_state = app.currentState;

    switch (app.currentState) {
        case STATE_INIT:         App_Init();            next_state = STATE_MEASURE;       break;
        case STATE_MEASURE:      State_Measure();       next_state = STATE_BAT_MEASURE;   break;
        case STATE_BAT_MEASURE:  State_BatMeasure();    next_state = STATE_SEND_DATA;     break;
        case STATE_SEND_DATA:    State_SendData();      next_state = STATE_RECOVER;  	  break;
        case STATE_RECOVER:      State_Recover();       next_state = STATE_TIME_REQ;      break;
        case STATE_TIME_REQ:     State_TimeReq();       next_state = STATE_GUI;           break;
        case STATE_GUI:          State_GUI();           next_state = STATE_SLEEP;         break;
        case STATE_SLEEP:
            State_Sleep();
            if (app.usb_on && app.cfg.vocMode == VOC_CONT_USB)
                next_state = STATE_MEASURE;
            else
                next_state = STATE_INIT;
            break;
        case STATE_RECALIBRATION:
            State_Recalib();
            next_state = STATE_INIT;
            break;
        case STATE_CONFIG:
            /* Stay in CONFIG until the console asks to exit; no spammy header logging */
            State_Config();
            return; /* early return: nothing else this tick */
        default:
            LOGW("Unknown state -> default to INIT");
            next_state = STATE_INIT;
            break;
    }

    /* Apply transition */
    if (app.currentState != next_state) {
        LOGN("Transition: %s -> %s", App_StateName(app.currentState), App_StateName(next_state));
        app.currentState = next_state;
    }

    /* VOC mode transitions and 10 s ticks between states (not in CONFIG due to early return) */
    VOC_Mode_Ensure();

    if (app.voc_running && app.voc_tick) {
        app.voc_tick = 0;

        uint16_t raw = 0;
        if (SGP40_ReadRawComp_HeaterOn(&hi2c1, 200, &raw) == HAL_OK) {
            app.voc_last_raw = raw;
            LOGI("VOC raw sample = %u", (unsigned)raw);
        } else {
            LOGW("VOC raw read failed");
        }
    }

    /* Short GUI status (state line + VOC partials) */
    if ((app.epd_link == EPD_LINK_CONNECTED) && app.usb_on) {
        EPD_PowerOn();
        GUI_UpdateState(app.currentState);

        if (!VOC_Active()) {
            EPD_PowerOff();
        }

        if (app.voc_running && app.voc_index_ready && app.gui_drawn_once) {
            app.voc_index_ready = 0;
            GUI_UpdateVOCIndex((uint16_t)app.voc_last_value);
        }
    } else {
        LOGW("Skipping GUI update");
        HAL_Delay(2000);
    }
}


/* ======================= State handlers ======================= */

static inline int should_do_measure(void) {
    const uint32_t now = sod_now();
    if (app.lastMeasureSeconds == 0U) return 1;
    return sod_diff(now, app.lastMeasureSeconds) >= app.cfg.interval_measure_sec;
}

/**
 * STATE_MEASURE
 *
 * - Checks USB and charger status.
 * - Performs CO2/T/RH or T/RH measurement depending on configuration.
 * - Builds the 6-byte payload (T, RH, CO2) in fixed-point format.
 * - Initializes TX baseline on first valid measurement.
 * - Manages sensor rail and I2C pull-ups, respecting VOC continuous mode.
 */
static void State_Measure(void)
{
    /* USB state always checked */
    app.usb_on = (HAL_GPIO_ReadPin(USB_ON_GPIO_Port, USB_ON_Pin) == GPIO_PIN_SET);
    LOGI("USB %s", app.usb_on ? "connected" : "disconnected");

    /* Charger status (most meaningful when USB present) */
    BQ25185_GPIO_Config(app.usb_on ? 1 : 0);
    BQ25185_GetStatus();
    BQ25185_PrintStatus();

    if (!should_do_measure()) { LOGI("MEASURE skipped (interval)"); return; }

    PS_ON(PS_SENSORS); HAL_Delay(5);
    HAL_GPIO_WritePin(PULL_CNTR_GPIO_Port, PULL_CNTR_Pin, GPIO_PIN_RESET); HAL_Delay(2);

    LOGI("Measuring values (%s)...", app.cfg.measureMode == MEAS_CO2_TRH ? "CO2+T+RH" : "T+RH only");

    if (app.cfg.measureMode == MEAS_CO2_TRH) {
        SCD41_WakeUp(&hi2c1); HAL_Delay(20);
        SCD41_SingleMeasurement_Averaged(&hi2c1);
        app.co2_data = (uint16_t)SCD_data.co2;
    } else {
        app.co2_data = 0;
    }

    SHTMeasure(&hi2c1);
    app.temp_data = (uint16_t)(SHT_data.temperature * 100.0f);
    app.hum_data  = (uint16_t)(SHT_data.rh * 100.0f);

    LOGI("T=%.2f C, RH=%.2f %%, CO2=%u ppm", SHT_data.temperature, SHT_data.rh, (unsigned)app.co2_data);

    /* payload (6B: T(.01C), RH(.01%), CO2 ppm) */
    app.payload[0] = (uint8_t)(app.temp_data >> 8); app.payload[1] = (uint8_t)(app.temp_data & 0xFF);
    app.payload[2] = (uint8_t)(app.hum_data  >> 8); app.payload[3] = (uint8_t)(app.hum_data  & 0xFF);
    app.payload[4] = (uint8_t)(app.co2_data  >> 8); app.payload[5] = (uint8_t)(app.co2_data  & 0xFF);
    LOG_Hex(app.payload, sizeof(app.payload));

    /* Prime TX baseline if needed */
    if (!app.baseline_ready) {
        memcpy(app.last_sent_payload, app.payload, sizeof(app.last_sent_payload));
        app.baseline_ready = 1;
        LOGI("TX baseline primed (no uplink yet)");
    }

    if (app.cfg.measureMode == MEAS_CO2_TRH) {
        SCD41_PowerDown(&hi2c1);
    }

    if (!VOC_Active()) {
        HAL_GPIO_WritePin(PULL_CNTR_GPIO_Port, PULL_CNTR_Pin, GPIO_PIN_SET); // disable pull-ups only if not in VOC
        PS_OFF(PS_SENSORS);                                                  // power off sensors rail only if not in VOC
    } else {
        LOGI("VOC active -> keep sensor rail + I2C pull-ups ON");
    }

    app.lastMeasureSeconds = sod_now();
}

static inline int should_do_bat(void) {
    const uint32_t now = sod_now();
    if (app.lastBatSeconds == 0U) return 1;
    return sod_diff(now, app.lastBatSeconds) >= app.cfg.interval_bat_sec;
}

/**
 * STATE_BAT_MEASURE
 *
 * - Calibrates the ADC once on first use.
 * - Measures VDDA using internal VREFINT.
 * - Measures Li-Ion and SOCl2 battery channels through voltage dividers.
 * - Converts raw readings to millivolts and stores them in the app context.
 * - Optionally forwards the results to ESP (Matter) for power source reporting.
 */
static void State_BatMeasure(void)
{
    if (!should_do_bat()) { LOGI("BAT_MEASURE skipped (interval)"); return; }

    static uint8_t adc_calibrated = 0;
    if (!adc_calibrated) {
        HAL_ADC_DeInit(&hadc1);
        if (HAL_ADC_Init(&hadc1) != HAL_OK) { LOGE("ADC init failed before calibration"); Error_Handler(); }
        if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) { LOGE("ADC calibration failed"); Error_Handler(); }
        adc_calibrated = 1; LOGI("ADC calibrated");
    }

    HAL_GPIO_WritePin(BAT_CTRL1_GPIO_Port, BAT_CTRL1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BAT_CTRL2_GPIO_Port, BAT_CTRL2_Pin, GPIO_PIN_SET);
    HAL_Delay(200);

    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc1.Instance), LL_ADC_PATH_INTERNAL_VREFINT);
    HAL_Delay(1);
    ADC_ChannelConfTypeDef s = {0};
    s.Channel = ADC_CHANNEL_VREFINT; s.Rank = ADC_REGULAR_RANK_1; s.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    HAL_ADC_ConfigChannel(&hadc1, &s);
    HAL_ADC_Start(&hadc1); HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); (void)HAL_ADC_GetValue(&hadc1); HAL_ADC_Stop(&hadc1);
    uint32_t acc = 0; for (int i=0;i<VREFINT_SAMPLES;i++){ HAL_ADC_Start(&hadc1); HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); acc += HAL_ADC_GetValue(&hadc1); HAL_ADC_Stop(&hadc1);}
    uint32_t raw_vref = (acc + (VREFINT_SAMPLES/2U)) / VREFINT_SAMPLES;
    float vdda_mV = (float)__LL_ADC_CALC_VREFANALOG_VOLTAGE(raw_vref, LL_ADC_RESOLUTION_12B);
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc1.Instance), LL_ADC_PATH_INTERNAL_NONE);

    /* Li-Ion on CH0 */
    s.Channel = ADC_CHANNEL_0; s.SamplingTime = ADC_SAMPLINGTIME_COMMON_2; HAL_ADC_ConfigChannel(&hadc1, &s);
    HAL_ADC_Start(&hadc1); HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); (void)HAL_ADC_GetValue(&hadc1); HAL_ADC_Stop(&hadc1);
    acc = 0; for (int i=0;i<BAT_SAMPLES;i++){ HAL_ADC_Start(&hadc1); HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); acc += HAL_ADC_GetValue(&hadc1); HAL_ADC_Stop(&hadc1);}
    uint32_t raw_liion = (acc + (BAT_SAMPLES/2U)) / BAT_SAMPLES;

    /* SOCl2 on CH1 */
    s.Channel = ADC_CHANNEL_1; HAL_ADC_ConfigChannel(&hadc1, &s);
    HAL_ADC_Start(&hadc1); HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); (void)HAL_ADC_GetValue(&hadc1); HAL_ADC_Stop(&hadc1);
    acc = 0; for (int i=0;i<BAT_SAMPLES;i++){ HAL_ADC_Start(&hadc1); HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); acc += HAL_ADC_GetValue(&hadc1); HAL_ADC_Stop(&hadc1);}
    uint32_t raw_lsocl2 = (acc + (BAT_SAMPLES/2U)) / BAT_SAMPLES;

    HAL_GPIO_WritePin(BAT_CTRL1_GPIO_Port, BAT_CTRL1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BAT_CTRL2_GPIO_Port, BAT_CTRL2_Pin, GPIO_PIN_RESET);

    const float gain = (100.0f + 330.0f) / 330.0f; // 1.303
    app.vbat_lsocl2_mV = (uint16_t)((raw_lsocl2 * vdda_mV / 4095.0f) * gain + 0.5f);
    app.vbat_liion_mV  = (uint16_t)((raw_liion  * vdda_mV / 4095.0f) * gain + 0.5f);

    LOGI("raw_vref=%lu raw_liion=%lu raw_lsocl2=%lu",
         (unsigned long)raw_vref, (unsigned long)raw_liion, (unsigned long)raw_lsocl2);
    LOGI("VDDA: %.1f mV | VBAT SOCl2: %u mV | VBAT Li-Ion: %u mV",
         vdda_mV, app.vbat_lsocl2_mV, app.vbat_liion_mV);

    app.lastBatSeconds = sod_now();

    if (app.cfg.commsMode == COMMS_MATTER)
    {
    App_SendBatteryToEsp();
    }

}

/**
 * STATE_SEND_DATA
 *
 * Send the current 6-byte payload depending on communication mode:
 *  - COMMS_OFFLINE: skip transmission.
 *  - COMMS_MATTER: send HEX frame to ESP32-C6 over UART (Matter bridge).
 *  - COMMS_LORA: send confirmed uplink via WioE5 LoRaWAN modem.
 *
 * TX is further gated by ShouldTxNow() (intervals and threshold logic).
 */
/* ======================= State_SendData ======================= */
static void State_SendData(void)
{
    if (!ShouldTxNow()) { LOGI("TX skipped"); return; }

    switch (app.cfg.commsMode) {
    case COMMS_OFFLINE:
        LOGI("COMMS offline -> TX skipped");
        app.Connected = 0;
        return;

    case COMMS_MATTER: {
        LOGI("Sending ESP/Matter text frame (HEX 6B)...");
        LOG_Hex(app.payload, sizeof(app.payload));
        uint8_t ok = EspC6_DataHex(app.payload);
        if (ok) {
            memcpy(app.last_sent_payload, app.payload, sizeof(app.last_sent_payload));
            app.last_sent_seconds = sod_now();
            app.espFailCount = 0;
            LOGI("ESP TX OK");
        } else {
            app.espFailCount++;
            LOGW("ESP TX failed (cnt=%u)", app.espFailCount);
        }
        return;
    }

    case COMMS_LORA: {
        (void)Wio_Wake(1000);
        if (!app.Connected) { LOGW("LoRa not connected, skipping TX"); return; }
        LOGI("Sending LoRa data (CONFIRMED)...");
        LOG_Hex(app.payload, sizeof(app.payload));
        WioSendResult_t r = Wio_SendData(app.payload, sizeof(app.payload));
        switch (r) {
            case WIO_SEND_ACK_RECEIVED:
                LOGI("ACK received");
                app.loraNoAckCount = 0;
                app.Connected = 1;
                memcpy(app.last_sent_payload, app.payload, sizeof(app.last_sent_payload));
                app.last_sent_seconds = sod_now();
                break;
            case WIO_SEND_NO_ACK:
                LOGW("No ACK");
                app.loraNoAckCount++;
                break;
            default:
                LOGE("Send error");
                app.loraNoAckCount++;
                break;
        }
        return;
    }
    }
}


/**
 * STATE_RECOVER
 *
 * Communication recovery common entry point:
 *  - For Matter (ESP): perform hardware reset after repeated TX failures,
 *    and periodically check STATUS to update connectivity flag.
 *  - For LoRa: re-join network if disconnected or after too many missed ACKs.
 *  - For Offline: no recovery is needed.
 */
/* ======================= Recover ======================= */
/* ======================= State_Recover (generic) ======================= */
static void State_Recover(void)
{
    /* ESP (Matter) – if repeated failures, try HW reset */
    if (app.cfg.commsMode == COMMS_MATTER) {
        if (app.espFailCount >= 3)
        {
        	app.Connected = 0;
            LOGW("ESP recover: %u consecutive TX fails -> HW reset", app.espFailCount);
            ESP_ResetPulse();
            app.espFailCount = 0;
        }
        else
        {
            LOGI("ESP recover: ok (fails=%u < 3)", app.espFailCount);

            uint8_t ok = EspC6_SendCommand("STATUS?", "STATUS ONLINE", 5000);
            app.Connected = ok ? 1 : 0;
        }
    }

    /* LoRa – legacy recovery (rejoin), formerly State_LoraRecover */
    if (app.cfg.commsMode == COMMS_LORA) {
        LOGB("LoRa Recovery");
        if (app.Connected && app.loraNoAckCount < 3) {
            LOGI("LoRa OK (noAck=%u)", (unsigned)app.loraNoAckCount);
            return;
        }
        if (!app.Connected)
            LOGW("Not connected → re-join required");
        else
            LOGW("Too many missed ACKs (%u ≥ %u)", (unsigned)app.loraNoAckCount, 3U);

        if (!Wio_EnsureReady(1500)) LOGE("Modem unresponsive!");
        Wio_SendCommand("AT+RESET", "OK", 5000); HAL_Delay(2000);
        if (Wio_SendCommand("AT+JOIN", "Network joined", 30000)) {
            app.Connected = 1; app.loraNoAckCount = 0; LOGI("Re-join OK");
        } else {
            app.Connected = 0; LOGE("Re-join FAILED");
        }
    }
    /* Offline – nothing to do */
}


/* ======================= State_TimeReq ======================= */
/**
 * STATE_TIME_REQ
 *
 * LoRa-only time synchronization:
 *  - Sends a time request on a dedicated port after:
 *      * initial boot (no previous sync),
 *      * configured interval has elapsed,
 *      * previous request failed.
 *  - Updates RTC and internal flags on success.
 */
static void State_TimeReq(void)
{
    if (app.cfg.commsMode != COMMS_LORA) {
        LOGI("TIME_REQ skipped (not LoRa)");
        return;
    }

    const uint32_t now  = sod_now();
    const uint32_t diff = (app.lastTimeSyncSeconds == 0xFFFFFFFFU) ? 0xFFFFFFFFU
                                                                   : sod_diff(now, app.lastTimeSyncSeconds);
    static uint8_t lastTimeReqOk = 1U;

    if (app.lastTimeSyncSeconds == 0xFFFFFFFFU || diff >= app.cfg.interval_time_req_sec || lastTimeReqOk == 0U) {
        if (app.Connected) {
            (void)Wio_Wake(800);
            LOGI("Sending TIME request (port 8)...");
            Wio_SendCommand("AT+PORT=8", "+PORT", 500);
            lastTimeReqOk = WioTimeReq();
            if (lastTimeReqOk) {
                app.lastTimeSyncSeconds = sod_now();
                app.time_sync_ok = 1;
                LOGI("TIME sync OK");
            } else {
                LOGW("TIME sync failed");
                app.time_sync_ok = 0;
            }
        } else {
            LOGW("LoRa not connected. Skipping TIME request");
        }
    } else {
        LOGI("Skipping TIME request, next in ~%lus",
             (unsigned long)(app.cfg.interval_time_req_sec - diff));
    }
    RTC_Print_DateTime();
}


/**
 * STATE_GUI
 *
 * Full-screen GUI rendering:
 *  - Skips if E-Ink is not connected.
 *  - Uses ShouldGUIRefreshNow() to avoid unnecessary full refreshes.
 *  - On refresh, powers E-Ink rail, draws the screen, then powers off.
 *  - Stores last drawn payload and GUI meta snapshot for change detection.
 */
static void State_GUI(void)
{
    if (app.epd_link != EPD_LINK_CONNECTED) {
        LOGW("E-Ink disconnected -> skipping GUI screen");
        return;
    }

    if (!ShouldGUIRefreshNow()) {
        LOGI("GUI skipped (no significant change)");
        return;
    }

    EPD_PowerOn();
    LOGI("Updating E-Ink display (change-based / first-run)...");
    GUI_Display();
    EPD_PowerOff();

    /* Remember what we drew and mark that we've done at least one full draw */
    memcpy(app.last_gui_payload, app.payload, sizeof(app.last_gui_payload));
    app.gui_drawn_once = 1;

    s_gui_meta_prev = BuildGuiMetaSnapshot();
    s_gui_meta_prev_valid = 1;
}

/**
 * STATE_SLEEP
 *
 * Enter STOP2 low-power mode if safe:
 *  - Skips STOP2 when USB is connected and continuous VOC mode is enabled.
 *  - Skips STOP2 while user button is pressed.
 *  - Powers down sensors, GUI, and (for LoRa) puts modem into low-power mode.
 *  - Uses RTC wake-up timer to sleep for interval_sleep_sec.
 */
/* ======================= State_Sleep (LoRa sleep only) ======================= */
static void State_Sleep(void)
{
    if (app.usb_on && app.cfg.vocMode == VOC_CONT_USB) {
        LOGI("USB+VOC active -> skip STOP2");
        HAL_Delay(50);
        return;
    }

    if (!IsButtonReleased()) {
        LOGW("Sleep blocked: user button pressed");
        HAL_Delay(50);
        return;
    }

    HAL_GPIO_WritePin(PULL_CNTR_GPIO_Port, PULL_CNTR_Pin, GPIO_PIN_SET);
    PS_OFF(PS_SENSORS);

    if (app.epd_link == EPD_LINK_CONNECTED) {
        EPD_PowerOff();
    }

    /* Internal low-power mode only for LoRa modem */
    if (app.cfg.commsMode == COMMS_LORA)
        Wio_SleepEnter();

    app.cfg.interval_sleep_sec = (app.cfg.interval_sleep_sec == 0U) ? 1U : app.cfg.interval_sleep_sec;
    LOGI("Entering STOP2 for %lu s", (unsigned long)app.cfg.interval_sleep_sec);

    GPIO_PrepareForSleep_Simple();
    HAL_SuspendTick();
    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, app.cfg.interval_sleep_sec, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0);
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

    SystemClock_Config();
    HAL_ResumeTick();
}


/**
 * STATE_RECALIBRATION
 *
 * Force recalibration of the SCD41 sensor to 400 ppm baseline.
 * Triggered by a long button press.
 * Steps:
 *  - Power sensor rail and I2C pull-ups.
 *  - Wake up SCD41 and perform force recalibration.
 *  - Optionally re-initialize GUI (show recalibration icon/screen).
 *  - Clean up power rails and clear recalibration flag.
 */
static void State_Recalib(void)
{
    PS_ON(PS_SENSORS); HAL_Delay(10);
    HAL_GPIO_WritePin(PULL_CNTR_GPIO_Port, PULL_CNTR_Pin, GPIO_PIN_RESET); HAL_Delay(2);
    SCD41_WakeUp(&hi2c1); HAL_Delay(30);

    if (app.epd_link == EPD_LINK_CONNECTED) {
        EPD_PowerOn();
        HAL_Delay(50);
        GUI_Init();
    }

    LOGI("Recalibrating SCD41 sensor to 400 ppm...");
    SCD41_ForceRecalibration(&hi2c1, 400);

    if (app.epd_link == EPD_LINK_CONNECTED) {
        EPD_PowerOff();
    }

    app.recalibFlag = 0;
    HAL_GPIO_WritePin(PULL_CNTR_GPIO_Port, PULL_CNTR_Pin, GPIO_PIN_SET);
    PS_OFF(PS_SENSORS);

    LOGI("Recalibration done");
}

/**
 * STATE_CONFIG
 *
 * Silent UART4 configuration mode with E-Ink icon and OK/ERR bar.
 *
 * Behavior:
 *  - On first entry:
 *      * Stops VOC and sensors, optionally idles radio.
 *      * In Matter mode, powers ESP for QR/STATUS commands (WAKE low).
 *      * Draws configuration icon on E-Ink.
 *      * Starts the configuration console on UART4 (with inactivity timeout).
 *  - On each subsequent call:
 *      * Polls the console (command parser, timeouts).
 *      * Exits on second 5 s button press or console exit/inactivity.
 *  - On exit:
 *      * Stops console, re-initializes GUI, and resets boot flags so that
 *        the main application restarts in a clean INIT state.
 */
/* ======================= State_Config ======================= */
static void State_Config(void)
{
    static uint8_t started = 0;

    if (!started) {
        /* --- one-time entry --- */
        LOGB("CONFIG ENTER");

        /* Stop VOC + sensors and optionally put radio back to idle */
        if (app.voc_running) {
            VOC_LPTIM_Stop();
            voc_algo_inited = 0;
            app.voc_running = 0;
        }
        app.voc_tick = 0;

        /* In Matter mode enable ESP power for QR/STATUS commands; keep WAKE low */
        if (app.cfg.commsMode == COMMS_MATTER) {
            PS_ON(PS_RF);
            HAL_GPIO_WritePin(RF_RST_GPIO_Port, RF_RST_Pin, GPIO_PIN_SET); /* if used as EN */
            HAL_GPIO_WritePin(RF_GPIO1_GPIO_Port, RF_GPIO1_Pin, GPIO_PIN_RESET); /* WAKE LOW */
        }

        /* E-Ink: show configuration icon */
        if (app.epd_link == EPD_LINK_CONNECTED) {
            EPD_PowerOn();
            GUI_DrawConfIcon();
            /* You can keep EPD on for partial OK/ERR updates */
        }

        /* Start UART4 configuration console */
        ConfConsole_Init(&(ConfConsole_Init_t){
            .huart = &huart4,
            .cfg = &app.cfg,
            .sod_now = sod_now,
            .inactivity_s = 10*60
        });
        ConfConsole_Start();
        ConfConsole_PrintBanner(); /* UART: "CONFIG MODE", E-Ink: "Ready" */

        started = 1;
        return; /* after entering, immediately hand over control to console */
    }

    /* --- regular loop while in CONFIG state --- */
    ConfConsole_Poll();

    /* Exit on second 5 s button hold or on EXIT/inactivity */
    if (app.confFlag >= 2 || ConfConsole_ShouldExit()) {
        LOGB("CONFIG EXIT");

        ConfConsole_Stop();

        if (app.epd_link == EPD_LINK_CONNECTED) {
            EPD_PowerOn();
            GUI_Init();
            EPD_PowerOff();
        }

        /* Return to a "clean" state */
        app.boot_done      = 0;
        app.wakeUpFlag     = 0;
        app.gui_drawn_once = 0;
        app.confFlag       = 0;
        started            = 0;

        app.currentState = STATE_INIT;
        return;
    }
}

/* ======================= VOC helpers & LPTIM2 ======================= */
/**
 * Ensure VOC mode matches current configuration and USB state.
 *
 * - In CONFIG state, VOC is always forced off (safety guard).
 * - In continuous VOC mode with USB present:
 *      * Keeps sensor rail and I2C pull-ups ON.
 *      * Initializes VOC algorithm with 10 s sampling interval.
 *      * Starts LPTIM2 periodic interrupt.
 *      * Optionally powers E-Ink for quick UI updates.
 * - When VOC should stop:
 *      * Stops LPTIM2, de-inits algorithm, powers down sensors and E-Ink.
 */
static void VOC_Mode_Ensure(void)
{
    /* In CONFIG state keep VOC off (safety guard). */
    if (app.currentState == STATE_CONFIG) {
        if (app.voc_running) {
            LOGI("VOC stop (CONFIG)");
            VOC_LPTIM_Stop();
            voc_algo_inited = 0;
            HAL_GPIO_WritePin(PULL_CNTR_GPIO_Port, PULL_CNTR_Pin, GPIO_PIN_SET);
            PS_OFF(PS_SENSORS);
            if (app.epd_link == EPD_LINK_CONNECTED) {
                EPD_PowerOff();
            }
            app.voc_running = 0;
        }
        return;
    }

    const uint8_t should_run = (app.cfg.vocMode == VOC_CONT_USB) && (app.usb_on != 0);

    if (should_run && !app.voc_running) {
        LOGI("VOC start (USB ON)");

        /* Keep sensors rail and I2C pull-ups on while VOC runs */
        PS_ON(PS_SENSORS);
        HAL_GPIO_WritePin(PULL_CNTR_GPIO_Port, PULL_CNTR_Pin, GPIO_PIN_RESET);

        /* VOC algorithm @ 10 s interval */
        GasIndexAlgorithm_init_with_sampling_interval(
            &voc_algo, GasIndexAlgorithm_ALGORITHM_TYPE_VOC, 10.0f);
        voc_algo_inited = 1;
        app.voc_index_ready = 0;

        if (app.epd_link == EPD_LINK_CONNECTED) {
            EPD_PowerOn(); /* allow quick partial UI refresh if desired */
        }
        VOC_LPTIM_Start();
        app.voc_running = 1;
    }
    else if (!should_run && app.voc_running) {
        LOGI("VOC stop");
        VOC_LPTIM_Stop();
        voc_algo_inited = 0;

        HAL_GPIO_WritePin(PULL_CNTR_GPIO_Port, PULL_CNTR_Pin, GPIO_PIN_SET);
        PS_OFF(PS_SENSORS);
        if (app.epd_link == EPD_LINK_CONNECTED) {
            EPD_PowerOff();
        }
        app.voc_running = 0;
    }
}

/** Start LPTIM2 to tick every 10 s (ARR match IRQ). */
static void VOC_LPTIM_Start(void)
{
    /* Ensure peripheral low-level init (enables clock, NVIC, GPIO as defined in your MSP). */
    HAL_LPTIM_MspInit(&hlptim2);

    /* Make sure the counter is stopped before (re)configuring. */
    (void)HAL_LPTIM_Counter_Stop_IT(&hlptim2);

#if defined(__HAL_LPTIM_AUTORELOAD_SET)
    __HAL_LPTIM_AUTORELOAD_SET(&hlptim2, LPTIM2_10S_ARR);
#else
    __HAL_LPTIM_SET_AUTORELOAD(&hlptim2, LPTIM2_10S_ARR);
#endif

    /* Clear ARRM flag before starting to avoid a spurious first interrupt. */
    __HAL_LPTIM_CLEAR_FLAG(&hlptim2, LPTIM_FLAG_ARRM);

    /* Start periodic counter with interrupts. */
    if (HAL_LPTIM_Counter_Start_IT(&hlptim2) != HAL_OK) {
        LOGE("LPTIM2 start failed");
    } else {
        LOGI("LPTIM2 started");
    }
}

/** Stop LPTIM2 periodic tick and fully de-initialize to save power. */
static void VOC_LPTIM_Stop(void)
{
    /* Stop counter & interrupts first. */
    HAL_LPTIM_Counter_Stop_IT(&hlptim2);

    /* Disable the peripheral (keeps registers/flags quiet before MSP deinit). */
    __HAL_LPTIM_DISABLE(&hlptim2);

    /* Optionally clear pending flags (defensive). */
    __HAL_LPTIM_CLEAR_FLAG(&hlptim2, LPTIM_FLAG_ARRM);

    /* Low-level deinit (disables NVIC and peripheral clock as defined in your MSP). */
    HAL_LPTIM_MspDeInit(&hlptim2);
}


/* ======================= EPD power helpers ======================= */
/**
 * Power up E-Ink rail and SPI, initialize panel driver, and notify GUI
 * that the display has been resumed (for partial updates / cache).
 */
static void EPD_PowerOn(void)
{
    PS_ON(PS_EINK); HAL_Delay(100);
    MX_SPI1_Init();
    EPD_Pins_Init();
    DEV_Module_Init();
    EPD_2in13_V4_Init();
    GUI_OnEPDResumed();
}

/**
 * Put E-Ink into deep sleep, release SPI and pins to Hi-Z,
 * and disable the display power rail to save energy.
 */
static void EPD_PowerOff(void)
{
    EPD_2in13_V4_Sleep();
    DEV_Digital_Write(EPD_CS_PIN, 1);
    DEV_Digital_Write(EPD_DC_PIN, 0);
    HAL_SPI_DeInit(&hspi1);
    EPD_Pins_HiZ();
    HAL_Delay(80);
    PS_OFF(PS_EINK);
}

/**
 * Generate a reset pulse for ESP32-C6 by toggling RF_RST pin.
 * Ensures RF power rail is on, applies low-high sequence with delays,
 * and logs completion.
 */
static void ESP_ResetPulse(void)
{
    /* Make sure RF power rail is on */
    PS_ON(PS_RF);
    /* LOW -> short delay -> HIGH -> stabilization delay */
    HAL_GPIO_WritePin(RF_RST_GPIO_Port, RF_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(RF_RST_GPIO_Port, RF_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(300);
    LOGI("ESP reset pulse done");
}

/*
 * One-shot, quick E-Ink presence test during boot.
 * Temporarily powers the display rail, brings up SPI and pins,
 * performs a SWRESET with timeout, decides CONNECTED/DISCONNECTED,
 * and then always powers everything back down.
 */
static void EPD_BootDetect(void)
{
    if (app.epd_link != EPD_LINK_CONNECTED) {
        LOGW("E-Ink disabled by user -> skipping probe");
        return;
    }

    PS_ON(PS_EINK);
    HAL_Delay(100);
    MX_SPI1_Init();
    EPD_Pins_Init();
    DEV_Module_Init();
    HAL_Delay(10);

    if (!EPD_2in13_V4_BootDetect(1500)) {
        app.epd_link = EPD_LINK_DISCONNECTED;
        LOGW("E-Ink not responding -> DISCONNECTED (boot probe)");
    } else {
        LOGI("E-Ink present (boot probe OK)");
    }

    EPD_PowerOff();   /* always clean up safely */
}


/* -------------------------------------------------------------------------- */
/* Sending battery information to ESP32-C6 over UART (Matter text protocol)   */
/* Frame format:                                                              */
/*   DATA BAT KV LI=<mV> LS=<mV> USB=<0|1> CHG=<0..6>                         */
/* -------------------------------------------------------------------------- */
/**
 * Send current battery status to ESP32-C6 (Matter Power Source cluster).
 *
 * Uses a text frame of the form:
 *   DATA BAT KV LI=<mV> LS=<mV> USB=<0/1> CHG=<0..6>
 *
 * Only active when COMMS_MATTER is selected.
 */
static void App_SendBatteryToEsp(void)
{
    /* Send only in COMMS_MATTER mode */
    if (app.cfg.commsMode != COMMS_MATTER) {
        return;
    }

    uint16_t li_mV  = app.vbat_liion_mV;
    uint16_t ls_mV  = app.vbat_lsocl2_mV;
    uint8_t  usb_on = app.usb_on ? 1U : 0U;
    uint8_t  chg    = (uint8_t)app.charger_status; /* BQ25185_Status_t -> uint8_t */

    uint8_t ok = EspC6_DataBat(li_mV, ls_mV, usb_on, chg);
    if (!ok) {
        LOGW("ESP BAT TX failed (LI=%u mV, LS=%u mV, USB=%u, CHG=%u)",
             (unsigned)li_mV, (unsigned)ls_mV, (unsigned)usb_on, (unsigned)chg);
    } else {
        LOGI("ESP BAT TX OK (LI=%u mV, LS=%u mV, USB=%u, CHG=%u)",
             (unsigned)li_mV, (unsigned)ls_mV, (unsigned)usb_on, (unsigned)chg);
    }
}




/* ======================= LPTIM2 IRQ callback ======================= */
/**
 * LPTIM2 AutoReloadMatch callback:
 *  - Logs the tick.
 *  - When VOC is running and algorithm is initialized, processes the last
 *    raw VOC value into an index, clamps it, and stores timestamp and index.
 *  - Signals main loop via voc_index_ready flag to update GUI outside ISR.
 */
void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
    if (hlptim->Instance != LPTIM2) return;

    LOGI("LPTIM2 IRQ");
    if (app.voc_running && voc_algo_inited) {
        const uint16_t raw = app.voc_last_raw; /* 16-bit read is atomic on CM0+ */
        int32_t voc_idx32 = 0;
        GasIndexAlgorithm_process(&voc_algo, (int32_t)raw, &voc_idx32);
        if (voc_idx32 < 0) voc_idx32 = 0;
        if (voc_idx32 > 65535) voc_idx32 = 65535;

        app.voc_last_value  = (uint16_t)voc_idx32;
        app.voc_last_time   = g_timeData;      /* timestamp for UI */
        app.voc_index_ready = 1;               /* GUI (outside ISR) may do a partial update */
    }

    /* Ask the main loop to fetch a fresh raw sample when convenient */
    App_VOC_TickIRQ();
}

/* ======================= UART RX complete (feed config console) ======================= */
/**
 * Generic HAL UART RX complete callback.
 * For this application, it forwards received characters
 * into the configuration console (if active).
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* your other UART handling could be here ... */

    /* Feed config console (active only in CONFIG state). */
    ConfConsole_OnRxCplt(huart);
}

/* ======================= RTC WAKE UP IRQ callback ======================= */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	App_WakeUpIRQ();
}


/* ======================= GPIO EXT rising callback ======================= */

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BUTTON_Pin)
    {
    	App_WakeUpIRQ();
        ButtonHandler_EXTI_Callback();
    }

    if (GPIO_Pin == USB_ON_Pin)
    {
    	App_WakeUpIRQ();
    }

}

/* ======================= GPIO EXT falling callback ======================= */

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BUTTON_Pin)
    {
    	App_WakeUpIRQ();
        ButtonHandler_EXTI_Callback();
    }
}
