/* app.h
 *
 * Author: Evzen Steif
 *
 * Description:
 *   Public application interface and global runtime context for the
 *   air quality node state machine.
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include "main.h"
#include "RTC.h"
#include "bq25185.h"

/* ===== Feature modes ===== */
typedef enum {
    MEAS_CO2_TRH = 0,   /* SCD41 + SHT41 */
    MEAS_TRH_ONLY = 1   /* only SHT41 */
} MeasureMode_t;

typedef enum {
    VOC_DISABLED = 0,
    VOC_CONT_USB = 1     /* continuous VOC if and only if USB is present */
} VocMode_t;

typedef enum {
    COMMS_OFFLINE = 0,  /* no radio TX/RX, GUI-only */
    COMMS_LORA    = 1,  /* LoRaWAN enabled */
	COMMS_MATTER  = 2   /* Matter (ESP32-C6 bridge) enabled */
} CommsMode_t;

/* E-Ink link status */
typedef enum {
    EPD_LINK_DISCONNECTED = 0,
    EPD_LINK_CONNECTED    = 1
} EpdLink_t;

/* ===== App configuration (runtime) ===== */
typedef struct {
    MeasureMode_t measureMode;          /* measurement profile */
    VocMode_t     vocMode;              /* continuous VOC (USB only) or disabled */
    CommsMode_t   commsMode;            /* wireless ON/OFF */

    uint32_t interval_measure_sec;      /* CO2/T/RH or T/RH period */
    uint32_t interval_sleep_sec;        /* STOP2 sleep interval between cycles (if VOC not running) */
    uint32_t interval_time_req_sec;     /* network time sync period (LoRa) */
    uint32_t interval_bat_sec;          /* battery/USB/charger sampling period */

    uint32_t tx_min_interval_sec;   /* minimum spacing between uplinks (e.g., 120) */
    uint32_t tx_max_interval_sec;   /* heartbeat; set 0 to disable */
    uint16_t th_temp_01C;           /* ΔT threshold in 0.01 °C (e.g., 20 = 0.20°C) */
    uint16_t th_rh_01pct;           /* ΔRH threshold in 0.01 %RH (e.g., 100 = 1.00%) */
    uint16_t th_co2_ppm;            /* ΔCO2 threshold in ppm (e.g., 50) */
} AppConfig_t;

/* ===== App state machine ===== */
typedef enum {
    STATE_INIT,
    STATE_MEASURE,
    STATE_BAT_MEASURE,
    STATE_SEND_DATA,
    STATE_RECOVER,
    STATE_TIME_REQ,
    STATE_GUI,
    STATE_SLEEP,
    STATE_RECALIBRATION,
	STATE_CONFIG
} SystemState_t;

/* ===== Global runtime context ===== */
typedef struct {
    /* housekeeping */
    SystemState_t currentState;
    volatile uint8_t wakeUpFlag;      /* STOP2 wake flag */
    volatile uint8_t recalibFlag;     /* user button request */
    volatile uint8_t confFlag;        /* configuration mode requests (button/console) */

    /* config */
    AppConfig_t cfg;

    /* LoRa / ESP state */
    uint8_t Connected;
    uint8_t loraNoAckCount;
    uint8_t espFailCount;

    /* time sync */
    uint8_t  time_sync_ok;
    uint32_t lastTimeSyncSeconds;     /* seconds-of-day of last successful sync */

    /* simple schedulers (seconds-of-day, wrap at midnight) */
    uint32_t lastMeasureSeconds;
    uint32_t lastBatSeconds;

    /* latest readings */
    uint16_t temp_data;   /* 0.01 °C */
    uint16_t hum_data;    /* 0.01 %RH */
    uint16_t co2_data;    /* ppm */
    uint8_t  payload[6];

    /* power / battery */
    uint16_t vbat_lsocl2_mV;
    uint16_t vbat_liion_mV;
    uint8_t  usb_on;      /* 1=USB connected */
    BQ25185_Status_t charger_status;

    /* Display link status (set DISCONNECTED to hard-skip all GUI/EPD calls) */
    EpdLink_t epd_link;

    /* VOC */
    TimeData_t voc_last_time;      /* timestamp of the last VOC index */
    uint32_t   voc_last_value;     /* VOC index */
    volatile uint8_t  voc_tick;    /* flag from LPTIM2 IRQ every 10 s */
    uint8_t    voc_running;        /* continuous VOC active */
    volatile uint16_t voc_last_raw;
    volatile uint8_t  voc_index_ready;

    /* boot/sleep bookkeeping */
    uint8_t   boot_done;           /* prevents re-running cold boot */
    uint8_t   cfg_loaded;          /* config was already loaded from EEPROM */

    /* TX bookkeeping */
    uint8_t  last_sent_payload[6];
    uint32_t last_sent_seconds;    /* seconds-of-day of last uplink; 0 = never */
    uint8_t  baseline_ready;       /* we have a baseline payload (no uplink yet) */

    /* GUI change-detection bookkeeping */
    uint8_t  last_gui_payload[6];  /* last payload that was fully drawn */
    uint8_t  gui_drawn_once;       /* at least one full GUI_Display happened */
} AppContext_t;

/* ===== Global app context ===== */
extern AppContext_t app;

/* ===== API ===== */
void App_Init(void);
void AppConfig_SetDefaults(AppConfig_t *cfg);
void App_Run(void);

/* IRQ callbacks (called from main.c or button handler) */
void App_WakeUpIRQ(void);
void App_UserButtonIRQ_15s(void);
void App_UserButtonIRQ_5s(void);
void App_VOC_TickIRQ(void);   /* to be called from LPTIM2 AutoReloadMatch callback */

void App_Config_SaveToEeprom(void);

#endif /* INC_APP_H_ */
