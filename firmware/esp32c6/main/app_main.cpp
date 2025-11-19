/*
 * Matter Multi-Sensor with UART data from STM32 - TEXT PROTOCOL VERSION
 * Author: Evzen Steif
 *
 * Overview:
 *  - Exposes temperature, humidity, CO2 concentration and power source state as Matter endpoints.
 *  - Communicates with an STM32 over a simple ASCII UART "TEXT" protocol (DATA/STATUS/COMM/etc.).
 *  - Handles commissioning (QR/manual codes, open/close commissioning window) over UART commands.
 *  - Uses a WAKE/READY GPIO handshake and PM lock to cooperate with light sleep on an ESP32-C6 SED.
 */

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_sleep.h>
#include "driver/rtc_io.h"

#if CONFIG_PM_ENABLE
#include <esp_pm.h>
#endif

#include <esp_matter.h>
#include <esp_matter_ota.h>
#include <common_macros.h>
#include <app_priv.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#include <platform/ConnectivityManager.h>
#endif

#include <app/InteractionModelEngine.h>
#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <app-common/zap-generated/cluster-objects.h>

/* === Setup payload (QR/manual) & providers === */
#include <platform/DeviceInstanceInfoProvider.h>
#include <platform/CommissionableDataProvider.h>
#include <setup_payload/SetupPayload.h>
#include <setup_payload/ManualSetupPayloadGenerator.h>
#include <setup_payload/QRCodeSetupPayloadGenerator.h>

/* NEW: schedule work helpers */
#include <platform/PlatformManager.h>
#include <inttypes.h>
#include <stdarg.h>
#include <string.h>

#include <platform/ConfigurationManager.h>
using chip::DeviceLayer::ConfigurationMgr;

static void InitNodeBasicInformation()
{
    static constexpr char kSerialNumber[] = "ESP32C6-SED-000123";
    static constexpr char kManufacturingDate[] = "2025-11-01";
    (void)ConfigurationMgr().StoreSerialNumber(kSerialNumber, strlen(kSerialNumber));
    (void)ConfigurationMgr().StoreSoftwareVersion(10000); // e.g. 1.0.0 => 10000
    (void)ConfigurationMgr().StoreHardwareVersion(1);
    (void)ConfigurationMgr().StoreManufacturingDate(kManufacturingDate, strlen(kManufacturingDate));
}

static const char *TAG = "app_main";

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;
namespace DeviceEventType = chip::DeviceLayer::DeviceEventType;

/* ===== UART & GPIO config ===== */
#define UART_NUM       UART_NUM_1
#define UART_TX_PIN    GPIO_NUM_5
#define UART_RX_PIN    GPIO_NUM_4
#define UART_BAUD_RATE 9600
#define UART_BUF_SIZE  256
#define WAKE_PIN       GPIO_NUM_2
static constexpr gpio_num_t kReadyPin = GPIO_NUM_3;

/* App-level limits for basic sanity checking of sensor values */
static constexpr int16_t  kTemperatureMinValue = -4000;
static constexpr int16_t  kTemperatureMaxValue = 12500;
static constexpr uint16_t kHumidityMinValue    = 0;
static constexpr uint16_t kHumidityMaxValue    = 10000;
static constexpr float    kCO2MinPpm           = 400.0f;
static constexpr float    kCO2MaxPpm           = 5000.0f;

constexpr auto k_timeout_seconds = 600;
constexpr uint8_t kAirQualityClusterRevision = 1;

/* CO2 cluster/attributes (custom concentration measurement cluster) */
constexpr uint32_t kClusterId_CO2                = 0x040D;
constexpr uint32_t kAttrId_CO2_MeasuredValue     = 0x0000;
constexpr uint32_t kAttrId_CO2_MinMeasured       = 0x0001;
constexpr uint32_t kAttrId_CO2_MaxMeasured       = 0x0002;
constexpr uint32_t kAttrId_CO2_MeasurementUnit   = 0x0008; // 0x00=PPM
constexpr uint32_t kAttrId_CO2_MeasurementMedium = 0x0009; // 0x00=Air
constexpr uint32_t kAttrId_CO2_Uncertainty       = 0x0007; // ± measurement error (nullable float)

static uint16_t g_temp_ep_id  = 0;
static uint16_t g_humid_ep_id = 0;
static uint16_t g_co2_ep_id   = 0;
static int16_t  g_current_temp = 0;
static uint16_t g_current_humidity = 0;
static uint16_t g_current_co2 = 0;

static uint16_t g_ps_usb_ep_id    = 0;
static uint16_t g_ps_liion_ep_id  = 0;
static uint16_t g_ps_lsocl2_ep_id = 0;
static SemaphoreHandle_t s_reply_mutex = nullptr;

/* Aggregated metadata for all power sources, received from STM32 */
typedef struct {
    bool     usb_on;
    uint16_t vbat_liion_mV;
    uint16_t vbat_lsocl2_mV;
    uint8_t  charger_status; /* BQ25185_Status_t from STM, 0..6 */
} power_meta_t;

static power_meta_t g_power_meta = {};

typedef enum {
    BQ25185_STATUS_USB_NOT_CONNECTED = 0,
    BQ25185_STATUS_CHARGE_DONE,
    BQ25185_STATUS_CHARGING,
    BQ25185_STATUS_RECOVERABLE_FAULT,
    BQ25185_STATUS_FATAL_FAULT,
    BQ25185_STATUS_BATTERY_MISSING,
    BQ25185_STATUS_UNKNOWN
} BQ25185_Status_t;

static SemaphoreHandle_t g_data_mutex = nullptr;
static volatile uint32_t g_wake_count = 0;
static TaskHandle_t s_uart_task_handle = nullptr;

#if CONFIG_PM_ENABLE
static esp_pm_lock_handle_t s_pm_lock = nullptr;
#endif

static bool s_gpio_isr_installed = false;

static volatile bool s_cw_open = false;
static volatile bool s_cw_close_by_app = false;  // true when commissioning window is closed by our app logic

/* ===== Fabric/session helper: expire all sessions & subscriptions for a fabric ===== */
static void ExpireAllFabricSessionsAndSubscriptions(chip::FabricIndex fabric_index)
{
    auto & server = chip::Server::GetInstance();
    server.GetSecureSessionManager().ExpireAllSessionsForFabric(fabric_index);

#if CHIP_CONFIG_ENABLE_SESSION_RESUMPTION
    if (auto * session_resumption = server.GetSessionResumptionStorage())
    {
        (void)session_resumption->DeleteAll(fabric_index);
    }
#endif

#if CHIP_CONFIG_ENABLE_READ_CLIENT
    chip::app::InteractionModelEngine::GetInstance()->ShutdownSubscriptions(fabric_index);
    if (auto * subscription_resumption = server.GetSubscriptionResumptionStorage())
    {
        (void)subscription_resumption->DeleteAll(fabric_index);
    }
#endif
}

/* ===== PM lock helpers (NO_LIGHT_SLEEP while UART IO, commissioning, etc.) ===== */
static inline bool pm_lock_acquire_if(void)
{
#if CONFIG_PM_ENABLE
    if (s_pm_lock && esp_pm_lock_acquire(s_pm_lock) == ESP_OK) return true;
#endif
    return false;
}

static inline void pm_lock_release_if(bool held)
{
#if CONFIG_PM_ENABLE
    if (held && s_pm_lock) (void)esp_pm_lock_release(s_pm_lock);
#else
    (void)held;
#endif
}

/* ===== READY pin =====
 * Output pin driven to indicate that the ESP32 is ready to receive/process a frame.
 */
static void ready_pin_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << kReadyPin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(kReadyPin, 0));
    ESP_LOGI(TAG, "Ready pin GPIO%d configured (initial LOW)", kReadyPin);
}

static inline void ready_pin_set(bool level)
{
    gpio_set_level(kReadyPin, level ? 1 : 0);
}

/* --- WAKE read helper
 * Return current logical level of the WAKE pin, using RTC IO if available.
 */
static inline int get_wake_level(void)
{
#if SOC_RTCIO_PIN_COUNT > 0
    if (WAKE_PIN <= GPIO_NUM_7)
        return rtc_gpio_get_level((gpio_num_t)WAKE_PIN);
#endif
    return gpio_get_level(WAKE_PIN);
}

/* ===== WAKE ISR =====
 * ISR is triggered on the rising edge of WAKE. It wakes the UART task via task notification.
 */
static void IRAM_ATTR wake_gpio_isr_handler(void *arg)
{
    BaseType_t hpw = pdFALSE;
    if (s_uart_task_handle) {
        g_wake_count++;
        vTaskNotifyGiveFromISR(s_uart_task_handle, &hpw);
    }
    if (hpw == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/* ===== UART init =====
 * Configure UART driver and wakeup behavior for low power.
 */
static void uart_init(void)
{
    const uart_config_t cfg = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk= UART_SCLK_DEFAULT,
    };

    /* RX ring-buffer = 2× UART_BUF_SIZE, TX ring-buffer = 1024B (smooth UART logging) */
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(gpio_pullup_en((gpio_num_t)UART_RX_PIN));
    ESP_ERROR_CHECK(uart_set_rx_full_threshold(UART_NUM, 1));
    ESP_ERROR_CHECK(uart_set_rx_timeout(UART_NUM, 4));
    ESP_ERROR_CHECK(uart_set_wakeup_threshold(UART_NUM, 3));     // minimum allowed by hardware
    ESP_ERROR_CHECK(esp_sleep_enable_uart_wakeup(UART_NUM));     // C6: UART0 and UART1 support wakeup
}

/* ===== WAKE pin init =====
 * Configure WAKE as input, with both GPIO and light-sleep wakeup, and attach ISR.
 */
static void wake_pin_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << WAKE_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(gpio_wakeup_enable(WAKE_PIN, GPIO_INTR_HIGH_LEVEL));
    ESP_ERROR_CHECK(gpio_set_intr_type(WAKE_PIN, GPIO_INTR_POSEDGE));

    if (!s_gpio_isr_installed) {
        esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (err == ESP_OK || err == ESP_ERR_INVALID_STATE)
            s_gpio_isr_installed = true;
        else
            ESP_LOGE(TAG, "Failed to install GPIO ISR service: %d", err);
    }

    ESP_ERROR_CHECK(gpio_isr_handler_add(WAKE_PIN, wake_gpio_isr_handler, nullptr));
    ESP_ERROR_CHECK(gpio_intr_enable(WAKE_PIN));
    ESP_LOGI(TAG, "Wake pin GPIO%d configured (ISR on rising edge, wake on HIGH)", WAKE_PIN);
}

/* Busy-wait until WAKE reaches expected_level or timeout */
static bool wait_for_wake_pin_level(int expected_level, uint32_t timeout_us)
{
    const uint32_t step_us = 50;
    uint32_t elapsed = 0;
    while (get_wake_level() != expected_level && elapsed < timeout_us) {
        esp_rom_delay_us(step_us);
        elapsed += step_us;
    }
    return get_wake_level() == expected_level;
}

/* ===== UART helpers (text I/O) ===== */
static inline void uart_write_crlf(void) {
    uart_write_bytes(UART_NUM, "\r\n", 2);
}

/* Format and send a single line over UART, appending CRLF */
static void uart_send_line_v(const char *fmt, va_list args)
{
    char buf[256];
    va_list ap;
    va_copy(ap, args);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;

    bool pm = pm_lock_acquire_if();
    uart_write_bytes(UART_NUM, buf, strnlen(buf, sizeof(buf)));
    uart_write_crlf();
    (void)uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(200)); // ensure TX completes
    pm_lock_release_if(pm);
}

/* Thread-safe printf-style helper for UART replies (uses recursive mutex) */
static void replyf(const char* fmt, ...)
{
    if (s_reply_mutex) {
        xSemaphoreTakeRecursive(s_reply_mutex, portMAX_DELAY);
    }
    va_list ap;
    va_start(ap, fmt);
    uart_send_line_v(fmt, ap);
    va_end(ap);
    if (s_reply_mutex) {
        xSemaphoreGiveRecursive(s_reply_mutex);
    }
}


/* "Block" send of a prebuilt buffer in one go over UART */
static void uart_write_block(const char* data, size_t len) {
    bool pm = pm_lock_acquire_if();
    uart_write_bytes(UART_NUM, data, len);
    (void)uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(300));
    pm_lock_release_if(pm);
}

/* Read a line from UART into 'out', stripping CR/LF, with timeout.
 * Returns number of bytes read (not including terminator).
 */
static int read_line(char* out, size_t out_sz, TickType_t timeout_ticks)
{
    size_t L = 0;
    TickType_t deadline = xTaskGetTickCount() + timeout_ticks;

    bool pm = pm_lock_acquire_if(); // keep light sleep disabled during RX window
    while (xTaskGetTickCount() < deadline && L + 1 < out_sz) {
        uint8_t c;
        int r = uart_read_bytes(UART_NUM, &c, 1, pdMS_TO_TICKS(30));
        if (r == 1) {
            if (c == '\n') break;
            if (c != '\r') out[L++] = (char)c;
        }
    }
    pm_lock_release_if(pm);

    out[L] = '\0';
    return (int)L;
}

/* ===== Battery status helpers ===== */
static inline bool battery_present_liion(uint16_t mv)
{
    return (mv >= 1000U);
}

static inline bool battery_present_lsocl2(uint16_t mv)
{
    return (mv >= 2000U);
}

/* Map raw charger state into Matter BatChargeStateEnum for the Li-Ion source */
static chip::app::Clusters::PowerSource::BatChargeStateEnum
map_charge_state_liion(uint8_t raw_status, bool usb_on)
{
    namespace PS = chip::app::Clusters::PowerSource;

    BQ25185_Status_t st = (BQ25185_Status_t)raw_status;

    if (!usb_on) {
        return PS::BatChargeStateEnum::kIsNotCharging;
    }

    switch (st) {
    case BQ25185_STATUS_CHARGING:
        return PS::BatChargeStateEnum::kIsCharging;
    case BQ25185_STATUS_CHARGE_DONE:
        return PS::BatChargeStateEnum::kIsAtFullCharge;
    case BQ25185_STATUS_USB_NOT_CONNECTED:
        return PS::BatChargeStateEnum::kIsNotCharging;
    case BQ25185_STATUS_BATTERY_MISSING:
        return PS::BatChargeStateEnum::kIsNotCharging;
    case BQ25185_STATUS_RECOVERABLE_FAULT:
    case BQ25185_STATUS_FATAL_FAULT:
        return PS::BatChargeStateEnum::kIsNotCharging;
    default:
        return PS::BatChargeStateEnum::kUnknown;
    }
}

/* Coarse thresholds for Li-Ion (1S) charge level classification */
static chip::app::Clusters::PowerSource::BatChargeLevelEnum
map_charge_level_liion(uint16_t mv)
{
    namespace PS = chip::app::Clusters::PowerSource;

    if (!battery_present_liion(mv)) {
        return PS::BatChargeLevelEnum::kCritical;
    }
    if (mv <= 3300U) {
        return PS::BatChargeLevelEnum::kCritical;
    }
    if (mv <= 3600U) {
        return PS::BatChargeLevelEnum::kWarning;
    }
    return PS::BatChargeLevelEnum::kOk;
}

/* Coarse thresholds for LiSOCl2 primary battery */
static chip::app::Clusters::PowerSource::BatChargeLevelEnum
map_charge_level_lsocl2(uint16_t mv)
{
    namespace PS = chip::app::Clusters::PowerSource;

    if (!battery_present_lsocl2(mv)) {
        return PS::BatChargeLevelEnum::kUnknownEnumValue;
    }
    if (mv <= 2900U) {
        return PS::BatChargeLevelEnum::kCritical;
    }
    if (mv <= 3200U) {
        return PS::BatChargeLevelEnum::kWarning;
    }
    return PS::BatChargeLevelEnum::kOk;
}

/* Create and configure three Power Source endpoints: USB, Li-Ion, LiSOCl2 */
static void init_power_source_endpoints(node_t *node)
{
    using namespace esp_matter;
    using namespace esp_matter::endpoint;

    endpoint::power_source::config_t cfg_usb{};
    endpoint::power_source::config_t cfg_li{};
    endpoint::power_source::config_t cfg_ls{};
    namespace PS = chip::app::Clusters::PowerSource;

    /* Configure feature sets for each Power Source cluster */
    cfg_usb.power_source.feature_flags = cluster::power_source::feature::wired::get_id();
    cfg_usb.power_source.features.wired.wired_current_type =
        static_cast<uint8_t>(PS::WiredCurrentTypeEnum::kDc);

    cfg_li.power_source.feature_flags =
        cluster::power_source::feature::battery::get_id() |
        cluster::power_source::feature::rechargeable::get_id();
    cfg_li.power_source.features.battery.bat_replaceability =
        static_cast<uint8_t>(PS::BatReplaceabilityEnum::kNotReplaceable);
    cfg_li.power_source.features.rechargeable.bat_functional_while_charging = true;

    cfg_ls.power_source.feature_flags =
        cluster::power_source::feature::battery::get_id() |
        cluster::power_source::feature::replaceable::get_id();
    cfg_ls.power_source.features.replaceable.bat_quantity = 1;

    /* Create Power Source endpoints */
    endpoint_t *ep_usb = endpoint::power_source::create(node, &cfg_usb, ENDPOINT_FLAG_NONE, nullptr);
    ABORT_APP_ON_FAILURE(ep_usb != nullptr, ESP_LOGE(TAG, "Failed to create USB Power Source endpoint"));
    g_ps_usb_ep_id = endpoint::get_id(ep_usb);

    endpoint_t *ep_li = endpoint::power_source::create(node, &cfg_li, ENDPOINT_FLAG_NONE, nullptr);
    ABORT_APP_ON_FAILURE(ep_li != nullptr, ESP_LOGE(TAG, "Failed to create Li-Ion Power Source endpoint"));
    g_ps_liion_ep_id = endpoint::get_id(ep_li);

    endpoint_t *ep_ls = endpoint::power_source::create(node, &cfg_ls, ENDPOINT_FLAG_NONE, nullptr);
    ABORT_APP_ON_FAILURE(ep_ls != nullptr, ESP_LOGE(TAG, "Failed to create LiSOCl2 Power Source endpoint"));
    g_ps_lsocl2_ep_id = endpoint::get_id(ep_ls);

    cluster_t *cl_usb = cluster::get(ep_usb, PowerSource::Id);
    cluster_t *cl_li  = cluster::get(ep_li, PowerSource::Id);
    cluster_t *cl_ls  = cluster::get(ep_ls, PowerSource::Id);
    ABORT_APP_ON_FAILURE(cl_usb != nullptr, ESP_LOGE(TAG, "Failed to get USB Power Source cluster"));
    ABORT_APP_ON_FAILURE(cl_li  != nullptr, ESP_LOGE(TAG, "Failed to get Li-Ion Power Source cluster"));
    ABORT_APP_ON_FAILURE(cl_ls  != nullptr, ESP_LOGE(TAG, "Failed to get LiSOCl2 Power Source cluster"));

    /* --- Priority / primary source ordering via Order attribute ---
     * 0 = first choice, 1 = second, 2 = third, ...
     * Here: 0 = USB, 1 = Li-Ion, 2 = LiSOCl2
     */
    ABORT_APP_ON_FAILURE(
        cluster::power_source::attribute::create_order(cl_usb, 0, 0, 0xFF) != nullptr,
        ESP_LOGE(TAG, "Failed to create USB Order attribute")
    );
    ABORT_APP_ON_FAILURE(
        cluster::power_source::attribute::create_order(cl_li, 1, 0, 0xFF) != nullptr,
        ESP_LOGE(TAG, "Failed to create Li-Ion Order attribute")
    );
    ABORT_APP_ON_FAILURE(
        cluster::power_source::attribute::create_order(cl_ls, 2, 0, 0xFF) != nullptr,
        ESP_LOGE(TAG, "Failed to create LiSOCl2 Order attribute")
    );

    ABORT_APP_ON_FAILURE(cluster::power_source::attribute::create_wired_present(cl_usb, false) != nullptr,
                         ESP_LOGE(TAG, "Failed to create WiredPresent attribute"));

    nullable<uint32_t> min_mv(0);
    nullable<uint32_t> max_mv(5000);
    nullable<uint32_t> zero_mv(0);

    uint8_t chem_li_val = static_cast<uint8_t>(PS::BatApprovedChemistryEnum::kLithiumIon);
    uint8_t chem_ls_val = static_cast<uint8_t>(PS::BatApprovedChemistryEnum::kLithiumThionylChloride);

    ABORT_APP_ON_FAILURE(cluster::power_source::attribute::create_bat_voltage(cl_li, zero_mv, min_mv, max_mv) != nullptr,
                         ESP_LOGE(TAG, "Failed to create Li-Ion BatVoltage attribute"));
    ABORT_APP_ON_FAILURE(cluster::power_source::attribute::create_bat_voltage(cl_ls, zero_mv, min_mv, max_mv) != nullptr,
                         ESP_LOGE(TAG, "Failed to create LiSOCl2 BatVoltage attribute"));

    ABORT_APP_ON_FAILURE(cluster::power_source::attribute::create_bat_present(cl_li, false) != nullptr,
                         ESP_LOGE(TAG, "Failed to create Li-Ion BatPresent attribute"));
    ABORT_APP_ON_FAILURE(cluster::power_source::attribute::create_bat_present(cl_ls, false) != nullptr,
                         ESP_LOGE(TAG, "Failed to create LiSOCl2 BatPresent attribute"));

    ABORT_APP_ON_FAILURE(cluster::power_source::attribute::create_bat_approved_chemistry(cl_li, chem_li_val, 0, 0xFF) != nullptr,
                         ESP_LOGE(TAG, "Failed to create Li-Ion BatApprovedChemistry attribute"));
    ABORT_APP_ON_FAILURE(cluster::power_source::attribute::create_bat_approved_chemistry(cl_ls, chem_ls_val, 0, 0xFF) != nullptr,
                         ESP_LOGE(TAG, "Failed to create LiSOCl2 BatApprovedChemistry attribute"));

    ESP_LOGI(TAG, "Power Source EPs: USB=%u, Li-Ion=%u, LiSOCl2=%u",
             g_ps_usb_ep_id, g_ps_liion_ep_id, g_ps_lsocl2_ep_id);
}


/* Push g_power_meta into the three Matter Power Source clusters */
static void update_power_source_clusters_from_meta(void)
{
    using namespace esp_matter;
    using namespace esp_matter::attribute;
    namespace PS = chip::app::Clusters::PowerSource;

    /* USB (wired) */
    if (g_ps_usb_ep_id) {
        PS::PowerSourceStatusEnum st =
            g_power_meta.usb_on ? PS::PowerSourceStatusEnum::kActive
                                : PS::PowerSourceStatusEnum::kUnavailable;

        esp_matter_attr_val_t v_status = esp_matter_enum8((uint8_t)st);
        update(g_ps_usb_ep_id, PS::Id,
               PS::Attributes::Status::Id, &v_status);

        esp_matter_attr_val_t v_p = esp_matter_bool(g_power_meta.usb_on);
        update(g_ps_usb_ep_id, PS::Id,
               PS::Attributes::WiredPresent::Id, &v_p);
    }

    /* Li-Ion / Li-Po (rechargeable) */
    if (g_ps_liion_ep_id) {
        uint16_t mv   = g_power_meta.vbat_liion_mV;
        bool     pres = battery_present_liion(mv);
        BQ25185_Status_t raw = (BQ25185_Status_t)g_power_meta.charger_status;

        // Defaults
        PS::PowerSourceStatusEnum st      = PS::PowerSourceStatusEnum::kActive;
        auto chg_state = map_charge_state_liion(g_power_meta.charger_status, g_power_meta.usb_on);
        auto lvl       = map_charge_level_liion(mv);

        if (!pres || raw == BQ25185_STATUS_BATTERY_MISSING) {
            // Battery physically not present: mark source as Unavailable and NotPresent
            st        = PS::PowerSourceStatusEnum::kUnavailable;
            chg_state = PS::BatChargeStateEnum::kIsNotCharging;
        } else if (raw == BQ25185_STATUS_RECOVERABLE_FAULT ||
                   raw == BQ25185_STATUS_FATAL_FAULT) {
            st        = PS::PowerSourceStatusEnum::kUnavailable;
            chg_state = PS::BatChargeStateEnum::kIsNotCharging;
        }

        // Status
        esp_matter_attr_val_t v_status = esp_matter_enum8((uint8_t)st);
        update(g_ps_liion_ep_id, PS::Id,
               PS::Attributes::Status::Id, &v_status);

        // Report actual measured voltage at all times
        esp_matter_attr_val_t v_mv =
            esp_matter_nullable_uint32(nullable<uint32_t>((uint32_t)mv));
        update(g_ps_liion_ep_id, PS::Id,
               PS::Attributes::BatVoltage::Id, &v_mv);

        // Presence
        esp_matter_attr_val_t v_pres = esp_matter_bool(pres);
        update(g_ps_liion_ep_id, PS::Id,
               PS::Attributes::BatPresent::Id, &v_pres);

        // ReplacementNeeded – for Li-Ion this is meaningful only at "very low" voltages;
        // kept at <= 3.2 V as before.
        bool repl_needed = pres && (mv <= 3200U);
        esp_matter_attr_val_t v_repl = esp_matter_bool(repl_needed);
        update(g_ps_liion_ep_id, PS::Id,
               PS::Attributes::BatReplacementNeeded::Id, &v_repl);

        // BatChargeLevel
        esp_matter_attr_val_t v_lvl;
        if (!pres) {
            // Battery not installed → Unknown
            v_lvl = esp_matter_enum8((uint8_t)PS::BatChargeLevelEnum::kUnknownEnumValue);
        } else {
            v_lvl = esp_matter_enum8((uint8_t)lvl);
        }
        update(g_ps_liion_ep_id, PS::Id,
               PS::Attributes::BatChargeLevel::Id, &v_lvl);

        // BatChargeState
        esp_matter_attr_val_t v_chg = esp_matter_enum8((uint8_t)chg_state);
        update(g_ps_liion_ep_id, PS::Id,
               PS::Attributes::BatChargeState::Id, &v_chg);
    }

    /* LiSOCl2 – non-rechargeable primary battery */
    if (g_ps_lsocl2_ep_id) {
        namespace PS = PowerSource;

        uint16_t mv   = g_power_meta.vbat_lsocl2_mV;
        bool     pres = battery_present_lsocl2(mv);
        auto     lvl  = map_charge_level_lsocl2(mv);

        PS::PowerSourceStatusEnum st;
        bool repl_needed;

        if (!pres) {
            // Battery physically not present – source is "Unavailable",
            // without replacement request and with no useful voltage.
            st           = PS::PowerSourceStatusEnum::kUnavailable;
            repl_needed  = false;
            lvl          = PS::BatChargeLevelEnum::kUnknownEnumValue; // be explicit
        } else {
            // Battery is present, derive state from voltage
            if (mv <= 2900U) {
                st = PS::PowerSourceStatusEnum::kUnavailable;
            } else {
                st = PS::PowerSourceStatusEnum::kActive;
            }
            repl_needed = (mv <= 3000U);
            // lvl is already computed by map_charge_level_lsocl2()
        }

        esp_matter_attr_val_t v_status =
            esp_matter_enum8((uint8_t)st);
        update(g_ps_lsocl2_ep_id, PS::Id,
               PS::Attributes::Status::Id, &v_status);

        esp_matter_attr_val_t v_mv =
            esp_matter_nullable_uint32(nullable<uint32_t>((uint32_t)mv));
        update(g_ps_lsocl2_ep_id, PS::Id,
               PS::Attributes::BatVoltage::Id, &v_mv);

        esp_matter_attr_val_t v_pres =
            esp_matter_bool(pres);
        update(g_ps_lsocl2_ep_id, PS::Id,
               PS::Attributes::BatPresent::Id, &v_pres);

        esp_matter_attr_val_t v_repl =
            esp_matter_bool(repl_needed);
        update(g_ps_lsocl2_ep_id, PS::Id,
               PS::Attributes::BatReplacementNeeded::Id, &v_repl);

        esp_matter_attr_val_t v_lvl =
            esp_matter_enum8((uint8_t)lvl);
        update(g_ps_lsocl2_ep_id, PS::Id,
               PS::Attributes::BatChargeLevel::Id, &v_lvl);
    }
}


/* ===== Matter cluster updates =====
 * Push new T/RH/CO2 values into Matter attributes.
 */
static void update_matter_clusters(int16_t temp, uint16_t humidity, uint16_t co2)
{
    if (g_temp_ep_id) {
        esp_matter_attr_val_t v = esp_matter_nullable_int16(temp);
        attribute::update(g_temp_ep_id, TemperatureMeasurement::Id,
                          TemperatureMeasurement::Attributes::MeasuredValue::Id, &v);
    }

    if (g_humid_ep_id) {
        esp_matter_attr_val_t v = esp_matter_nullable_uint16(humidity);
        attribute::update(g_humid_ep_id, RelativeHumidityMeasurement::Id,
                          RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, &v);
    }

    if (g_co2_ep_id) {
        esp_matter_attr_val_t v1 = esp_matter_nullable_float((float)co2);
        attribute::update(g_co2_ep_id, kClusterId_CO2, kAttrId_CO2_MeasuredValue, &v1);

        using AQ = chip::app::Clusters::AirQuality::AirQualityEnum;
        uint8_t aq = (co2 < 800)  ? (uint8_t)AQ::kGood :
                     (co2 < 1000) ? (uint8_t)AQ::kFair :
                     (co2 < 1400) ? (uint8_t)AQ::kModerate :
                     (co2 < 2000) ? (uint8_t)AQ::kPoor :
                     (co2 < 3000) ? (uint8_t)AQ::kVeryPoor :
                                    (uint8_t)AQ::kExtremelyPoor;
        esp_matter_attr_val_t v2 = esp_matter_enum8(aq);
        attribute::update(g_co2_ep_id, AirQuality::Id, AirQuality::Attributes::AirQuality::Id, &v2);
    }
}

/* === QR/Manual helpers ===
 * Build QR and manual setup payload strings using Matter providers.
 */
static bool get_qr_and_manual(char* qr, size_t qr_sz, char* manual, size_t man_sz)
{
    using namespace chip;
    using namespace chip::DeviceLayer;

    auto * inst = GetDeviceInstanceInfoProvider();
    auto * comm = GetCommissionableDataProvider();
    if (!inst || !comm) return false;

    uint16_t vid = 0, pid = 0;
    uint32_t passcode = 0;
    SetupDiscriminator disc;
    uint16_t disc_long = 0;

    if (inst->GetVendorId(vid) != CHIP_NO_ERROR) return false;
    if (inst->GetProductId(pid) != CHIP_NO_ERROR) return false;
    if (comm->GetSetupPasscode(passcode) != CHIP_NO_ERROR) return false;
    if (comm->GetSetupDiscriminator(disc_long) != CHIP_NO_ERROR) return false;
    disc.SetLongValue(disc_long);

    SetupPayload pl;
    pl.version = 0;
    pl.commissioningFlow = CommissioningFlow::kStandard;
    pl.vendorID  = vid;
    pl.productID = pid;
    pl.setUpPINCode = passcode;
    pl.discriminator = disc;

    RendezvousInformationFlags rflags;
    rflags.Set(RendezvousInformationFlag::kBLE);
    rflags.Set(RendezvousInformationFlag::kOnNetwork);
    pl.rendezvousInformation.SetValue(rflags);

    std::string qr_s, man_s;
    QRCodeSetupPayloadGenerator qgen(pl);
    if (qgen.payloadBase38Representation(qr_s) != CHIP_NO_ERROR) return false;

    ManualSetupPayloadGenerator mgen(pl);
    if (mgen.payloadDecimalStringRepresentation(man_s) != CHIP_NO_ERROR) return false;

    strlcpy(qr, qr_s.c_str(), qr_sz);
    strlcpy(manual, man_s.c_str(), man_sz);
    return true;
}

/* Send the current QR and manual codes over UART as COMM lines */
static void send_qr_manual_now()
{
    char qr[192] = {0}, man[64] = {0};
    if (get_qr_and_manual(qr, sizeof(qr), man, sizeof(man))) {
        replyf("COMM QR %s", qr);
        replyf("COMM MANUAL %s", man);
    } else {
        replyf("COMM QR na");
        replyf("COMM MANUAL na");
    }
}

/* ===== DATA ... line handlers ===== */

/* Parse 12 hex digits into 6 bytes (used for compact DATA HEX format) */
static bool parse_hex12(const char* hex, uint8_t out[6])
{
    if (!hex) return false;
    size_t L = strlen(hex);
    if (L != 12) return false;
    for (int i = 0; i < 6; i++) {
        char b[3] = { hex[i*2], hex[i*2+1], 0 };
        char* e = nullptr;
        long v = strtol(b, &e, 16);
        if (e == b || v < 0 || v > 255) return false;
        out[i] = (uint8_t)v;
    }
    return true;
}


/* Parse: "KV LI=<u16> LS=<u16> USB=<0|1> CHG=<0..255>" and update power metadata */
static bool handle_bat_line(const char *rest)
{
    unsigned li  = 0;
    unsigned ls  = 0;
    unsigned usb = 0;
    unsigned chg = 0;

    if (sscanf(rest, "KV LI=%u LS=%u USB=%u CHG=%u", &li, &ls, &usb, &chg) != 4) {
        ESP_LOGW(TAG, "BAT KV parse failed: %s", rest);
        return false;
    }

    if (li > 65535U || ls > 65535U || usb > 1U || chg > 255U) {
        ESP_LOGW(TAG, "BAT KV out of range: %s", rest);
        return false;
    }

    g_power_meta.vbat_liion_mV  = (uint16_t)li;
    g_power_meta.vbat_lsocl2_mV = (uint16_t)ls;
    g_power_meta.usb_on         = (usb != 0U);
    g_power_meta.charger_status = (uint8_t)chg;

    update_power_source_clusters_from_meta();
    return true;
}

/* Parse incoming "DATA ..." line, update local T/RH/CO2 and relevant clusters.
 * Supported payloads:
 *  - "HEX <12hex>"
 *  - "KV T=<i16> RH=<u16> CO2=<u16>"
 *  - "BAT KV LI=<u16> LS=<u16> USB=<0|1> CHG=<0..255>"
 */
static bool handle_data_line(const char* rest)
{
    // rest:
    //  - "HEX <12hex>"
    //  - "KV T=<i16> RH=<u16> CO2=<u16>"
    //  - "BAT KV LI=<u16> LS=<u16> USB=<0|1> CHG=<0..255>"

    int16_t t = 0; uint16_t rh = 0, co2 = 0;

    if (strncasecmp(rest, "HEX ", 4) == 0) {
        uint8_t b[6];
        if (!parse_hex12(rest + 4, b)) return false;
        t   = (int16_t)((b[0] << 8) | b[1]);
        rh  = (uint16_t)((b[2] << 8) | b[3]);
        co2 = (uint16_t)((b[4] << 8) | b[5]);
    } else if (strncasecmp(rest, "KV ", 3) == 0) {
        const char* p = rest + 3;
        int tt; unsigned rr, cc;
        if (sscanf(p, "T=%d RH=%u CO2=%u", &tt, &rr, &cc) != 3) return false;
        t = (int16_t)tt; rh = (uint16_t)rr; co2 = (uint16_t)cc;
    } else if (strncasecmp(rest, "BAT ", 4) == 0) {
        /* Battery frame – does not update T/RH/CO2 */
        return handle_bat_line(rest + 4);
    } else {
        return false;
    }

    /* Basic validation and update of environmental clusters */
    if (t < kTemperatureMinValue || t > kTemperatureMaxValue) return false;
    if (rh < kHumidityMinValue   || rh > kHumidityMaxValue)   return false;
    if (co2 < (uint16_t)kCO2MinPpm || co2 > (uint16_t)kCO2MaxPpm) return false;

    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        g_current_temp = t;
        g_current_humidity = rh;
        g_current_co2 = co2;
        xSemaphoreGive(g_data_mutex);
    }
    update_matter_clusters(t, rh, co2);
    return true;
}


/* ===== STATUS / COMM / VERSION commands ===== */

/* Send high-level connectivity/Matter status over UART */
static void send_status_over_uart(void)
{
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    using chip::DeviceLayer::ConnectivityMgr;
    bool attached = ConnectivityMgr().IsThreadAttached();
#else
    bool attached = false;
#endif
    if (!attached) replyf("STATUS OFFLINE");
    else           replyf("STATUS ONLINE role=SED");
}


/* ===== Factory reset work item (runs on Matter platform thread) ===== */
static void FactoryResetWork(intptr_t)
{
    chip::DeviceLayer::ConfigurationMgr().InitiateFactoryReset();
}

/* ===== FABRIC mgmt helpers ===== */
/* ==== DELETE FABRIC safely + immediately expire controller state on that fabric ==== */
[[maybe_unused]] static CHIP_ERROR DeleteFabric_Safe(chip::FabricIndex idx)
{
    struct Ctx {
        chip::FabricIndex idx;
        SemaphoreHandle_t done;
        CHIP_ERROR err;
    };
    auto *ctx = new Ctx{ idx, xSemaphoreCreateBinary(), CHIP_ERROR_INTERNAL };

    chip::DeviceLayer::PlatformMgr().ScheduleWork(
        [](intptr_t arg) {
            auto *c = reinterpret_cast<Ctx *>(arg);
            auto &table = chip::Server::GetInstance().GetFabricTable();
            c->err = table.Delete(c->idx);
            xSemaphoreGive(c->done);
        },
        reinterpret_cast<intptr_t>(ctx)
    );

    bool ok = (xSemaphoreTake(ctx->done, pdMS_TO_TICKS(3000)) == pdTRUE);
    vSemaphoreDelete(ctx->done);
    CHIP_ERROR out = ok ? ctx->err : CHIP_ERROR_TIMEOUT;

    ExpireAllFabricSessionsAndSubscriptions(idx);

    delete ctx;
    return out;
}


/* List all fabrics over UART in a compact text format */
static void ListFabrics_Safe()
{
    chip::DeviceLayer::PlatformMgr().ScheduleWork(
        [](intptr_t) {
            static char out[1024];
            size_t L = 0;
            auto append = [&](const char* fmt, ...) {
                if (L >= sizeof(out)) return;
                va_list ap; va_start(ap, fmt);
                int n = vsnprintf(out + L, sizeof(out) - L, fmt, ap);
                va_end(ap);
                if (n > 0) {
                    L += (size_t)n;
                    if (L + 2 < sizeof(out)) { out[L++] = '\r'; out[L++] = '\n'; out[L] = 0; }
                }
            };

            append("FABRICS BEGIN");

            auto &table = chip::Server::GetInstance().GetFabricTable();
            for (const auto & fabric : table) {
                append("FABRIC idx=%u cfid=%016" PRIX64 " fid=%016" PRIX64
                       " node=%016" PRIX64 " vid=%u",
                       (unsigned)fabric.GetFabricIndex(),
                       (uint64_t)fabric.GetCompressedFabricId(),
                       (uint64_t)fabric.GetFabricId(),
                       (uint64_t)fabric.GetNodeId(),
                       (unsigned)fabric.GetVendorId());
            }

            append("FABRICS END");

            uart_write_block(out, L); // single TX burst
        },
        0
    );
}

/* Work item that opens a basic commissioning window and sends QR/manual to STM32 */
static void OpenCW_Work(intptr_t)
{
    auto &mgr = chip::Server::GetInstance().GetCommissioningWindowManager();
    if (mgr.IsCommissioningWindowOpen()) {
        replyf("COMM STATE AlreadyOpen");
        send_qr_manual_now();
        return;
    }
    constexpr auto kT = chip::System::Clock::Seconds16(k_timeout_seconds);
    CHIP_ERROR err = mgr.OpenBasicCommissioningWindow(kT, chip::CommissioningWindowAdvertisement::kAllSupported);
    if (err == CHIP_NO_ERROR) {
        s_cw_open = true;
        send_qr_manual_now();
    } else {
        replyf("ERR 0x%04" PRIX32, static_cast<uint32_t>(err.AsInteger()));
    }
}

/* Work item that closes the commissioning window, if open */
static void CloseCW_Work(intptr_t)
{
    auto &mgr = chip::Server::GetInstance().GetCommissioningWindowManager();
    if (mgr.IsCommissioningWindowOpen()) {
        s_cw_close_by_app = true;
        mgr.CloseCommissioningWindow();
    }
}


/* ===== UART command parser =====
 * Handle high-level ASCII commands (PING, STATUS, COMM, FABRICS, etc.).
 */
static void handle_cmd_line(const char* line)
{
    if (strcasecmp(line, "PING") == 0) { replyf("PONG"); return; }
    if (strcasecmp(line, "STATUS?") == 0) { send_status_over_uart(); return; }

    // New: QR? / GET QR -> always send QR & MANUAL on request
    if (strcasecmp(line, "QR?") == 0 || strcasecmp(line, "GET QR") == 0) {
        send_qr_manual_now();
        return;
    }

    if (strcasecmp(line, "VERSION?") == 0) {
    #ifdef IDF_VER
        const char* idf = IDF_VER;
    #else
        const char* idf = "idf";
    #endif
        replyf("VERSION app=1.0.0 idf=%s matter=1.x", idf);
        return;
    }

    if (strcasecmp(line, "FACTORYRESET") == 0) {
        replyf("OK");
        chip::DeviceLayer::PlatformMgr().ScheduleWork(FactoryResetWork, 0);
        return;
    }

    /* === FABRIC mgmt === */
    if (strcasecmp(line, "FABRICS?") == 0) {
        ListFabrics_Safe();
        return;
    }

    if (strcasecmp(line, "COMM START") == 0 || strcasecmp(line, "COMMISSION OPEN") == 0) {
        chip::DeviceLayer::PlatformMgr().ScheduleWork(OpenCW_Work, 0);
        return;
    }

    if (strcasecmp(line, "COMM STOP") == 0 || strcasecmp(line, "COMMISSION CLOSE") == 0) {
        chip::DeviceLayer::PlatformMgr().ScheduleWork(CloseCW_Work, 0);
        return;
    }


    replyf("ERR unknown");
}

/* ===== UART receiver task =====
 * Implements the WAKE/READY handshake, reads one line from STM32, and routes it
 * either to handle_data_line() or handle_cmd_line().
 */
static void uart_rx_task(void *arg)
{
    s_uart_task_handle = xTaskGetCurrentTaskHandle();
    ready_pin_set(false);

    ESP_LOGI(TAG, "UART RX task started (WAKE=GPIO%d, READY=GPIO%d)", WAKE_PIN, kReadyPin);

#if CONFIG_PM_ENABLE
    if (s_pm_lock == nullptr) {
        esp_err_t perr = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "uart_io", &s_pm_lock);
        if (perr == ESP_OK)
            ESP_LOGI(TAG, "PM lock created (NO_LIGHT_SLEEP for UART IO)");
        else
            ESP_LOGE(TAG, "PM lock create failed: %d", perr);
    }
#endif

    char line[256];

    while (true) {
        // Wait for WAKE=1 (ISR notify)
        if (get_wake_level() == 0) {
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            if (get_wake_level() == 0)
                continue;
        }

        // Debounce WAKE=1
        if (!wait_for_wake_pin_level(1, 20000)) {
            ESP_LOGW(TAG, "WAKE glitch, skipping");
            continue;
        }

        // Flush any stale bytes
        size_t buffered_pre = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM, &buffered_pre));
        if (buffered_pre) {
            bool pm = pm_lock_acquire_if();
            uart_flush_input(UART_NUM);
            pm_lock_release_if(pm);
        }

        ESP_LOGI(TAG, "WAKE HIGH (count=%lu), READY up", (unsigned long)g_wake_count);
        ready_pin_set(true);

        // Settle ~1 ms (9600 Bd ~ 1 ms/byte)
        esp_rom_delay_us(1000);

        // === Read a single line ===
        int n = read_line(line, sizeof(line), pdMS_TO_TICKS(1000));

        if (n > 0) {
            if (strncasecmp(line, "DATA ", 5) == 0) {
                if (!handle_data_line(line + 5)) {
                    ESP_LOGW(TAG, "DATA parse failed: %s", line);
                }
                // DATA frames are one-way (no reply)
            } else {
                // Commands usually respond with one or more lines
                handle_cmd_line(line);
            }
        } else {
            ESP_LOGW(TAG, "Empty/timeout line");
        }

        // Handshake done
        ready_pin_set(false);
        ESP_LOGI(TAG, "READY down (WAKE=%d)", get_wake_level());

        if (!wait_for_wake_pin_level(0, 500000)) {   // 500 ms
            ESP_LOGW(TAG, "WAKE stuck HIGH after transaction; waiting...");
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* ===== Matter / App events =====
 * Translate Matter DeviceEvents into UART COMM state notifications and commissioning behavior.
 */
static void app_event_cb(const ChipDeviceEvent *event, intptr_t)
{
    switch (event->Type) {
        case DeviceEventType::kCommissioningWindowOpened:
            s_cw_open = true;
            replyf("COMM STATE Discover");
            send_qr_manual_now();    // send fresh QR/manual when discovery starts
            break;

        case DeviceEventType::kCommissioningSessionStarted:
            replyf("COMM STATE Connected");
            break;

        case DeviceEventType::kCommissioningWindowClosed:
        {
            bool closed_by_app = s_cw_close_by_app;
            bool was_open = s_cw_open;
            s_cw_close_by_app = false;

            if (!was_open && !closed_by_app) {
                ESP_LOGD(TAG, "Commissioning window already closed; ignoring duplicate event");
                break;
            }

            s_cw_open = false;

            if (closed_by_app) {
                replyf("COMM STATE Closed");
                replyf("COMM CLOSED");
            } else {
                // Closed "on its own" (typically timeout), but commissioning may still complete.
                replyf("COMM STATE Timeout");
                replyf("COMM TIMEOUT");
            }
            break;
        }

        case DeviceEventType::kCommissioningComplete:
        {
            replyf("COMM STATE Complete");
            replyf("COMM DONE");

        #if CONFIG_PM_ENABLE
            bool held = pm_lock_acquire_if();
            chip::DeviceLayer::SystemLayer().StartTimer(
                chip::System::Clock::Seconds16(300),
                [](chip::System::Layer*, void*){ pm_lock_release_if(true); },
                nullptr);
            (void)held;
        #endif
            break;
        }

        case DeviceEventType::kCommissioningSessionStopped:
            replyf("COMM STATE Stopped");
            replyf("COMM CLOSED");
            break;

        case DeviceEventType::kFabricRemoved:
            ESP_LOGI(TAG, "Fabric removed");
            if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
                // If all fabrics have been removed, open a commissioning window again
                auto &mgr = chip::Server::GetInstance().GetCommissioningWindowManager();
                constexpr auto kT = chip::System::Clock::Seconds16(k_timeout_seconds);
                if (!mgr.IsCommissioningWindowOpen()) {
                    if (mgr.OpenBasicCommissioningWindow(kT, chip::CommissioningWindowAdvertisement::kAllSupported) != CHIP_NO_ERROR) {
                        ESP_LOGE(TAG, "Failed to open commissioning window");
                    }
                }
            }
            break;

        case DeviceEventType::kThreadStateChange:
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
        {
            using chip::DeviceLayer::ConnectivityMgr;
            bool attached = ConnectivityMgr().IsThreadAttached();
            ESP_LOGI(TAG, "Thread role change -> %s", attached ? "ATTACHED" : "DETACHED");
        }
#endif
            break;

        default:
            break;
    }
}

static esp_err_t app_identification_cb(identification::callback_type_t, uint16_t, uint8_t, uint8_t, void *)
{
    return ESP_OK;
}

static esp_err_t app_attribute_update_cb(attribute::callback_type_t, uint16_t, uint32_t, uint32_t, esp_matter_attr_val_t *, void *)
{
    return ESP_OK;
}

/* ===== Main entry point ===== */
extern "C" void app_main()
{
    esp_err_t err = ESP_OK;
    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("uart", ESP_LOG_INFO);

    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "Matter Multi-Sensor with UART - TEXT PROTOCOL");
    ESP_LOGI(TAG, "==============================================");

    /* NVS */
    nvs_flash_init();
    InitNodeBasicInformation();
    g_data_mutex = xSemaphoreCreateMutex();
    if (s_reply_mutex == nullptr) {
        s_reply_mutex = xSemaphoreCreateRecursiveMutex();
    }

#if CONFIG_PM_ENABLE
    /* Configure PM once at boot (SED -> light sleep ON) */
    esp_pm_config_t pm_cfg = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true,
#else
        .light_sleep_enable = false,
#endif
    };
    err = esp_pm_configure(&pm_cfg);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "PM configured (light sleep %s)", pm_cfg.light_sleep_enable ? "ENABLED" : "DISABLED");
    } else {
        ESP_LOGW(TAG, "esp_pm_configure failed (%d)", err);
    }
#endif

    /* UART & GPIO BEFORE Matter */
    uart_init();
    wake_pin_init();
    ready_pin_init();

#ifdef CONFIG_ENABLE_USER_ACTIVE_MODE_TRIGGER_BUTTON
    app_driver_button_init();
#endif

    /* ===== Matter endpoints ===== */
    node::config_t node_cfg;
    node_t *node = node::create(&node_cfg, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    {
        /* Temperature sensor endpoint */
        temperature_sensor::config_t cfg;
        cfg.temperature_measurement.measured_value = static_cast<int16_t>(0);        // 0 = 0.00 °C (format 0.01 °C)
        cfg.temperature_measurement.min_measured_value = kTemperatureMinValue;
        cfg.temperature_measurement.max_measured_value = kTemperatureMaxValue;
        endpoint_t *ep = temperature_sensor::create(node, &cfg, ENDPOINT_FLAG_NONE, nullptr);
        ABORT_APP_ON_FAILURE(ep != nullptr, ESP_LOGE(TAG, "Failed to create temperature sensor"));
        g_temp_ep_id = esp_matter::endpoint::get_id(ep);
        ESP_LOGI(TAG, "Temperature sensor: endpoint_id=%u", g_temp_ep_id);
    }

    {
        /* Humidity sensor endpoint */
        humidity_sensor::config_t cfg;
        cfg.relative_humidity_measurement.measured_value = static_cast<uint16_t>(0); // 0 = 0.00 %
        cfg.relative_humidity_measurement.min_measured_value = kHumidityMinValue;
        cfg.relative_humidity_measurement.max_measured_value = kHumidityMaxValue;
        endpoint_t *ep = humidity_sensor::create(node, &cfg, ENDPOINT_FLAG_NONE, nullptr);
        ABORT_APP_ON_FAILURE(ep != nullptr, ESP_LOGE(TAG, "Failed to create humidity sensor"));
        g_humid_ep_id = esp_matter::endpoint::get_id(ep);
        ESP_LOGI(TAG, "Humidity sensor: endpoint_id=%u", g_humid_ep_id);
    }

    {
        /* Air Quality + CO2 (Concentration Measurement – MEA) endpoint */
        endpoint::app_base_config ep_cfg{};
        ep_cfg.identify.identify_type =
            static_cast<uint8_t>(chip::app::Clusters::Identify::IdentifyTypeEnum::kVisibleIndicator);

        endpoint_t *ep = endpoint::create(node, ENDPOINT_FLAG_NONE, nullptr);
        ABORT_APP_ON_FAILURE(ep != nullptr, ESP_LOGE(TAG, "Failed to create air quality endpoint"));
        g_co2_ep_id = esp_matter::endpoint::get_id(ep);

        ABORT_APP_ON_FAILURE(cluster::descriptor::create(ep, &(ep_cfg.descriptor), CLUSTER_FLAG_SERVER) != nullptr,
                             ESP_LOGE(TAG, "Failed to create descriptor cluster"));
        ABORT_APP_ON_FAILURE(cluster::identify::create(ep, &(ep_cfg.identify), CLUSTER_FLAG_SERVER) != nullptr,
                             ESP_LOGE(TAG, "Failed to create identify cluster"));

        esp_err_t err = endpoint::add_device_type(
            ep,
            ESP_MATTER_AIR_QUALITY_SENSOR_DEVICE_TYPE_ID,
            ESP_MATTER_AIR_QUALITY_SENSOR_DEVICE_TYPE_VERSION
        );
        ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to tag air quality device type, err:%d", err));

        /* ---- AirQuality (0x005B) ----
           Safe choice is to keep FeatureMap = 0x0 (no extra promises) and only expose AirQuality attribute.
         */
        {
            using AQ = chip::app::Clusters::AirQuality::AirQualityEnum;

            cluster_t *aq_cl = cluster::create(ep, chip::app::Clusters::AirQuality::Id, CLUSTER_FLAG_SERVER);
            ABORT_APP_ON_FAILURE(aq_cl != nullptr, ESP_LOGE(TAG, "Failed to create AirQuality cluster"));

            cluster::global::attribute::create_feature_map(aq_cl, 0x00000000);
            cluster::global::attribute::create_cluster_revision(aq_cl, kAirQualityClusterRevision);

            attribute::create(
                aq_cl,
                chip::app::Clusters::AirQuality::Attributes::AirQuality::Id,
                ATTRIBUTE_FLAG_NONE,
                esp_matter_enum8(static_cast<uint8_t>(AQ::kUnknown))
            );
        }

        /* ---- CO2 Concentration Measurement (0x040D) ----
           Only MEA feature is used. A mandatory Uncertainty (0x0007) attribute is added.
           AVG feature is not enabled (no averaging implementation).
         */
        {
            cluster_t *co2 = cluster::create(ep, kClusterId_CO2, CLUSTER_FLAG_SERVER);
            ABORT_APP_ON_FAILURE(co2 != nullptr, ESP_LOGE(TAG, "Failed to create CO2 cluster"));

            cluster::global::attribute::create_feature_map(co2, 0x00000001); // MEA
            cluster::global::attribute::create_cluster_revision(co2, 3);

            // Mandatory for MEA (Measured/Min/Max)
            attribute::create(co2, kAttrId_CO2_MeasuredValue, ATTRIBUTE_FLAG_NONE,
                               esp_matter_nullable_float(0.0f));
            attribute::create(co2, kAttrId_CO2_MinMeasured, ATTRIBUTE_FLAG_NONE,
                               esp_matter_nullable_float(0.0f));
            attribute::create(co2, kAttrId_CO2_MaxMeasured, ATTRIBUTE_FLAG_NONE,
                               esp_matter_nullable_float(0.0f));
            attribute::create(co2, kAttrId_CO2_Uncertainty, ATTRIBUTE_FLAG_NONE,
                               esp_matter_nullable_float(0.0f));

            attribute::create(co2, kAttrId_CO2_MeasurementUnit,   ATTRIBUTE_FLAG_NONE, esp_matter_enum8(0)); // 0=PPM
            attribute::create(co2, kAttrId_CO2_MeasurementMedium, ATTRIBUTE_FLAG_NONE, esp_matter_enum8(0)); // 0=Air
        }

        ESP_LOGI(TAG, "CO2/Air Quality sensor: endpoint_id=%u", g_co2_ep_id);
    }

    /* Power Source endpoints for USB + Li-Ion + LiSOCl2 */
    init_power_source_endpoints(node);



#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t ot_cfg = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config  = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&ot_cfg);
#endif

    /* UART task BEFORE Matter start (so we can catch the very first WAKE after boot) */
    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", 4096, nullptr, tskIDLE_PRIORITY + 2, nullptr, 0);

    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "System ready - waiting for STM32 data/commands");
    ESP_LOGI(TAG, "UART%d: RX=GPIO%d, TX=GPIO%d @ %d baud", UART_NUM, UART_RX_PIN, UART_TX_PIN, UART_BAUD_RATE);
    ESP_LOGI(TAG, "Wake: GPIO%d (trigger on HIGH)", WAKE_PIN);
    ESP_LOGI(TAG, "==============================================");

    /* Start Matter - LAST step */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "Matter started successfully - device is ready");
    ESP_LOGI(TAG, "==============================================");
}
