#include "ee_config.h"
#include <string.h>

#define APP_CFG_MAGIC   0x43464731u   /* 'CFG1' */
#define APP_CFG_VERSION 1u
#define APP_CFG_EE_ADDR 0x00u         /* začátek EEPROM (0xC0 ponecháme pro self-test) */

/* Struktura uložená v EEPROM (cca 188 B včetně hlavičky). */
typedef struct __attribute__((packed)) {
    uint32_t        magic;
    uint8_t         version;
    uint8_t         reserved[3];
    uint16_t        length;
    uint16_t        crc;

    AppConfig_t     cfg;
    WioOtaaConfig_t wio;
    EspC6Pairing_t  esp;  /* QR + MANUAL */
} EepromAppCfg_t;

/* Jednoduchý CRC – součet všech bajtů */
static uint16_t EE_Config_CalcCrc(const AppConfig_t     *cfg,
                                  const WioOtaaConfig_t *wio,
                                  const EspC6Pairing_t  *esp)
{
    uint16_t crc = 0;
    const uint8_t *p;
    size_t len;

    if (cfg) {
        p   = (const uint8_t*)cfg;
        len = sizeof(*cfg);
        for (size_t i = 0; i < len; ++i) crc = (uint16_t)(crc + p[i]);
    }

    if (wio) {
        p   = (const uint8_t*)wio;
        len = sizeof(*wio);
        for (size_t i = 0; i < len; ++i) crc = (uint16_t)(crc + p[i]);
    }

    if (esp) {
        p   = (const uint8_t*)esp;
        len = sizeof(*esp);
        for (size_t i = 0; i < len; ++i) crc = (uint16_t)(crc + p[i]);
    }

    return crc;
}

HAL_StatusTypeDef EE_Config_Load(I2C_HandleTypeDef *hi2c,
                                 AppConfig_t        *cfg,
                                 WioOtaaConfig_t    *wio,
                                 EspC6Pairing_t     *esp_pairing)
{
    if (!hi2c || !cfg || !wio || !esp_pairing) return HAL_ERROR;

    M24C02_HandleTypeDef hee;
    M24C02_Init(&hee, hi2c, 100);

    EepromAppCfg_t img;
    HAL_StatusTypeDef st = M24C02_Read(&hee, APP_CFG_EE_ADDR,
                                       (uint8_t*)&img, sizeof(img));
    if (st != HAL_OK) {
        /* EEPROM nedostupná -> defaulty */
        AppConfig_SetDefaults(cfg);
        memset(wio, 0, sizeof(*wio));
        memset(esp_pairing, 0, sizeof(*esp_pairing));
        return st;
    }

    /* Validace hlavičky */
    if (img.magic != APP_CFG_MAGIC || img.version != APP_CFG_VERSION) {
        AppConfig_SetDefaults(cfg);
        memset(wio, 0, sizeof(*wio));
        memset(esp_pairing, 0, sizeof(*esp_pairing));
        return HAL_ERROR;
    }

    uint16_t expected_len =
        (uint16_t)(sizeof(AppConfig_t) +
                   sizeof(WioOtaaConfig_t) +
                   sizeof(EspC6Pairing_t));
    if (img.length != expected_len) {
        AppConfig_SetDefaults(cfg);
        memset(wio, 0, sizeof(*wio));
        memset(esp_pairing, 0, sizeof(*esp_pairing));
        return HAL_ERROR;
    }

    uint16_t crc = EE_Config_CalcCrc(&img.cfg, &img.wio, &img.esp);
    if (crc != img.crc) {
        AppConfig_SetDefaults(cfg);
        memset(wio, 0, sizeof(*wio));
        memset(esp_pairing, 0, sizeof(*esp_pairing));
        return HAL_ERROR;
    }

    /* OK -> přenos do runtime struktur */
    *cfg         = img.cfg;
    *wio         = img.wio;
    *esp_pairing = img.esp;

    return HAL_OK;
}

HAL_StatusTypeDef EE_Config_Save(I2C_HandleTypeDef      *hi2c,
                                 const AppConfig_t      *cfg,
                                 const WioOtaaConfig_t  *wio,
                                 const EspC6Pairing_t   *esp_pairing)
{
    if (!hi2c || !cfg || !wio || !esp_pairing) return HAL_ERROR;

    M24C02_HandleTypeDef hee;
    M24C02_Init(&hee, hi2c, 100);

    EepromAppCfg_t img;
    memset(&img, 0, sizeof(img));

    img.magic   = APP_CFG_MAGIC;
    img.version = APP_CFG_VERSION;
    img.length  = (uint16_t)(sizeof(AppConfig_t) +
                             sizeof(WioOtaaConfig_t) +
                             sizeof(EspC6Pairing_t));

    img.cfg = *cfg;
    img.wio = *wio;
    img.esp = *esp_pairing;

    img.crc  = EE_Config_CalcCrc(&img.cfg, &img.wio, &img.esp);

    return M24C02_Write(&hee, APP_CFG_EE_ADDR,
                        (const uint8_t*)&img, sizeof(img));
}
