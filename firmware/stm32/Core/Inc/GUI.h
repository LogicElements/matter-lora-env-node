#pragma once
/**
 * @file GUI.h
 * @author Evzen Steif
 * @brief Public API for E-Ink UI (full screen, partial updates, config overlays, QR).
 *
 * @details
 *   This header exposes a small API for drawing on the 2.13" e-paper panel:
 *     - Initialization and full dashboard rendering.
 *     - Partial updates (state line, VOC line).
 *     - Configuration-mode visuals (icon + status strip).
 *     - CO2 calibration progress.
 *     - QR code rendering for Matter commissioning and similar use cases.
 */

#include <stdint.h>
#include "app.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Main screens */
void GUI_Init(void);                         /**< Initialize e-ink, canvas and clear to white. */
void GUI_Display(void);                      /**< Full redraw of the main metrics/dashboard. */
void GUI_UpdateState(SystemState_t state);   /**< Partial update of the bottom state line. */
void GUI_UpdateVOCIndex(uint16_t voc_index); /**< Partial update of the VOC row with latest index. */

/* Calibration */
void GUI_DrawCalibrationProgressFromStruct(void); /**< Partial progress bar for CO2 calibration. */

/* Low-level helpers used from app */
void GUI_OnEPDResumed(void);                 /**< Notify GUI that the controller was re-initialized (reset partial base). */
void EPDClear(void);                         /**< Clear canvas buffer to white (no display update). */
void GUI_DrawLoraIcon(uint8_t connected);    /**< Draw small LoRa connectivity icon (OK/No) in header. */

/* Config mode visuals */
void GUI_DrawConfIcon(void);                 /**< Full draw of config icon + label and set partial base. */
void GUI_ConfClearStatus(void);              /**< Clear narrow status strip (partial refresh). */
void GUI_ConfShowStatus(const char* msg);    /**< Show a single-line status (OK/ERR…) in the strip (partial). */
void GUI_ConfShowSetResult(const char* key,
                           const char* value,
                           uint8_t ok);      /**< Show "SET key=value : OK/ERR" in the status strip. */

/* QR rendering */
uint8_t GUI_DrawQR_TextFull(const char* text,
                            const char* footer); /**< Generate and display QR + optional footer (full refresh). */

#ifdef __cplusplus
}
#endif
