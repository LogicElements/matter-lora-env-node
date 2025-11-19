/**
 * @file GUI.c
 * @author Evzen Steif
 * @brief E-Ink GUI rendering: main dashboard, partial updates, config overlays and QR.
 *
 * @details
 *   This module encapsulates all drawing logic for the 2.13" e-paper display:
 *     - Full main dashboard (CO2/T/RH/VOC, batteries, USB, charger, state).
 *     - Partial updates of the state line and VOC line to minimize ghosting.
 *     - Configuration-mode overlays (configuration icon, status strip).
 *     - CO2 calibration progress view.
 *     - QR code rendering for Matter commissioning, etc.
 *
 *   All public functions are documented with a short English description.
 */

#include "GUI.h"
#include "EPD_2in13_V4.h"
#include "GUI_Paint.h"
#include "fonts.h"
#include "DEV_Config.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "WioE5.h"
#include "SDC41.h"
#include "SHT41.h"
#include "RTC.h"
#include "images.h"

#include "qrcodegen.h"

extern AppContext_t app;   /* from app.c */

static UBYTE *BlackImage = NULL;

static uint8_t s_partial_base_valid = 0;

/**
 * @brief Ensure the e-paper controller has an "old image" for partial refreshes.
 */
static inline void EnsurePartialBase(void) {
    if (!s_partial_base_valid) {
        EPD_2in13_V4_LoadBase_RAMOnly(BlackImage);
        s_partial_base_valid = 1;
    }
}

/**
 * @brief Called after EPD resumed from deep sleep: forces re-establish of partial base.
 */
void GUI_OnEPDResumed(void) { s_partial_base_valid = 0; }

/* -------------------- Helpers -------------------- */
static inline UWORD TextWidth(const sFONT* font, const char* s) {
    return (UWORD)(font->Width * strlen(s));
}
static inline UWORD TextHeight(const sFONT* font) {
    return (UWORD)font->Height;
}
static void DrawRightAligned(UWORD rightX, UWORD y, const char* s, const sFONT* font, UWORD bg, UWORD fg) {
    UWORD w = TextWidth(font, s);
    Paint_DrawString_EN(rightX - w, y, (char*)s, (sFONT*)font, bg, fg);
}
static void FormatInterval(char* out, size_t n, uint32_t secs) {
    uint32_t h = secs / 3600U;
    uint32_t m = (secs % 3600U) / 60U;
    uint32_t s = secs % 60U;
    if (h > 0)      snprintf(out, n, "%luh %lum", (unsigned long)h, (unsigned long)m);
    else if (m > 0) snprintf(out, n, "%lum %lus", (unsigned long)m, (unsigned long)s);
    else            snprintf(out, n, "%lus", (unsigned long)s);
}

/* ---- Tiny trend arrows (driver-safe: lines only, 9x9 box) ---- */
typedef enum { TREND_DOWN=-1, TREND_FLAT=0, TREND_UP=1 } Trend_t;

static void DrawTrendArrow(UWORD x, UWORD y, Trend_t t) {
    UWORD xmid = x + 4;
    UWORD ytop = y + 1;
    UWORD ybot = y + 7;

    switch (t) {
        case TREND_UP:
            Paint_DrawLine(xmid, ybot, xmid, ytop+1, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
            Paint_DrawLine(xmid, ytop+1, xmid-3, ytop+4, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
            Paint_DrawLine(xmid, ytop+1, xmid+3, ytop+4, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
            break;
        case TREND_DOWN:
            Paint_DrawLine(xmid, ytop, xmid, ybot-1, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
            Paint_DrawLine(xmid, ybot-1, xmid-3, ybot-4, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
            Paint_DrawLine(xmid, ybot-1, xmid+3, ybot-4, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
            break;
        default:
            Paint_DrawLine(x+1, y+4, x+7, y+4, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
            Paint_DrawLine(x+1, y+3, x+1, y+5, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
            Paint_DrawLine(x+7, y+3, x+7, y+5, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
            break;
    }
}

/* -------------------- Init -------------------- */
/**
 * @brief Initialize drawing canvas and clear the screen (full refresh).
 *
 * Allocates the BlackImage buffer, sets orientation and clears to white. Also
 * resets the partial-refresh base so the next GUI_Display establishes it.
 */
void GUI_Init(void)
{
    DEV_Module_Init();
    EPD_2in13_V4_Init();
    EPD_2in13_V4_Clear();

    UWORD ImageSize = ((EPD_2in13_V4_WIDTH % 8 == 0) ?
                       (EPD_2in13_V4_WIDTH / 8) :
                       (EPD_2in13_V4_WIDTH / 8 + 1)) * EPD_2in13_V4_HEIGHT;

    /* Avoid leaking the buffer if GUI_Init() is called repeatedly. */
    if (BlackImage) {
        free(BlackImage);
        BlackImage = NULL;
    }

    BlackImage = (UBYTE *)malloc(ImageSize);
    if (BlackImage == NULL) {
        printf("Failed to allocate memory for e-Paper buffer.\r\n");
        return;
    }

    Paint_NewImage(BlackImage, EPD_2in13_V4_WIDTH, EPD_2in13_V4_HEIGHT, 270, WHITE);
    Paint_SelectImage(BlackImage);
    Paint_Clear(WHITE);

    s_partial_base_valid = 0;
}

/* -------------------- Main screen -------------------- */
/**
 * @brief Draw full main screen (full refresh).
 *
 * Renders the complete dashboard: header with connectivity/time, main grid with
 * Temp/RH/CO2/VOC (including trends), emoticon + CO2 gauge on the right, and
 * bottom strip with battery/USB/charger/sleep information. Establishes a new
 * partial-refresh base image at the end.
 */
void GUI_Display(void)
{
    const UWORD CW = 250;
    const UWORD CH = 122;

    const UWORD MARGIN_X   = 6;
    const UWORD HEADER_Y   = 4;
    const UWORD GRID_Y0    = 28;
    const UWORD LINE_STEP  = 16;

    const UWORD LABEL_X      = MARGIN_X + 2;
    const UWORD VALUE_RIGHT  = 155;
    const UWORD ARROW_X      = VALUE_RIGHT + 8;
    const UWORD DIVIDER_X    = VALUE_RIGHT + 20;

    const UWORD EMO_W = 50, EMO_H = 50;
    const UWORD EMO_X = CW - EMO_W - 18;
    const UWORD EMO_Y = GRID_Y0 - 5;

    const UWORD GAUGE_W = 60;
    const UWORD GAUGE_H = 6;
    const UWORD GAUGE_X = EMO_X + (EMO_W - GAUGE_W)/2;
    const UWORD GAUGE_Y = EMO_Y + EMO_H + 2;

    const UWORD STRIP_TOP = 83;
    const UWORD STRIP_H   = 26;
    const UWORD STRIP_IN  = 3;

    static int   firstRun = 1;
    static float lastTemp = 0.0f;
    static float lastRH   = 0.0f;
    static int   lastCO2  = 0;
    static int   lastVOC  = 0;

    char buf[48];

    Paint_SelectImage(BlackImage);
    Paint_Clear(WHITE);

    /* Header */
    if (app.Connected)
        Paint_DrawBitMap_Paste(gImage_CommOK, 10, 0, 20, 20, 0);
    else
        Paint_DrawBitMap_Paste(gImage_CommNo, 10, 0, 20, 20, 0);

    UWORD left_limit  = 10 + 20 + 4;

    char day_time[25];

    if(app.time_sync_ok)
    {
    snprintf(day_time, sizeof(day_time),
             "%02u.%02u %02u:%02u:%02u",
             g_timeData.day, g_timeData.month,
             g_timeData.hour, g_timeData.minute, g_timeData.second);
    DrawRightAligned(CW - MARGIN_X, HEADER_Y + 2, day_time, &Font12, WHITE, BLACK);
    }

    UWORD time_w = TextWidth(&Font12, day_time);
    UWORD right_limit = 250 - 6 - time_w - 4;

    const char *title = "LocuSense";
    UWORD title_w = TextWidth(&Font16, title);
    UWORD span_w = (right_limit > left_limit) ? (right_limit - left_limit) : 0;
    UWORD title_x = left_limit + (span_w > title_w ? (span_w - title_w)/2 : 0);

    Paint_DrawString_EN(title_x, HEADER_Y, (char*)title, &Font16, WHITE, BLACK);

    Paint_DrawLine(MARGIN_X, HEADER_Y + 18, CW - MARGIN_X, HEADER_Y + 18,
                   BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

    /* Trends */
    Trend_t tTemp = TREND_FLAT, tRH = TREND_FLAT, tCO2 = TREND_FLAT, tVOC = TREND_FLAT;
    if (!firstRun) {
        float dT = SHT_data.temperature - lastTemp;
        float dH = SHT_data.rh - lastRH;
        int   dC = (int)SCD_data.co2 - lastCO2;
        int   dV = (int)app.voc_last_value - lastVOC;

        tTemp = (dT >  0.1f) ? TREND_UP   : (dT < -0.1f ? TREND_DOWN : TREND_FLAT);
        tRH   = (dH >  0.5f) ? TREND_UP   : (dH < -0.5f ? TREND_DOWN : TREND_FLAT);
        tCO2  = (dC >   20  ) ? TREND_UP  : (dC < -20   ? TREND_DOWN : TREND_FLAT);

        if (lastVOC == 0) tVOC = TREND_FLAT;
        else tVOC = (dV >  10) ? TREND_UP : (dV < -10 ? TREND_DOWN : TREND_FLAT);
    }

    lastTemp = SHT_data.temperature;
    lastRH   = SHT_data.rh;
    lastCO2  = (int)SCD_data.co2;
    lastVOC  = (int)app.voc_last_value;
    firstRun = 0;

    /* Left column */
    UWORD y = GRID_Y0 - 4;

    /* Temp */
    Paint_DrawString_EN(LABEL_X, y, "Temp:", &Font16, WHITE, BLACK);
    snprintf(buf, sizeof(buf), "%.1f C", SHT_data.temperature);
    DrawRightAligned(VALUE_RIGHT, y, buf, &Font16, WHITE, BLACK);
    char* pc = strrchr(buf, 'C');
    if (pc) {
        UWORD w = TextWidth(&Font16, buf);
        UWORD idx = (UWORD)(pc - buf);
        UWORD cx = VALUE_RIGHT - w + idx * Font16.Width - 4;
        UWORD cy = y + 5;
        Paint_DrawCircle(cx, cy, 2, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    }
    DrawTrendArrow(ARROW_X, y+2, tTemp);

    /* RH */
    y += LINE_STEP;
    Paint_DrawString_EN(LABEL_X, y, "RH:", &Font16, WHITE, BLACK);
    snprintf(buf, sizeof(buf), "%.1f %%", SHT_data.rh);
    DrawRightAligned(VALUE_RIGHT, y, buf, &Font16, WHITE, BLACK);
    DrawTrendArrow(ARROW_X, y+2, tRH);

    /* CO2 */
    y += LINE_STEP;
    if (app.cfg.measureMode == MEAS_TRH_ONLY) {
        /* TRH-only -> clear entire CO2 row (label + value + arrow). */
        const UWORD clear_left  = LABEL_X;
        const UWORD clear_right = DIVIDER_X - 2;
        const UWORD clear_top   = y - 1;
        const UWORD clear_bot   = y + TextHeight(&Font16) - 2;
        Paint_ClearWindows(clear_left, clear_top, clear_right, clear_bot, WHITE);
    } else {
        Paint_DrawString_EN(LABEL_X, y, "CO2:", &Font16, WHITE, BLACK);
        snprintf(buf, sizeof(buf), "%u ppm", SCD_data.co2);
        DrawRightAligned(VALUE_RIGHT, y, buf, &Font16, WHITE, BLACK);
        DrawTrendArrow(ARROW_X, y+2, tCO2);
    }

    /* VOC */
    y += LINE_STEP;
    if (app.cfg.vocMode == VOC_DISABLED || !app.usb_on) {
        /* VOC disabled OR USB OFF -> clear entire VOC row (label and content). */
        const UWORD clear_left  = LABEL_X;
        const UWORD clear_right = DIVIDER_X - 2;
        const UWORD clear_top   = y - 1;
        const UWORD clear_bot   = y + TextHeight(&Font12) - 2;
        Paint_ClearWindows(clear_left, clear_top, clear_right, clear_bot, WHITE);
    } else {
        /* USB ON + VOC enabled -> draw VOC row. */
        Paint_DrawString_EN(LABEL_X, y, "VOC:", &Font12, WHITE, BLACK);
        if (app.voc_last_value == 0) {
            const char *msg = "not measured";
            DrawRightAligned(VALUE_RIGHT, y, msg, &Font12, WHITE, BLACK);
            DrawTrendArrow(ARROW_X, y, TREND_FLAT);
        } else {
            snprintf(buf, sizeof(buf), "%u idx", (unsigned)app.voc_last_value);
            UWORD voc_val_w = TextWidth(&Font12, buf);
            UWORD voc_val_x = (VALUE_RIGHT - 40) - voc_val_w;
            Paint_DrawString_EN(voc_val_x, y, buf, &Font12, WHITE, BLACK);

            DrawTrendArrow(ARROW_X, y, tVOC);


            if(app.time_sync_ok)
            {
            char tbuf[20];
            snprintf(tbuf, sizeof(tbuf), "%02u:%02u",
                     app.voc_last_time.hour,
                     app.voc_last_time.minute);

            UWORD ts_x = voc_val_x + voc_val_w + 6;
            Paint_DrawString_EN(ts_x, y, tbuf, &Font12, WHITE, BLACK);
            }
        }
    }

    /* Divider line between columns */
    Paint_DrawLine(DIVIDER_X, GRID_Y0 - 4, DIVIDER_X, STRIP_TOP - 4,
                   BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

    /* Right panel emoji */
    const unsigned char *emo = NULL;

    if (app.co2_data < 800) {
        if (SHT_data.temperature < 15.0f)      emo = epd_bitmap_happy_cold;
        else if (SHT_data.temperature > 35.0f) emo = epd_bitmap_happy_hot;
        else                                   emo = epd_bitmap_happy_basic;
    }
    else if (app.co2_data < 1200) {
        if (SHT_data.temperature < 15.0f)      emo = epd_bitmap_neutral_cold;
        else if (SHT_data.temperature > 35.0f) emo = epd_bitmap_neutral_hot;
        else                                   emo = epd_bitmap_neutral_basic;
    }
    else if (app.co2_data < 2500) {
        if (SHT_data.temperature < 15.0f)      emo = epd_bitmap_sad_cold;
        else if (SHT_data.temperature > 35.0f) emo = epd_bitmap_sad_hot;
        else                                   emo = epd_bitmap_sad_basic;
    }
    else {
        if (SHT_data.temperature < 15.0f)      emo = epd_bitmap_dead_cold;
        else if (SHT_data.temperature > 35.0f) emo = epd_bitmap_dead_hot;
        else                                   emo = epd_bitmap_dead_basic;
    }

    if (emo) {
        Paint_DrawBitMap_Paste(emo, EMO_X, EMO_Y, EMO_W, EMO_H, 0);
    }

    /* CO2 mini gauge */
    Paint_DrawRectangle(GAUGE_X, GAUGE_Y, GAUGE_X + GAUGE_W, GAUGE_Y + GAUGE_H,
                        BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);

    int co2 = (int)app.co2_data;
    if (co2 < 400)  co2 = 400;
    if (co2 > 2000) co2 = 2000;
    UWORD fill_w = (UWORD)((long)(co2 - 400) * GAUGE_W / (2000 - 400));
    if (fill_w > 0) {
        Paint_DrawRectangle(GAUGE_X, GAUGE_Y, GAUGE_X + fill_w, GAUGE_Y + GAUGE_H,
                            BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    }
    UWORD t800  = GAUGE_X + (UWORD)((long)(800  - 400) * GAUGE_W / (2000 - 400));
    UWORD t1200 = GAUGE_X + (UWORD)((long)(1200 - 400) * GAUGE_W / (2000 - 400));
    Paint_DrawLine(t800,  GAUGE_Y - 2, t800,  GAUGE_Y + GAUGE_H + 2, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(t1200, GAUGE_Y - 2, t1200, GAUGE_Y + GAUGE_H + 2, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

    /* Bottom strip */
    Paint_DrawRectangle(MARGIN_X, STRIP_TOP, CW - MARGIN_X, STRIP_TOP + STRIP_H,
                        BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);

    UWORD innerL = MARGIN_X + STRIP_IN;
    UWORD innerR = CW - MARGIN_X - STRIP_IN;
    UWORD innerW = innerR - innerL;
    UWORD colW   = innerW / 3;

    UWORD c1x = innerL;
    UWORD c2x = innerL + colW;
    UWORD c3x = innerL + 2 * colW;

    if (app.vbat_lsocl2_mV < 1000) snprintf(buf, sizeof(buf), "SOCl2: OFF");
    else snprintf(buf, sizeof(buf), "SOCl2: %.2f V", app.vbat_lsocl2_mV / 1000.0f);
    Paint_DrawString_EN(c1x, STRIP_TOP + 5, buf, &Font8, WHITE, BLACK);

    if (app.vbat_liion_mV < 1000) snprintf(buf, sizeof(buf), "LiIon: OFF");
    else snprintf(buf, sizeof(buf), "LiIon: %.2f V", app.vbat_liion_mV / 1000.0f);
    Paint_DrawString_EN(c1x, STRIP_TOP + 5 + TextHeight(&Font8), buf, &Font8, WHITE, BLACK);

    snprintf(buf, sizeof(buf), "USB: %s", app.usb_on ? "ON" : "OFF");
    Paint_DrawString_EN(c2x, STRIP_TOP + 5, buf, &Font8, WHITE, BLACK);

    if (app.usb_on) {
        Paint_DrawString_EN(c2x, STRIP_TOP + 5 + TextHeight(&Font8),
                            "LiIon state:", &Font8, WHITE, BLACK);
        const char *liion_state = BQ25185_StatusToString(app.charger_status);
        Paint_DrawString_EN(c3x,
                            STRIP_TOP + 5 + TextHeight(&Font8),
                            liion_state, &Font8, WHITE, BLACK);
    } else {
        Paint_ClearWindows(c2x,
                           STRIP_TOP + 5 + TextHeight(&Font8),
                           CW - MARGIN_X - 1,
                           STRIP_TOP + 5 + 2*TextHeight(&Font8),
                           WHITE);
    }

    char iv[24];
    FormatInterval(iv, sizeof(iv), app.cfg.interval_sleep_sec);
    snprintf(buf, sizeof(buf), "Sleep: %s", iv);
    Paint_DrawString_EN(c3x, STRIP_TOP + 5, buf, &Font8, WHITE, BLACK);

    EPD_2in13_V4_Display(BlackImage);

    EPD_2in13_V4_LoadBase_RAMOnly(BlackImage);
    s_partial_base_valid = 1;
}

/* -------------------- State line (partial refresh) -------------------- */
/**
 * @brief Update the bottom state line (partial refresh), leaves rest intact.
 *
 * Draws textual representation of the current SystemState_t near the bottom
 * edge using a partial refresh region.
 */
void GUI_UpdateState(SystemState_t state)
{
    Paint_SelectImage(BlackImage);

    const char *stateStr = NULL;
    switch (state) {
        case STATE_INIT:           stateStr = "STATE: INIT";           break;
        case STATE_MEASURE:        stateStr = "STATE: MEASURE";        break;
        case STATE_BAT_MEASURE:    stateStr = "STATE: BAT_MEASURE";    break;
        case STATE_SEND_DATA:      stateStr = "STATE: DATA_SEND";      break;
        case STATE_RECOVER:   	   stateStr = "STATE: RECOVER";   	   break;
        case STATE_TIME_REQ:       stateStr = "STATE: TIME_REQUEST";   break;
        case STATE_GUI:            stateStr = "STATE: GUI";            break;
        case STATE_SLEEP:          stateStr = "STATE: SLEEP";          break;
        case STATE_RECALIBRATION:  stateStr = "STATE: CO2_RECALIB";    break;
        case STATE_CONFIG:         stateStr = "STATE: CONFIG";         break;
        default:                   stateStr = "STATE: DEFAULT";        break;
    }

    const UWORD STATE_Y  = 110;
    const UWORD STATE_X0 = 6;
    const UWORD STATE_X1 = 244;

    UWORD h = TextHeight(&Font12);

    EnsurePartialBase();

    Paint_ClearWindows(STATE_X0, STATE_Y, STATE_X1, STATE_Y + h, WHITE);
    Paint_DrawString_EN(STATE_X0, STATE_Y, (char*)stateStr, &Font12, WHITE, BLACK);

    EPD_2in13_V4_Display_Partial(BlackImage);
}

/* -------------------- VOC line partial refresh -------------------- */
/**
 * @brief Update just the VOC line with latest index (partial refresh).
 *
 * Re-draws the VOC value, trend arrow and VOC timestamp (if time is synced)
 * without touching the rest of the screen.
 */
void GUI_UpdateVOCIndex(uint16_t voc_index)
{
    const UWORD CW = 250;
    const UWORD MARGIN_X   = 6;
    const UWORD GRID_Y0    = 28;
    const UWORD LINE_STEP  = 16;

    const UWORD LABEL_X      = MARGIN_X + 2;
    const UWORD VALUE_RIGHT  = 155;
    const UWORD ARROW_X      = VALUE_RIGHT + 8;
    const UWORD DIVIDER_X    = VALUE_RIGHT + 20;

    UWORD y = (GRID_Y0 - 4) + (3 * LINE_STEP);

    static int lastVOC_partial = 0;
    Trend_t tVOC = TREND_FLAT;
    if (lastVOC_partial != 0) {
        int dV = (int)voc_index - lastVOC_partial;
        tVOC = (dV > 10) ? TREND_UP : (dV < -10 ? TREND_DOWN : TREND_FLAT);
    }
    lastVOC_partial = (int)voc_index;

    Paint_SelectImage(BlackImage);

    const UWORD clear_left  = LABEL_X + 36;      /* after "VOC:" */
    const UWORD clear_right = DIVIDER_X - 2;
    const UWORD clear_top   = y - 1;
    const UWORD clear_bot   = y + TextHeight(&Font12) - 2;

    EnsurePartialBase();

    if (!app.usb_on || app.cfg.vocMode == VOC_DISABLED) {
        Paint_ClearWindows(clear_left, clear_top, clear_right, clear_bot, WHITE);
        EPD_2in13_V4_Display_Partial(BlackImage);
        return;
    }
    Paint_ClearWindows(clear_left, clear_top, clear_right, clear_bot, WHITE);

    char buf[24];
    snprintf(buf, sizeof(buf), "%u idx", (unsigned)voc_index);
    UWORD voc_val_w = TextWidth(&Font12, buf);
    UWORD voc_val_x = (VALUE_RIGHT - 40) - voc_val_w;
    Paint_DrawString_EN(voc_val_x, y, buf, &Font12, WHITE, BLACK);

    DrawTrendArrow(ARROW_X, y, tVOC);

    if(app.time_sync_ok)
    {
    char tbuf[20];
    snprintf(tbuf, sizeof(tbuf), "%02u:%02u", app.voc_last_time.hour, app.voc_last_time.minute);
    UWORD ts_x = voc_val_x + voc_val_w + 6;
    Paint_DrawString_EN(ts_x, y, tbuf, &Font12, WHITE, BLACK);
    }

    EPD_2in13_V4_Display_Partial(BlackImage);
}


/* -------------------- CO2 calibration screen (partial) -------------------- */
/**
 * @brief Partial update for CO2 calibration progress from SCD_calib struct.
 *
 * Draws a header, reference ppm, progress bar and percentage text. Optionally
 * shows SCD_calib.result_msg if non-empty.
 */
void GUI_DrawCalibrationProgressFromStruct(void)
{
    Paint_SelectImage(BlackImage);

    Paint_DrawString_EN(10, 5, "CO2 Calibration", &Font16, WHITE, BLACK);

    char ppm_msg[32];
    snprintf(ppm_msg, sizeof(ppm_msg), "Reference: %u ppm", SCD_calib.reference_ppm);
    Paint_DrawString_EN(10, 25, ppm_msg, &Font12, WHITE, BLACK);

    uint16_t bar_x = 10, bar_y = 45, bar_w = 120, bar_h = 15;
    uint16_t fill_w = (bar_w * SCD_calib.progress_percent) / 100;
    Paint_DrawRectangle(bar_x, bar_y, bar_x + bar_w, bar_y + bar_h,
                        BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    if (fill_w > 0) {
        Paint_DrawRectangle(bar_x, bar_y, bar_x + fill_w, bar_y + bar_h,
                            BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    }

    Paint_ClearWindows(130, 43, 170, 58, WHITE);
    char percent_str[8];
    snprintf(percent_str, sizeof(percent_str), "%u%%", SCD_calib.progress_percent);
    Paint_DrawString_EN(bar_x + bar_w + 5, bar_y, percent_str, &Font12, WHITE, BLACK);

    if (strlen(SCD_calib.result_msg) > 0) {
        Paint_DrawString_EN(10, 70, SCD_calib.result_msg, &Font12, WHITE, BLACK);
    }

    EPD_2in13_V4_Display_Partial(BlackImage);
}

/* -------------------- Utilities -------------------- */
/**
 * @brief Fully clear the canvas to white (no display update yet).
 */
void EPDClear(void)
{
    Paint_SelectImage(BlackImage);
    Paint_Clear(WHITE);
}

/**
 * @brief Draw icon (connected or not) without full screen redraw.
 *
 * Draws the LoRa/WAN connectivity icon at the top-left corner. Does not trigger
 * a display refresh by itself.
 */
void GUI_DrawLoraIcon(uint8_t connected)
{
    UWORD iconX = 10;
    UWORD iconY = 0;
    if (connected)
        Paint_DrawBitMap_Paste(gImage_CommOK, iconX, iconY, 20, 20, 0);
    else
        Paint_DrawBitMap_Paste(gImage_CommNo, iconX, iconY, 20, 20, 0);
}

/**
 * @brief Draw a centered configuration icon and label, establish partial base,
 *        then clear the status area so we can show OK/ERR overlays.
 */
void GUI_DrawConfIcon(void)
{
    EPDClear();

    UWORD iconX = 125 - 30 ;
    UWORD iconY = 30;
    Paint_DrawBitMap_Paste(ConfigureIcon, iconX, iconY, 60, 60, 0);

    const char *label = "Configuration mode";
    UWORD w = TextWidth(&Font12, label);
    UWORD x = (250 > w ? (250 - w)/2 : 2);
    Paint_DrawString_EN(x, iconY + 60 + 6, (char*)label, &Font12, WHITE, BLACK);

    EPD_2in13_V4_Display(BlackImage);

    EPD_2in13_V4_LoadBase_RAMOnly(BlackImage);
    s_partial_base_valid = 1;

    //GUI_ConfClearStatus();
}

/* ===== Config-mode status overlay (partial only, icon stays) ===== */
static const UWORD CONF_STATUS_X0 = 20;
static const UWORD CONF_STATUS_Y0 = 100;
static const UWORD CONF_STATUS_X1 = 230;
static const UWORD CONF_STATUS_Y1 = 118;

/**
 * @brief Clear the narrow status strip used in configuration mode.
 *
 * The configuration icon and the rest of the screen remain intact; only the
 * status strip is blanked.
 */
void GUI_ConfClearStatus(void)
{
    Paint_SelectImage(BlackImage);
    EnsurePartialBase();
    Paint_ClearWindows(CONF_STATUS_X0, CONF_STATUS_Y0, CONF_STATUS_X1, CONF_STATUS_Y1, WHITE);
    EPD_2in13_V4_Display_Partial(BlackImage);
}

/**
 * @brief Show a single-line status text (e.g., "OK" or "ERR: ...") in config mode.
 *
 * Draws a framed strip at the bottom with the message centered, using a
 * partial refresh only over that strip.
 */
void GUI_ConfShowStatus(const char* msg)
{
    if (!msg) return;

    Paint_SelectImage(BlackImage);
    EnsurePartialBase();

    Paint_ClearWindows(CONF_STATUS_X0, CONF_STATUS_Y0, CONF_STATUS_X1, CONF_STATUS_Y1, WHITE);

    Paint_DrawRectangle(CONF_STATUS_X0, CONF_STATUS_Y0, CONF_STATUS_X1, CONF_STATUS_Y1,
                        BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);

    UWORD w = TextWidth(&Font12, msg);
    UWORD h = TextHeight(&Font12);
    UWORD cx = CONF_STATUS_X0 + ((CONF_STATUS_X1 - CONF_STATUS_X0) > w ? ((CONF_STATUS_X1 - CONF_STATUS_X0) - w)/2 : 0);
    UWORD cy = CONF_STATUS_Y0 + ((CONF_STATUS_Y1 - CONF_STATUS_Y0) > h ? ((CONF_STATUS_Y1 - CONF_STATUS_Y0) - h)/2 : 0);

    Paint_DrawString_EN(cx, cy, (char*)msg, &Font12, WHITE, BLACK);

    EPD_2in13_V4_Display_Partial(BlackImage);
}

/**
 * @brief Convenience helper to show "SET key=value : OK/ERR" in the status strip.
 */
void GUI_ConfShowSetResult(const char* key, const char* value, uint8_t ok)
{
    char line[80];
    if (!key) key = "?";
    if (!value) value = "?";

    snprintf(line, sizeof(line), "SET %s=%s : %s", key, value, ok ? "OK" : "ERR");
    GUI_ConfShowStatus(line);
}

/* ===== QR rendering ===== */

/**
 * @brief Compute integer scale so that (qrSize + 2*quiet) * scale fits into panelW×panelH.
 */
static int QR_ComputeScale(int panelW, int panelH, int qrSize, int quiet)
{
    int maxModules = qrSize + 2*quiet;
    int sx = panelW / maxModules;
    int sy = panelH / maxModules;
    int s  = (sx < sy ? sx : sy);
    return (s < 1 ? 1 : s);
}

/**
 * @brief Render raw qrcodegen matrix into the current canvas (BlackImage).
 *
 * Places the QR code centered, adds a quiet zone, and uses integer scale so
 * that edges stay crisp. Only black modules are drawn; background is white.
 */
static void GUI_DrawQR_Matrix(const uint8_t *qr, int quiet)
{
    const int W = EPD_2in13_V4_WIDTH;   // 250
    const int H = EPD_2in13_V4_HEIGHT;  // 122

    int qrSize = qrcodegen_getSize(qr);
    int scale  = QR_ComputeScale(W, H, qrSize, quiet);

    int imgW = (qrSize + 2*quiet) * scale;
    int x0 = (W - imgW)/2;
    int y0 = (H - imgW)/2;

    /* Clear background to white. */
    Paint_Clear(WHITE);

    /* Draw QR modules (black squares only). */
    for (int y = 0; y < qrSize; y++) {
        for (int x = 0; x < qrSize; x++) {
            if (!qrcodegen_getModule(qr, x, y)) continue;
            int px = x0 + (x + quiet) * scale;
            int py = y0 + (y + quiet) * scale;

            /* Fill a small scale×scale square – rectangle is fastest. */
            Paint_DrawRectangle(px, py, px + scale - 1, py + scale - 1,
                                BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        }
    }
}

/**
 * @brief Generate a QR code for the given text and render it on E-Ink (full refresh).
 *
 * Uses ECC=M and automatic mask selection. Optionally renders a footer text
 * under the QR (e.g., g_esp.pairing.manual).
 *
 * @param text   QR payload string (e.g. g_esp.pairing.qr).
 * @param footer Optional footer text under the QR (may be NULL).
 * @return 1 on success, 0 on encoding error.
 */
uint8_t GUI_DrawQR_TextFull(const char* text, const char* footer)
{
    if (!text || !text[0]) return 0;

    /* 1) Encode */
    uint8_t qrcode[qrcodegen_BUFFER_LEN_MAX];
    uint8_t temp  [qrcodegen_BUFFER_LEN_MAX];
    if (!qrcodegen_encodeText(text, temp, qrcode,
                              qrcodegen_Ecc_MEDIUM,
                              qrcodegen_VERSION_MIN, qrcodegen_VERSION_MAX,
                              qrcodegen_Mask_AUTO, true)) {
        return 0;
    }

    /* 2) Canvas dimensions (logical after 270° rotation). */
    const int W = 250, H = 122;
    int qrSize = qrcodegen_getSize(qrcode);

    /* Find scale and quiet so that (qrSize + 2*quiet) * scale fits into W×H. */
    int quiet = 3;
    int scale;
    for (;;) {
        int need = qrSize + 2*quiet;
        scale = (W / need < H / need) ? (W / need) : (H / need);
        if (scale >= 1) break;
        if (quiet > 1) { quiet--; continue; }
        /* Even quiet=1 does not fit -> fallback text. */
        Paint_SelectImage(BlackImage);
        Paint_Clear(WHITE);
        Paint_DrawString_EN(10, (H-12)/2, "QR too large", &Font12, WHITE, BLACK);
        EPD_2in13_V4_Display(BlackImage);
        EPD_2in13_V4_LoadBase_RAMOnly(BlackImage);
        s_partial_base_valid = 1;
        return 1;
    }

    /* 3) Render QR centered. */
    Paint_SelectImage(BlackImage);
    Paint_Clear(WHITE);

    int img = (qrSize + 2*quiet) * scale;
    int x0 = (W - img)/2 + quiet*scale;
    int y0 = (H - img)/2 + quiet*scale;

    for (int y = 0; y < qrSize; y++) {
        for (int x = 0; x < qrSize; x++) {
            if (!qrcodegen_getModule(qrcode, x, y)) continue;
            int px0 = x0 + x*scale;
            int py0 = y0 + y*scale;
            int px1 = px0 + scale - 1;
            int py1 = py0 + scale - 1;
            /* Clamp defensively to canvas bounds. */
            if (px0 < 0 || py0 < 0 || px0 >= W || py0 >= H) continue;
            if (px1 >= W) px1 = W - 1;
            if (py1 >= H) py1 = H - 1;
            Paint_DrawRectangle(px0, py0, px1, py1, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        }
    }

    /* 4) Optional footer (e.g. manual pairing code). */
    if (footer && footer[0]) {
        UWORD tw = TextWidth(&Font12, footer), th = TextHeight(&Font12);
        UWORD tx = (W > (int)tw ? (W - tw)/2 : 0);
        UWORD ty = (H > (int)th + 4 ? H - th - 4 : 0);
        Paint_DrawString_EN(tx, ty, (char*)footer, &Font12, WHITE, BLACK);
    }

    EPD_2in13_V4_Display(BlackImage);
    EPD_2in13_V4_LoadBase_RAMOnly(BlackImage);
    s_partial_base_valid = 1;
    return 1;
}

/**
 * @brief Fast partial variant for QR (currently unused, full refresh is used instead).
 *
 * Could be implemented to only redraw the QR rectangle when the QR content
 * changes frequently, but for simplicity the full-refresh variant is used.
 */
