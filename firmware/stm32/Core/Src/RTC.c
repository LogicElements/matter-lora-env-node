/*
 * RTC.c
 *
 *  Created on: Aug 7, 2025
 *      Author: Evzen Steif
 *
 *  @brief RTC helper functions: Unix time conversion, caching and printing.
 *
 *  This module provides small helpers around the STM32 HAL RTC:
 *    - Set RTC from a Unix epoch timestamp (UTC).
 *    - Read current Unix epoch from RTC.
 *    - Keep a cached TimeData_t structure (g_timeData).
 *    - Convenience helpers for seconds-of-day and numeric YYYYMMDD date.
 */

#include "RTC.h"
#include "stm32u0xx_hal.h"
#include "stm32u0xx_hal_rtc.h"
#include <time.h>

extern RTC_HandleTypeDef hrtc;

TimeData_t g_timeData;

/**
 * @brief Set the RTC date and time from a Unix epoch value (UTC).
 *
 * @param epoch_time Unix timestamp (seconds since 1970-01-01 00:00:00 UTC).
 */
void Set_RTC_From_Epoch(uint32_t epoch_time)
{
    time_t rawtime = (time_t)epoch_time;
    struct tm *timeinfo = gmtime(&rawtime);

    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours = timeinfo->tm_hour;
    sTime.Minutes = timeinfo->tm_min;
    sTime.Seconds = timeinfo->tm_sec;

    sDate.WeekDay = timeinfo->tm_wday == 0 ? 7 : timeinfo->tm_wday;
    sDate.Month = timeinfo->tm_mon + 1;
    sDate.Date = timeinfo->tm_mday;
    sDate.Year = timeinfo->tm_year - 100;

    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
}

/**
 * @brief Read current RTC date/time and convert it to Unix time.
 *
 * @return Current Unix timestamp (seconds since 1970-01-01 00:00:00).
 */
uint32_t RTC_Get_Unix_Time(void)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    struct tm timeinfo = {0};

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    timeinfo.tm_year = sDate.Year + 100;
    timeinfo.tm_mon  = sDate.Month - 1;
    timeinfo.tm_mday = sDate.Date;
    timeinfo.tm_hour = sTime.Hours;
    timeinfo.tm_min  = sTime.Minutes;
    timeinfo.tm_sec  = sTime.Seconds;

    time_t epoch = mktime(&timeinfo);

    return (uint32_t)epoch;
}

/**
 * @brief Read RTC, update g_timeData and print human-readable timestamp.
 */
void RTC_Print_DateTime(void)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    g_timeData.unix_time = RTC_Get_Unix_Time();
    g_timeData.year = sDate.Year + 2000;
    g_timeData.month = sDate.Month;
    g_timeData.day = sDate.Date;
    g_timeData.hour = sTime.Hours;
    g_timeData.minute = sTime.Minutes;
    g_timeData.second = sTime.Seconds;

    printf("\r\nRTC time: %04d-%02d-%02d %02d:%02d:%02d\r\n",
           sDate.Year + 2000,
           sDate.Month,
           sDate.Date,
           sTime.Hours,
           sTime.Minutes,
           sTime.Seconds);
}

/**
 * @brief Return number of seconds since midnight (0..86399).
 *
 * Uses current RTC time; date is only read to unlock the shadow registers.
 */
uint32_t RTC_GetSecondsOfDay(void) {
    RTC_TimeTypeDef t;
    RTC_DateTypeDef d;
    HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN); // must be read to unlock shadow

    return (uint32_t)t.Hours * 3600U +
           (uint32_t)t.Minutes * 60U +
           (uint32_t)t.Seconds;
}

/**
 * @brief Return numeric date in format YYYYMMDD (e.g. 20250908).
 *
 * @return Packed date as 32-bit integer.
 */
uint32_t RTC_GetYMD(void) {
    RTC_TimeTypeDef t;
    RTC_DateTypeDef d;
    HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);

    uint32_t y = 2000U + d.Year;
    return (y * 10000U) + (d.Month * 100U) + d.Date;
}

/**
 * @brief Refresh global g_timeData struct from current RTC date/time.
 */
void RTC_UpdateTimeData(void)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    g_timeData.unix_time = RTC_Get_Unix_Time();
    g_timeData.year   = sDate.Year + 2000;
    g_timeData.month  = sDate.Month;
    g_timeData.day    = sDate.Date;
    g_timeData.hour   = sTime.Hours;
    g_timeData.minute = sTime.Minutes;
    g_timeData.second = sTime.Seconds;
}
