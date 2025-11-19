/*
 * RTC.h
 *
 *  Created on: Aug 7, 2025
 *      Author: Evzen Steif
 *
 *  @brief Public interface for simple RTC helpers and cached time data.
 *
 *  This header exposes:
 *    - TimeData_t: cached timestamp structure (Unix + broken-down fields).
 *    - Helpers to set RTC from Unix epoch and read current Unix time.
 *    - Convenience functions for seconds-of-day and numeric YYYYMMDD date.
 *    - A function to refresh the global g_timeData from the RTC.
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

#include "main.h"
#include "stm32u0xx_hal.h"

/**
 * @brief Cached time snapshot mirrored from RTC.
 *
 * unix_time  - Unix timestamp (seconds since 1970-01-01 00:00:00).
 * year       - Full year (e.g. 2025).
 * month      - Month 1..12.
 * day        - Day 1..31.
 * hour       - Hour 0..23.
 * minute     - Minute 0..59.
 * second     - Second 0..59.
 * time_str   - Optional formatted string buffer (user-managed).
 */
typedef struct {
    uint32_t unix_time;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    char     time_str[32];
} TimeData_t;

/** Global cached time data, updated by RTC_UpdateTimeData() or RTC_Print_DateTime(). */
extern TimeData_t g_timeData;

/**
 * @brief Set RTC date/time from a Unix epoch timestamp (UTC).
 *
 * @param epoch_time Unix timestamp (seconds since 1970-01-01 00:00:00 UTC).
 */
void Set_RTC_From_Epoch(uint32_t epoch_time);

/**
 * @brief Read current RTC date/time and return it as Unix time.
 *
 * @return Unix timestamp (seconds since 1970-01-01 00:00:00).
 */
uint32_t RTC_Get_Unix_Time(void);

/**
 * @brief Read RTC, update g_timeData and print human-readable timestamp.
 */
void RTC_Print_DateTime(void);

/**
 * @brief Get number of seconds elapsed since local midnight (0..86399).
 */
uint32_t RTC_GetSecondsOfDay(void);

/**
 * @brief Get date encoded as YYYYMMDD (e.g., 20250908).
 *
 * @return Packed date (year*10000 + month*100 + day).
 */
uint32_t RTC_GetYMD(void);

/**
 * @brief Refresh global g_timeData from the current RTC date and time.
 */
void RTC_UpdateTimeData(void);

#endif /* INC_RTC_H_ */
