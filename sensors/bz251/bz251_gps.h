/*
 * bz251_gps.h
 *
 *  Created on: Mar 12, 2026
 *  Updated   : May 12, 2026 — switched from NMEA $GNGGA parsing to u-blox UBX
 *              binary protocol. Single NAV-PVT message at 10 Hz gives us
 *              position, altitude, velocity (in knots and m/s NED), heading,
 *              fix info and timestamp in one parse.
 *
 *      Author: david / diego
 */

#ifndef BZ251_GPS_H_
#define BZ251_GPS_H_

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"

/* ---------------------------------------------------------------------------
 * Public data struct — populated by the GPS task on every successful NAV-PVT.
 * Read it under gps_data_mutex from other tasks.
 *
 * Conventions:
 *   - Latitude / longitude are in decimal degrees (positive = N / E).
 *     Stored as double because float only gives ~7 sig digits, which is the
 *     same as the lat/lon resolution itself — you lose precision after the
 *     decimal in float. Drones don't care, but it's a foot-gun for fusion.
 *   - Altitudes are in metres.
 *   - velocities in m/s; ground_speed_knots is also pre-computed for
 *     telemetry / pilot display convenience.
 *   - NED frame: +N, +E, +Down (matches the IMU body-to-NED convention).
 * ------------------------------------------------------------------------- */
typedef struct {
    /* Position */
    double  latitude_deg;        /* degrees, +N */
    double  longitude_deg;       /* degrees, +E */
    float   altitude_msl_m;      /* metres above mean sea level */
    float   altitude_ell_m;      /* metres above WGS84 ellipsoid */

    /* Velocity */
    float   ground_speed_mps;    /* 2-D horizontal ground speed [m/s] */
    float   ground_speed_knots;  /* same, pre-converted to knots */
    float   heading_deg;         /* heading of motion [deg], 0..360 */
    float   vel_north_mps;       /* NED north component */
    float   vel_east_mps;        /* NED east  component */
    float   vel_down_mps;        /* NED down  component */

    /* Fix quality */
    uint8_t fix_type;            /* 0=no fix, 1=DR, 2=2D, 3=3D, 4=GNSS+DR, 5=time */
    uint8_t satellites;          /* satellites used in nav solution */
    bool    is_valid;            /* true when fix_type >= 2 */

    /* GPS time-of-week and UTC timestamp (useful for delta-T and logging) */
    uint32_t i_tow_ms;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
} gps_data_t;

/* Shared data + mutex (defined in bz251_gps.c) */
extern gps_data_t        global_gps_data;
extern SemaphoreHandle_t gps_data_mutex;

/* FreeRTOS task entry points.
 *   - GPS_Task:        owns LPUART1, configures the BZ251 over UBX, parses
 *                      incoming frames and updates global_gps_data.
 *   - Tarea_Telemetry: optional debug task that prints the current fix
 *                      once per second. Wire it up only when you need it. */
void GPS_Task(void *pvParameters);
void Tarea_Telemetry(void *pvParameters);

#endif /* BZ251_GPS_H_ */
