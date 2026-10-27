/*
 * GeoFox-K5: 360° Polar Radio Direction Finding (RDF) & 2-Point Triangulation
 * Author: Efe Işık (TA1EEI)
 * Target: Quansheng UV-K5 / UV-K6 (DP32G030 Cortex-M0 + BK4819 + ST7565 128x64 LCD)
 * Compatible with: F4HWN & Egzumer Custom Firmware trees
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "driver/bk4819.h"
#include "driver/st7565.h"
#include "driver/keyboard.h"
#include "driver/system.h"
#include "ui/helper.h"

#define RDF_SECTORS 8

// Fixed-point Sin/Cos Lookup Table (scaled by 256) for 8 compass sectors (0°, 45°, ..., 315°)
static const int16_t SIN_LUT8[RDF_SECTORS] = {   0,  181,  256,  181,    0, -181, -256, -181 };
static const int16_t COS_LUT8[RDF_SECTORS] = { 256,  181,    0, -181, -256, -181,    0,  181 };

typedef struct {
    int16_t  sector_rssi[RDF_SECTORS]; // dBm values in 8 directions (N, NE, E, SE, S, SW, W, NW)
    uint8_t  current_sector;
    uint16_t bearing_deg;              // Calculated target bearing (0..359 deg)
    uint16_t bearing_point_a;          // Saved Point A bearing
    uint16_t bearing_point_b;          // Saved Point B bearing
    uint16_t baseline_meters;          // Walked baseline distance A -> B (default 100m)
    uint16_t est_distance_m;           // Triangulated target distance in meters
    uint8_t  vario_tone_enabled;       // Audio RSSI bridge (Mod 3) for smartphone GPS/Map fusion
} GeoFoxState_t;

static GeoFoxState_t gGeoFox = {
    .sector_rssi = { -110, -110, -110, -110, -110, -110, -110, -110 },
    .current_sector = 0,
    .bearing_deg = 0,
    .bearing_point_a = 0xFFFF,
    .bearing_point_b = 0xFFFF,
    .baseline_meters = 100,
    .est_distance_m = 0,
    .vario_tone_enabled = 0
};

// Convert BK4819 REG_67 raw value to dBm (-160 to -20 dBm)
static int16_t GEOFOX_ReadRssiDbm(void) {
    uint16_t raw = BK4819_GetRSSI();
    return (int16_t)(raw / 2) - 160;
}

// Emit RSSI-proportional audio pitch (600 Hz .. 2200 Hz) for Smartphone Map Bridge (Mod 3)
static void GEOFOX_UpdateVarioTone(int16_t rssi_dbm) {
    if (!gGeoFox.vario_tone_enabled) {
        return;
    }
    int32_t clamped = rssi_dbm;
    if (clamped < -120) clamped = -120;
    if (clamped > -40)  clamped = -40;
    uint16_t freq_hz = (uint16_t)(600 + (clamped + 120) * 20);
    // BK4819_REG_71 tone tuning word = freq_hz * 10.32444f (~ (freq_hz * 10572) >> 10)
    uint16_t tone_word = (uint16_t)(((uint32_t)freq_hz * 10572U) >> 10);
    BK4819_WriteRegister(BK4819_REG_70, 0x8000 | (96U << 8)); // Enable Tone1 gain
    BK4819_WriteRegister(BK4819_REG_71, tone_word);
}

// Compute vector centroid bearing from 8 sector RSSI measurements (Body-Shielding Peak/Null)
static void GEOFOX_ComputeVectorBearing(void) {
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int16_t min_rssi = 0;

    for (uint8_t i = 0; i < RDF_SECTORS; i++) {
        if (gGeoFox.sector_rssi[i] < min_rssi || i == 0) {
            min_rssi = gGeoFox.sector_rssi[i];
        }
    }

    for (uint8_t i = 0; i < RDF_SECTORS; i++) {
        int32_t weight = (int32_t)(gGeoFox.sector_rssi[i] - min_rssi) + 1;
        sum_x += weight * SIN_LUT8[i]; // East component
        sum_y += weight * COS_LUT8[i]; // North component
    }

    // Fast octant/quadrant integer atan2 approximation (0..359 deg)
    int32_t ax = sum_x < 0 ? -sum_x : sum_x;
    int32_t ay = sum_y < 0 ? -sum_y : sum_y;
    int32_t angle = 0;
    if (ax + ay > 0) {
        angle = (ax > ay) ? (45 * ay) / ax : 90 - (45 * ax) / ay;
    }
    if (sum_x >= 0 && sum_y >= 0)      gGeoFox.bearing_deg = (uint16_t)(90 - angle);
    else if (sum_x >= 0 && sum_y < 0)  gGeoFox.bearing_deg = (uint16_t)(90 + angle);
    else if (sum_x < 0 && sum_y < 0)   gGeoFox.bearing_deg = (uint16_t)(270 - angle);
    else                               gGeoFox.bearing_deg = (uint16_t)(270 + angle);
}

// 2-Point Law of Sines Triangulation: d = Baseline * sin(90 - dTheta) / sin(|thetaB - thetaA|)
static void GEOFOX_ComputeTriangulation(void) {
    if (gGeoFox.bearing_point_a == 0xFFFF || gGeoFox.bearing_point_b == 0xFFFF) return;
    int16_t diff = (int16_t)gGeoFox.bearing_point_b - (int16_t)gGeoFox.bearing_point_a;
    if (diff < 0) diff = -diff;
    if (diff > 180) diff = 360 - diff;
    if (diff < 8) diff = 8; // Prevent division by near-zero parallel rays

    // Small-angle / chord approximation for Cortex-M0 without libm
    uint32_t num = (uint32_t)gGeoFox.baseline_meters * 57U;
    gGeoFox.est_distance_m = (uint16_t)(num / (uint32_t)diff);
}

// Set a single pixel in ST7565 gFrameBuffer (x: 0..127, y: 0..63)
static void GEOFOX_PutPixel(uint8_t x, uint8_t y) {
    if (x >= 128 || y >= 64) return;
    if (y < 8) {
        gStatusLine[x] |= (1U << y);
    } else {
        uint8_t row = (y - 8) >> 3;
        uint8_t bit = (y - 8) & 7;
        gFrameBuffer[row][x] |= (1U << bit);
    }
}

// Render 360° Polar Radar & Telemetry on UV-K5 128x64 LCD
void GEOFOX_RenderScreen(void) {
    memset(gStatusLine, 0, sizeof(gStatusLine));
    memset(gFrameBuffer, 0, sizeof(gFrameBuffer));

    const uint8_t cx = 30, cy = 32, r_max = 26;
    // Draw outer radar circle & crosshair axes
    for (uint8_t i = 0; i < 64; i++) {
        GEOFOX_PutPixel(cx, i);
        GEOFOX_PutPixel(i, cy);
    }

    // Plot 8-sector RSSI polygon vertices
    for (uint8_t s = 0; s < RDF_SECTORS; s++) {
        int16_t norm = gGeoFox.sector_rssi[s] + 125;
        if (norm < 4) norm = 4;
        if (norm > r_max) norm = r_max;
        int16_t px = cx + ((norm * SIN_LUT8[s]) >> 8);
        int16_t py = cy - ((norm * COS_LUT8[s]) >> 8);
        GEOFOX_PutPixel((uint8_t)px, (uint8_t)py);
        GEOFOX_PutPixel((uint8_t)(px + 1), (uint8_t)py);
    }

    // Telemetry text on right half (x = 64..127)
    char buf[22];
    int16_t cur_rssi = GEOFOX_ReadRssiDbm();
    GEOFOX_UpdateVarioTone(cur_rssi);

    sprintf(buf, "RSSI:%d dBm", cur_rssi);
    UI_PrintStringSmall(buf, 62, 0, 0);

    sprintf(buf, "YON :%03u deg", gGeoFox.bearing_deg);
    UI_PrintStringSmall(buf, 62, 0, 2);

    sprintf(buf, "BAZ :%um", gGeoFox.baseline_meters);
    UI_PrintStringSmall(buf, 62, 0, 4);

    if (gGeoFox.est_distance_m > 0) {
        sprintf(buf, "HEDEF:%um", gGeoFox.est_distance_m);
    } else {
        sprintf(buf, "A/B KERT SEC");
    }
    UI_PrintStringSmall(buf, 62, 0, 6);

    ST7565_BlitStatusLine();
    ST7565_BlitFullScreen();
}
