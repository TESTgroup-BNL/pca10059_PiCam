#ifndef _GPS_H 
#define _GPS_H

#include <TimeLib.h>
#include <Wire.h>
#include <Metro.h>
#include <Adafruit_GPS.h>
#include <bluetooth.h>

extern Adafruit_GPS GPS;
extern Metro last_sync;

void cfgGPS(uint32_t timeout, uint32_t clk_sync_int, uint32_t pin, int8_t tz);
bool getGPSFix(bool location=false);
void syncTime();
void debugGPS();

#endif