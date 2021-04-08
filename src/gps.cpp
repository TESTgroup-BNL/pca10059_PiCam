#include <gps.h>

uint32_t enable_pin;
int8_t tz_offset = 0;
Adafruit_GPS GPS(&Wire);
Metro gps_timeout;
Metro last_sync;

void cfgGPS(uint32_t timeout, uint32_t clk_sync_int, uint32_t pin, int8_t tz) {
  gps_timeout = Metro(timeout);
  last_sync = Metro(clk_sync_int);
  enable_pin = pin;
  tz_offset = tz;
}

bool getGPSFix(bool location) {
  //Power on and init GPS
  digitalWrite(enable_pin, HIGH);
  delay(500);

  bool gotFix = false;
  GPS.fix = false;    //forcibly reset so that a stale "true" doesn't mess us up

  if (GPS.begin(0x10)) {  // The I2C address to use is 0x10
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

    Metro msg_timer = Metro(1000);
    gps_timeout.reset();
    msg_timer.reset();
    
    gotFix = true;
    while((location && !GPS.fix) || (GPS.secondsSinceTime() > 60)) {
      //Wait until we have a location (if using location) or until we've gotten a new time (within the last minute; this only works because we're always cold starting)
      char c = GPS.read();
      
      if (GPS.newNMEAreceived()) {
        GPS.parse(GPS.lastNMEA());
        if (connected && msg_timer.check()) {
          char str_buf[255];
          sprintf(str_buf, "\nGPS: Fix:%d Sats:%d Quality:%d\n%s", GPS.fix, GPS.satellites, GPS.fixquality, GPS.lastNMEA());
          writeBLE(str_buf);
        }
      }

      if (gps_timeout.check()) {      //check timeout
        gotFix = false;
        break;
      }
      yield();
    }

    //If we're past the sync interval or never set the time and we have a new time, update the clock
    if ((last_sync.check()||(year()<2000)) && gotFix) syncTime();

    if (gotFix) writeBLE("Updated GPS info.");
    else writeBLE("Unable to acquire GPS fix.");
  }
  else writeBLE("Error: GPS module didn't respond.");
  
  Wire.end();
  digitalWrite(enable_pin, LOW);

  return gotFix;
}

void syncTime() {
    setTime(GPS.hour, GPS.minute, GPS.seconds, GPS.day, GPS.month, GPS.year);
    adjustTime(tz_offset * SECS_PER_HOUR); //Apply timezone offset
}

void debugGPS() {
    Serial1.print("\nTime: ");
    if (GPS.hour < 10) { Serial1.print('0'); }
    Serial1.print(GPS.hour, DEC); Serial1.print(':');
    if (GPS.minute < 10) { Serial1.print('0'); }
    Serial1.print(GPS.minute, DEC); Serial1.print(':');
    if (GPS.seconds < 10) { Serial1.print('0'); }
    Serial1.print(GPS.seconds, DEC); Serial1.print('.');
    if (GPS.milliseconds < 10) {
      Serial1.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial1.print("0");
    }
    Serial1.println(GPS.milliseconds);
    Serial1.print("Date: ");
    Serial1.print(GPS.day, DEC); Serial1.print('/');
    Serial1.print(GPS.month, DEC); Serial1.print("/20");
    Serial1.println(GPS.year, DEC);
    Serial1.print("Fix: "); Serial1.print((int)GPS.fix);
    Serial1.print(" quality: "); Serial1.println((int)GPS.fixquality);
    
    if (GPS.fix) {
      Serial1.print("Location: ");
      Serial1.print(GPS.latitude, 4); Serial1.print(GPS.lat);
      Serial1.print(", ");
      Serial1.print(GPS.longitude, 4); Serial1.println(GPS.lon);
      Serial1.print("Speed (knots): "); Serial1.println(GPS.speed);
      Serial1.print("Angle: "); Serial1.println(GPS.angle);
      Serial1.print("Altitude: "); Serial1.println(GPS.altitude);
      Serial1.print("Satellites: "); Serial1.println((int)GPS.satellites);
    }
}

void debugGPS_BLE() {
    Serial1.print("\nTime: ");
    if (GPS.hour < 10) { Serial1.print('0'); }
    Serial1.print(GPS.hour, DEC); Serial1.print(':');
    if (GPS.minute < 10) { Serial1.print('0'); }
    Serial1.print(GPS.minute, DEC); Serial1.print(':');
    if (GPS.seconds < 10) { Serial1.print('0'); }
    Serial1.print(GPS.seconds, DEC); Serial1.print('.');
    if (GPS.milliseconds < 10) {
      Serial1.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial1.print("0");
    }
    Serial1.println(GPS.milliseconds);
    Serial1.print("Date: ");
    Serial1.print(GPS.day, DEC); Serial1.print('/');
    Serial1.print(GPS.month, DEC); Serial1.print("/20");
    Serial1.println(GPS.year, DEC);
    Serial1.print("Fix: "); Serial1.print((int)GPS.fix);
    Serial1.print(" quality: "); Serial1.println((int)GPS.fixquality);
    
    if (GPS.fix) {
      Serial1.print("Location: ");
      Serial1.print(GPS.latitude, 4); Serial1.print(GPS.lat);
      Serial1.print(", ");
      Serial1.print(GPS.longitude, 4); Serial1.println(GPS.lon);
      Serial1.print("Speed (knots): "); Serial1.println(GPS.speed);
      Serial1.print("Angle: "); Serial1.println(GPS.angle);
      Serial1.print("Altitude: "); Serial1.println(GPS.altitude);
      Serial1.print("Satellites: "); Serial1.println((int)GPS.satellites);
    }
}