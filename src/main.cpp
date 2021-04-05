#include <Arduino.h>
#include <Metro.h>
#include <bluetooth.h>
#include <gps.h>

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

File file(InternalFS);

#define Pi_Enable_Pin   17 //P0.17
#define GPS_Enable_Pin  20 //P0.20

#define PIN_WIRE_SDA    29
#define PIN_WIRE_SCL    31

#define IMAGE_INTERVAL   5         //mins - MUST BE A MULTIPLE OF CLK_RESOLUTION
#define BLE_INTERVAL     1         //mins - MUST BE A MULTIPLE OF CLK_RESOLUTION
#define GPS_INTERVAL     4         //hours - MUST BE A MULTIPLE OF CLK_RESOLUTION
#define CLK_SYNC_INT     3600*1000 //ms - GPS clock sync interval (minimum time before accepting an update)
#define CLK_RESOLUTION   5         //secs - this affects power consumption vs. response time

#define PI_ON_TIMEOUT    10*1000   //ms
#define PI_SAVE_TIMEOUT  30*1000   //ms
#define GPS_TIMEOUT      5*60*1000 //ms

#define MAX_BATT_VOLTAGE 5.4f      //used to scale batt level
#define LOW_BATT_CUTOFF  3.4f      //volts

// Offset hours from gps time (UTC)
#define TZ_OFFSET  -5  // Eastern Standard Time
//#define TZ_OFFSET  -9  // Alaska Standard Time

void enterStandby();
void exitStandby();
void startPi(bool);
void startPiPreview();

void getAdv(char*);
void getStatus(char*);
float getBattV();
float calcBattPerc(float);
float getTemp();
void getBLECmd(char cmd=0x00);
void showMenu();

void print2digits(int);
void blinkLED(int);
uint32_t freeRam(void);

Metro image_timer = Metro(IMAGE_INTERVAL*60*1000);
Metro ble_timer = Metro(BLE_INTERVAL*60*1000);
Metro gps_timer = Metro(GPS_INTERVAL*60*60*1000);
Metro uart_timer = Metro(1000);
Metro menu_timeout = Metro(30000);  //If no response after 30 seconds, resume normal operation

float battV = 0;
uint16_t img_count=0;
int16_t old_count=0;
uint16_t id=0;
uint32_t timer = millis();
bool inMenu=false;
char buf[255];
char adv_buf[31];
char pi_buf[255];

uint32_t clk_res = CLK_RESOLUTION*1000;

void setup() 
{ 
  getAdv(buf);
  setupBluetooth(buf);

  //Setup I/O
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  pinMode(GPS_Enable_Pin, OUTPUT);
  digitalWrite(GPS_Enable_Pin, LOW);
  pinMode(Pi_Enable_Pin, OUTPUT);
  digitalWrite(Pi_Enable_Pin, LOW);
  delay(2000); //Give everything a chance to reset

  Serial1.begin(460800);

  //Set batt voltage
  updateBatt(calcBattPerc(battV));

  //Get time and location from GPS
  cfgGPS(GPS_TIMEOUT, CLK_SYNC_INT, GPS_Enable_Pin, TZ_OFFSET);
  getGPSFix(true);

  //sync to seconds; something less ugly should replace this eventually
  if (connected) {
    writeBLE("Syncing timers to clock...");
  }

  while (second() != 0) {
    blinkLED(1);
    delay(300);
  }

  image_timer.reset();
  ble_timer.reset();
  gps_timer.reset();

  startPi(true);

  enterStandby();
}

void loop() 
{
  if (image_timer.check()) {
    startPi(false);
  }

  if (ble_timer.check() || (old_count != img_count)) {
    old_count = img_count;
    ble_timer.reset();
    getAdv(adv_buf);
    updateAdv(adv_buf);
  }

  if (gps_timer.check()) {
    getGPSFix(true);
  }

  if (connected) {
    battV = getBattV();
    updateBatt(calcBattPerc(battV));
    
    if (!inMenu) {
      getStatus(buf);
      writeBLE(buf, true, false);

      if (img_count==0) writeBLE("No images saved yet");
      else writeBLE(pi_buf);  //Send last log line
    }

    if (bleuart.available()>0) {
      if (inMenu) {
        getBLECmd();
        bleuart.flush();
        inMenu = false;
      }
      else {
        char cmd = bleuart.read();
        char cmd_nxt = bleuart.read();
        if ((cmd == cmd_nxt) && ((cmd == 'c') || (cmd == 'p') || (cmd == 'g') || (cmd == 't') || (cmd == 'i'))) getBLECmd(cmd);
        else {
          inMenu = true;
          showMenu();
        }
        bleuart.flush();
      }
    } 
  }

  delay(clk_res);
}

void getBLECmd(char cmd) {
  if (cmd == 0x00) cmd = bleuart.read();
  switch (cmd) {
    case 'c': {
      sprintf(pi_buf, "\nCfg:\n\timg int(min):%d\n\tBLE int(min):%d\n\tGPS int(hr):%d\n\tGPS clk int(min):%d\n\tGPS TO(s):%d\n\tClk res(s):%d\n\tPi pwr TO(s):%d\n\tImg cap TO(s)%d\n\tBatt Max(V):%0.2f\n\tLow Batt(V):%0.2f\n",
      IMAGE_INTERVAL, BLE_INTERVAL, GPS_INTERVAL, CLK_SYNC_INT/60000, GPS_TIMEOUT/1000, CLK_RESOLUTION, PI_ON_TIMEOUT/1000, PI_SAVE_TIMEOUT/1000, MAX_BATT_VOLTAGE, LOW_BATT_CUTOFF);
      writeBLE(pi_buf);
      break;
    }
    case 'i': {
      startPi(false);
      break;
    }
    case 'p': {
      startPiPreview();
      break;
    }
    case 'g': {
      getGPSFix(true);
      break;
    }
    case 't': {
      uint8_t yr = bleuart.parseInt();
      uint8_t mth = bleuart.parseInt();
      uint8_t dy = bleuart.parseInt();
      uint8_t h = bleuart.parseInt();
      uint8_t m = bleuart.parseInt();
      uint8_t s = bleuart.parseInt();
      setTime(h,m,s,dy,mth,yr);
      sprintf(pi_buf, "Time set: %04d-%02d-%02d %02d:%02d:%02d\n", year(), month(), day(), hour(), minute(), second());
      writeBLE(pi_buf);
      break;
    }
    default: {
      writeBLE("Unknown command\n");
      break;
    }
  }
}

void showMenu(void) {
  sprintf(pi_buf, "\nMenu:\n'c': Print cfg\n'g': Get GPS fix\n'p': Take picture now\n't [YYYY,MM,DD,hh,mm,ss]': Set time\nEnter command:");
  writeBLE(pi_buf);
}

void getAdv(char* statbuf) {
  //new name MUST be the same length or the soft device will reject it and stop advertising
  sprintf(statbuf, "PiCam%03d %2.0f %04d %02d/%02d %02d:%02d", id, calcBattPerc(getBattV()), img_count, month(), day(), hour(), minute());
}

void getStatus(char* statbuf) {
  sprintf(statbuf, "%04d-%02d-%02d %02d:%02d:%02d, imgs: %04d, batt: %.2f", year(), month(), day(), hour(), minute(), second(), img_count, getBattV());
}

void enterStandby() {
  Serial1.end();

  // Power cycle the UARTE0 - Fix or Workaround since UARTE0 is not getting turned Off if RX is enabled
  *(volatile uint32_t *)0x40002FFC = 0;
  *(volatile uint32_t *)0x40002FFC;
  *(volatile uint32_t *)0x40002FFC = 1;

  digitalWrite(GPS_Enable_Pin, LOW);
  digitalWrite(Pi_Enable_Pin, LOW);  

  nrf_gpio_cfg_default(PIN_WIRE_SDA);
  nrf_gpio_cfg_default(PIN_WIRE_SCL);
  nrf_gpio_cfg_default(PIN_SERIAL1_RX);
  nrf_gpio_cfg_default(PIN_SERIAL1_TX);
}

void exitStandby() {
  //pinMode(PIN_WIRE_SDA, OUTPUT_S0D1);
  //pinMode(PIN_WIRE_SCL, OUTPUT_S0D1);
  digitalWrite(Pi_Enable_Pin, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial1.begin(460800);
  Serial1.setTimeout(PI_ON_TIMEOUT);  
}

void startPi(bool newrun) {
  float batt = getBattV();
  if (batt < LOW_BATT_CUTOFF) {
    blinkLED(3);
    return;
  }

  sprintf(buf, "Capturing image %d\n", img_count);
  writeBLE(buf);

  exitStandby();

  //Serial1.println("Waiting for 'filename'");
  if (Serial1.find("fv?")) {
    delay(250);

    if (newrun) Serial1.print("*n");
    
    //UUID and datetime for filename
    char fn_buf[64];
    sprintf(fn_buf, "%02X%02X%02X%02X%02X%02X_%04d-%02d-%02d_%02d%02d%02d",
        mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],
        year(), month(), day(), hour(), minute(), second());
    Serial1.println(fn_buf);

    //Append other data for log
    //FULL OUTPUT FORMAT: [PiTime], filename, battery voltage, latitude, longitude, altitude, satellites, fix quality
    if (GPS.lat==0x00) {  //No GPS data yet
      sprintf(pi_buf, "%.2f,%.2f,0,X,0,X,0,0,0",
          batt, getTemp());
    }
    else {
      sprintf(pi_buf, "%.2f,%.2f,%.6f,%c,%.6f,%c,%f,%d,%d",
          batt, getTemp(),
          (double)GPS.latitude_fixed/10000000, GPS.lat,
          (double)GPS.longitude_fixed/10000000, GPS.lon,
          GPS.altitude, (int)GPS.satellites, (int)GPS.fixquality);
    }
    Serial1.println(pi_buf);
    
    //Wait for image to be saved
    Serial1.setTimeout(PI_SAVE_TIMEOUT);
    if (!Serial1.find("s!")) {
      writeBLE("Error: Image not saved.");
      blinkLED(3);
    }
    else {
      img_count++;
    }

    //Serial1.println("Done.");
    Serial1.flush();
    delay(200);
  }
  else writeBLE("Error: Pi didn't respond.");
  
  enterStandby();
}


void startPiPreview() {
  float batt = getBattV();
  if (batt < LOW_BATT_CUTOFF) {
    blinkLED(3);
    return;
  }
  writeBLE("Capturing preview...\n");

  exitStandby();

  if (Serial1.find("fv?")) {
    delay(250);

    Serial1.print("*p");

    //UUID and datetime for filename
    char fn_buf[64];
    sprintf(fn_buf, "%02X%02X%02X%02X%02X%02X_%04d-%02d-%02d_%02d%02d%02d",
        mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],
        year(), month(), day(), hour(), minute(), second());
    Serial1.println(fn_buf);
    
    //Wait for image to be saved
    Serial1.setTimeout(PI_SAVE_TIMEOUT);
    if (!Serial1.find("p!")) {
      writeBLE("Error: Preview not saved.");
      blinkLED(3);
    }
    else {
      //Read preview data
      uint32_t f_len = Serial1.parseInt();
      char img_buf[1024];

      Serial1.read(); //skip space
      Serial1.readBytes(img_buf, min(f_len, sizeof(img_buf)));

      sprintf(pi_buf, "img_blob:%06d:", f_len);
      writeBLE(pi_buf); 
      writeBLE(img_buf);
    }

    Serial1.flush();
    delay(200);
  }
  else writeBLE("Error: Pi didn't respond.");
  
  enterStandby();
}



void print2digits(int number) {
  if (number < 10) {
    Serial1.print("0"); // print a 0 before if the number is < than 10
  }
  Serial1.print(number);
}

float getBattV() {
  float measuredvbat = analogReadVDDHDIV5();
  measuredvbat /= (1024/18);         //10 bit ADC, div by 5, 3.6V ref
  return measuredvbat;  
}

float calcBattPerc(float v) {
  v -= LOW_BATT_CUTOFF;
  v /= (MAX_BATT_VOLTAGE - LOW_BATT_CUTOFF);  //Scale by actual useful voltage range
  v *= 100;              //Convert to percent
  return v;  
}

float getTemp() {
  return 0.0;
}

void blinkLED(int count) {
  for (int i=0;i<count;i++) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

uint32_t freeRam(){
    uint32_t stackTop;
    uint32_t heapTop;

    // current position of the stack.
    stackTop = (uint32_t) &stackTop;

    // current position of heap.
    void* hTop = malloc(1);
    heapTop = (uint32_t) hTop;
    free(hTop);

    // The difference is the free, available ram.
    return stackTop - heapTop;
}