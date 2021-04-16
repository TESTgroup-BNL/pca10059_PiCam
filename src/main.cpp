#include <Arduino.h>
#include <Metro.h>
#include <bluetooth.h>
#include <gps.h>

#define Pi_Enable_Pin   17 //P0.17
#define GPS_Enable_Pin  20 //P0.20

#define PIN_WIRE_SDA    29
#define PIN_WIRE_SCL    31

uint16_t ID=0;
uint16_t IMAGE_INTERVAL  = 30;        //mins - MUST BE A MULTIPLE OF CLK_RESOLUTION
uint8_t  BLE_INTERVAL    = 1;         //mins - MUST BE A MULTIPLE OF CLK_RESOLUTION
uint16_t GPS_INTERVAL    = 24;        //hours - MUST BE A MULTIPLE OF CLK_RESOLUTION
uint32_t CLK_SYNC_INT    = 60000;     //ms - GPS clock sync interval (minimum time before accepting an update)    
uint32_t CLK_RESOLUTION  = 5000;      //ms - this affects power consumption vs. response time

uint16_t PI_ON_TIMEOUT   = 10*1000;   //ms
uint16_t PI_SAVE_TIMEOUT = 30*1000;   //ms
uint32_t GPS_TIMEOUT     = 2*60*1000; //ms

float MAX_BATT_VOLTAGE   = 5.2f;      //used to scale batt level
float LOW_BATT_CUTOFF    = 3.4f;      //volts

// Offset hours from gps time (UTC)
int8_t TZ_OFFSET         = -5;  // Eastern Standard Time
                      // = -9;  // Alaska Standard Time

void enterStandby();
void exitStandby();
void startPi(bool);
void startPiPreview();

void parseConfig(char* cmd_buf);

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

//Metro image_timer = Metro(IMAGE_INTERVAL*60*1000);
Metro ble_timer = Metro(BLE_INTERVAL*60*1000);
Metro gps_timer = Metro(GPS_INTERVAL*60*60*1000);
Metro uart_timer = Metro(1000);
Metro menu_timeout = Metro(30000);  //If no response after 30 seconds, resume normal operation

float battV = 0;
uint16_t img_count=0;
uint16_t run_count=0;
float free_space = 0;
float total_space = 0;
int16_t old_count=0;

uint32_t timer = millis();
bool inMenu=false;
char buf[255];
char adv_buf[31];
char pi_buf[255];

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

  Serial1.begin(230400);

  //Set batt voltage
  updateBatt(calcBattPerc(battV));

  startPi(true);

  //Get time and location from GPS
  cfgGPS(GPS_TIMEOUT, CLK_SYNC_INT, GPS_Enable_Pin, TZ_OFFSET);
  getGPSFix(true);

/*
  //sync to seconds; something less ugly should replace this eventually
  if (connected) {
    writeBLE("Syncing timers to clock...");
  }

  while (second() != 0) {
    blinkLED(1);
    delay(300);
  }
*/

  //image_timer.reset();
  ble_timer.reset();
  gps_timer.reset();

  enterStandby();
}

void loop() 
{
  if ((second() < CLK_RESOLUTION/1000) && ((hour()*60) + minute()) % IMAGE_INTERVAL == 0) {
    //(second() < CLK_RESOLUTION) tells us effectively when we're on a minute
    //(((hour()*60) + minute()) % IMAGE_INTERVAL == 0) keeps everything relative to midnight so intervals don't drift over time
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
      writeBLE(buf);

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
        if ((cmd == cmd_nxt) && ((cmd == 'c') || (cmd == 'p') || (cmd == 'g') || (cmd == 't') || (cmd == 'i') || (cmd == 'r'))) getBLECmd(cmd);
        else {
          inMenu = true;
          showMenu();
        }
        bleuart.flush();
      }
    } 
  }

  delay(CLK_RESOLUTION);
}

void getBLECmd(char cmd) {
  if (cmd == 0x00) cmd = bleuart.read();
  switch (cmd) {
    case 'c': {
      sprintf(buf, "\nCfg:\n\tIMAGE_INTERVAL(min):%d\n\tBLE_INTERVAL(min):%d\n\tGPS_INTERVAL(hr):%d\n\tCLK_SYNC_INT(min):%0.1f\n\tGPS_TIMEOUT(s):%d\n\tCLK_RESOLUTION(ms):%d",
      IMAGE_INTERVAL, BLE_INTERVAL, GPS_INTERVAL, (double)CLK_SYNC_INT/60000, GPS_TIMEOUT/1000, CLK_RESOLUTION);
      writeBLE(buf, false);
      //split into 2 packets since it's over 255 chars
      sprintf(buf, "\n\tPI_ON_TIMEOUT(s):%d\n\tPI_SAVE_TIMEOUT(s):%d\n\tMAX_BATT_VOLTAGE(V):%0.2f\n\tLOW_BATT_CUTOFF(V):%0.2f\n\tTZ_OFFSET(hr):%d\n",
      PI_ON_TIMEOUT/1000, PI_SAVE_TIMEOUT/1000, MAX_BATT_VOLTAGE, LOW_BATT_CUTOFF, TZ_OFFSET);
      writeBLE(buf);
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
    case 'r': {
      startPi(true);
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
      sprintf(buf, "Time set: %04d-%02d-%02d %02d:%02d:%02d\n", year(), month(), day(), hour(), minute(), second());
      writeBLE(buf);
      break;
    }
    default: {
      writeBLE("Unknown command\n");
      break;
    }
  }
}

void showMenu(void) {
  sprintf(pi_buf, "\nMenu:\n'c': Print cfg\n'g': Get GPS fix\n'i': Capture img now\n'r': Reload cfg\n't [YYYY,MM,DD,hh,mm,ss]': Set time\nEnter command:");
  writeBLE(pi_buf);
}

void getAdv(char* statbuf) {
  //new name MUST be the same length or the soft device will reject it and stop advertising
  sprintf(statbuf, "PiCam%03d %02d %04d %02d/%02d %02d:%02d", ID, (int)min(calcBattPerc(getBattV()),99), img_count, month(), day(), hour(), minute());
}

void getStatus(char* statbuf) {
  if (GPS.lat==0x00) {  //No GPS data yet
    sprintf(statbuf, "%04d-%02d-%02d %02d:%02d:%02d, imgs: %04d, runs: %d, dfs: %.2f/%.2fGB, batt: %.2f, loc: 00.000000X 00.000000X",
      year(), month(), day(), hour(), minute(), second(),
      img_count, run_count, free_space, total_space, getBattV());
  }
  else {
    sprintf(statbuf, "%04d-%02d-%02d %02d:%02d:%02d, imgs: %04d, runs: %d, dfs: %.2f/%.2fGB, batt: %.2f, loc: %02.6f%c %02.6f%c",
      year(), month(), day(), hour(), minute(), second(),
      img_count, run_count, free_space, total_space, getBattV(),
      (double)GPS.latitude_fixed/10000000, GPS.lat,
      (double)GPS.longitude_fixed/10000000, GPS.lon);
  }
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
  Serial1.begin(230400);
  Serial1.setTimeout(PI_ON_TIMEOUT);  
}

void startPi(bool newrun) {
  float batt = getBattV();
  if (batt < LOW_BATT_CUTOFF) {
    blinkLED(3);
    return;
  }

  sprintf(buf, "Capturing image %d...", img_count);
  writeBLE(buf);

  exitStandby();

  //Serial1.println("Waiting for 'filename'");
  if (Serial1.find("fv?")) {
    delay(250);

    //UUID and datetime for filename
    char fn_buf[64];
    sprintf(fn_buf, "%03d_%02X%02X%02X%02X%02X%02X_%04d-%02d-%02d_%02d%02d%02d",
        ID, mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],
        year(), month(), day(), hour(), minute(), second());

    if (newrun) {
      Serial1.print("*n");
      Serial1.println(fn_buf);
      
      //Check for config file on SD card
      char serdat[255];
      Metro cfg_timer = Metro(15000);   //Timeout after 15 seconds
      cfg_timer.reset();

      Serial1.println(serdat);          //Request next line
      while (1) {
        size_t len = Serial1.readBytesUntil('\n',serdat,255);
        if (serdat[len-2] == '\r') serdat[len-2] = 0x00;    //Deal with different OS EOLs
        else serdat[len-1] = 0x00;
        while(Serial1.available()) Serial1.read();          //Flush anything left in the buffer
        //writeBLE(serdat);

        if (strstr(serdat,"!n") != NULL) break;
        else {
          parseConfig(serdat);
          Serial1.println(serdat);    //Request next line
        }

        if (cfg_timer.check()) {
          writeBLE("Timeout getting new config data.");
          break;
        }
      }
      getBLECmd('c');  //Print new config
      img_count = 0;
    }
    else Serial1.println(fn_buf);

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
      free_space = Serial1.parseFloat();
      total_space = Serial1.parseFloat();
      run_count = Serial1.parseInt();
      img_count++;
      sprintf(buf, "Done, Free disk space: %.2f/%.2fGB", free_space, total_space);
      writeBLE(buf,true);     
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
  writeBLE("Capturing preview...");

  exitStandby();

  if (Serial1.find("fv?")) {
    delay(250);

    Serial1.print("*p");

    //UUID and datetime for filename
    char fn_buf[64];
    sprintf(fn_buf, "%03d_%02X%02X%02X%02X%02X%02X_%04d-%02d-%02d_%02d%02d%02d",
        ID, mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],
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

      sprintf(buf, "Getting data from Pi (%d kB)", f_len/1024);
      writeBLE(buf);

      char* img_buf = (char*) malloc(204800);
      //char img_buf[102400];

      Serial1.readStringUntil('*'); //skip space(s)
      Serial1.readBytes(img_buf, min(f_len, 204800));

      sprintf(buf, "img_blob:%06d:", f_len);
      writeBLE(buf, false, true);

      const uint16_t packet_size = 200; //>=255 causes issues at packet boundaries
      for (uint32_t buf_pos=0;buf_pos<=(f_len-1);buf_pos+=packet_size) {
        writeBLE(&img_buf[buf_pos], false, true, min(packet_size,(f_len-buf_pos)));
      } 

      free(img_buf);
    }
    writeBLE("Preview sent.");

    Serial1.flush();
    delay(200);
  }
  else writeBLE("Error: Pi didn't respond.");
  
  enterStandby();
}

void parseConfig(char* cmd_buf) {
    char* cmd;
    char* val;
    cmd = strtok(cmd_buf, " ");
    val = strtok(NULL, " ");

    if (strstr(cmd_buf, "ID") != NULL) {
      unsigned int tmp = ID;                //if scan fails, keep default
      sscanf(val, "%u", &tmp);
      ID = constrain(tmp, 0, 999);
    }
    else if (strstr(cmd_buf, "IMAGE_INTERVAL") != NULL) {
      unsigned int tmp = IMAGE_INTERVAL;    //if scan fails, keep default
      sscanf(val, "%u", &tmp);
      IMAGE_INTERVAL = constrain(tmp, 1, 1440);
    }
    else if (strstr(cmd_buf, "BLE_INTERVAL") != NULL) {
      unsigned int tmp = BLE_INTERVAL;      //if scan fails, keep default
      sscanf(val, "%u", &tmp);
      BLE_INTERVAL = constrain(tmp, 1, 1440);
      ble_timer.interval(BLE_INTERVAL*60*1000);
      ble_timer.reset(); 
    }
    else if (strstr(cmd_buf, "GPS_INTERVAL") != NULL) {
      unsigned int tmp = GPS_INTERVAL;      //if scan fails, keep default
      sscanf(val, "%u", &tmp);
      GPS_INTERVAL = constrain(tmp, 1, 720);
      gps_timer.interval(GPS_INTERVAL*60*60*1000);
      gps_timer.reset();
    }
    else if (strstr(cmd_buf, "CLK_SYNC_INT") != NULL) {
      uint32_t tmp = CLK_SYNC_INT;      //if scan fails, keep default
      sscanf(val, "%lu", &tmp);
      CLK_SYNC_INT = constrain(tmp, 1000, ((uint32_t)86400)*(uint32_t)1000);
    }
    else if (strstr(cmd_buf, "CLK_RESOLUTION") != NULL) {
      uint32_t tmp = CLK_RESOLUTION;    //if scan fails, keep default
      sscanf(val, "%lu", &tmp);
      CLK_RESOLUTION = constrain(tmp, 100, (uint32_t)60*(uint32_t)1000);
    }
    else if (strstr(cmd_buf, "PI_ON_TIMEOUT") != NULL) {
      unsigned int tmp = PI_ON_TIMEOUT;     //if scan fails, keep default
      sscanf(val, "%u", &tmp);
      PI_ON_TIMEOUT = constrain(tmp, 1, 60*1000);
    }
    else if (strstr(cmd_buf, "PI_SAVE_TIMEOUT") != NULL) {
      unsigned int tmp = PI_SAVE_TIMEOUT;   //if scan fails, keep default
      sscanf(val, "%u", &tmp);
      PI_SAVE_TIMEOUT = constrain(tmp, 1, 60*1000);
    }
    else if (strstr(cmd_buf, "GPS_TIMEOUT") != NULL) {
      uint32_t tmp = GPS_TIMEOUT;       //if scan fails, keep default
      sscanf(val, "%lu", &tmp);
      GPS_TIMEOUT = constrain(tmp, 1000, 30*60*1000);
    }
    else if (strstr(cmd_buf, "MAX_BATT_VOLTAGE") != NULL) {
      //float tmp = MAX_BATT_VOLTAGE;     //if scan fails, keep default
      //sscanf(val, "%f", &tmp);
      MAX_BATT_VOLTAGE = constrain(atof(val), 3.3, 5.5);
    }
    else if (strstr(cmd_buf, "LOW_BATT_CUTOFF") != NULL) {
      //float tmp = LOW_BATT_CUTOFF;     //if scan fails, keep default
      //sscanf(val, "%f", &tmp);
      LOW_BATT_CUTOFF = constrain(atof(val), 1.2, MAX_BATT_VOLTAGE);
    }
    else if (strstr(cmd_buf, "TZ_OFFSET") != NULL) {
      int tmp = TZ_OFFSET;   //if scan fails, keep default
      sscanf(val, "%d", &tmp);
      TZ_OFFSET = constrain(tmp, -12, 12);
    }
    char* first_null = strchr(cmd_buf, 0);
    *first_null = ' '; //Get rid of the first null so that we can print cmd and val as one array
}

/*
void parseConfig(char* cmd_buf) {
    char* pos;
    pos = strchr(cmd_buf, ' ');
    pos = strchr(pos+1, ' ');
    *pos = 0x00; //terminate string at second space/delimiter

    if (strstr(cmd_buf, "ID") != NULL) {
      uint16_t tmp = ID;                //if scan fails, keep default
      sscanf(cmd_buf, "%*s %u", &tmp);
      ID = constrain(tmp, 0, 999);
    }
    else if (strstr(cmd_buf, "IMAGE_INTERVAL") != NULL) {
      uint16_t tmp = IMAGE_INTERVAL;    //if scan fails, keep default
      sscanf(cmd_buf, "%*s %u", &tmp);
      IMAGE_INTERVAL = constrain(tmp, 1, 1440);
    }
    else if (strstr(cmd_buf, "BLE_INTERVAL") != NULL) {
      uint16_t tmp = BLE_INTERVAL;      //if scan fails, keep default
      sscanf(cmd_buf, "%*s %u", &tmp);
      BLE_INTERVAL = constrain(tmp, 1, 1440);
      ble_timer.interval(BLE_INTERVAL*60*1000);
      ble_timer.reset(); 
    }
    else if (strstr(cmd_buf, "GPS_INTERVAL") != NULL) {
      uint16_t tmp = GPS_INTERVAL;      //if scan fails, keep default
      sscanf(cmd_buf, "%*s %u", &tmp);
      GPS_INTERVAL = constrain(tmp, 1, 720);
      gps_timer.interval(GPS_INTERVAL*60*60*1000);
      gps_timer.reset();
    }
    else if (strstr(cmd_buf, "CLK_SYNC_INT") != NULL) {
      uint32_t tmp = CLK_SYNC_INT;      //if scan fails, keep default
      sscanf(cmd_buf, "%*s %u", &tmp);
      CLK_SYNC_INT = constrain(tmp, 1000, 24*60*60*1000);
    }
    else if (strstr(cmd_buf, "CLK_RESOLUTION") != NULL) {
      uint32_t tmp = CLK_RESOLUTION;    //if scan fails, keep default
      sscanf(cmd_buf, "%*s %u", &tmp);
      CLK_RESOLUTION = constrain(tmp, 100, 60000);
    }
    else if (strstr(cmd_buf, "PI_ON_TIMEOUT") != NULL) {
      uint16_t tmp = PI_ON_TIMEOUT;     //if scan fails, keep default
      sscanf(cmd_buf, "%*s %u", &tmp);
      PI_ON_TIMEOUT = constrain(tmp, 1, 60);
    }
    else if (strstr(cmd_buf, "PI_SAVE_TIMEOUT") != NULL) {
      uint16_t tmp = PI_SAVE_TIMEOUT;   //if scan fails, keep default
      sscanf(cmd_buf, "%*s %u", &tmp);
      PI_SAVE_TIMEOUT = constrain(tmp, 1, 120);
    }
    else if (strstr(cmd_buf, "GPS_TIMEOUT") != NULL) {
      uint32_t tmp = GPS_TIMEOUT;       //if scan fails, keep default
      sscanf(cmd_buf, "%*s %d", &tmp);
      GPS_TIMEOUT = constrain(tmp, 1000, 30*60*1000);
    }
    else if (strstr(cmd_buf, "MAX_BATT_VOLTAGE") != NULL) {
      float tmp = MAX_BATT_VOLTAGE;     //if scan fails, keep default
      sscanf(cmd_buf, "%*s %f", &tmp);
      MAX_BATT_VOLTAGE = constrain(tmp, 3.3, 5.5);
    }
    else if (strstr(cmd_buf, "LOW_BATT_CUTOFF") != NULL) {
      float tmp = LOW_BATT_CUTOFF;     //if scan fails, keep default
      sscanf(cmd_buf, "%*s %f", &tmp);
      LOW_BATT_CUTOFF = constrain(tmp, 1.2, MAX_BATT_VOLTAGE);
    }
    else if (strstr(cmd_buf, "TZ_OFFSET") != NULL) {
      int8_t tmp = TZ_OFFSET;   //if scan fails, keep default
      sscanf(cmd_buf, "%*s %d", &tmp);
      TZ_OFFSET = constrain(tmp, -12, 12);
    }
} */

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