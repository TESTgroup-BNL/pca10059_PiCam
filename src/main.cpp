#include <Arduino.h>
#include <TimeLib.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <bluefruit.h>
#include "nrf_usbd.h"

#define Pi_Enable_Pin   17 //P0.17
#define GPS_Enable_Pin  20 //P0.20

#define PIN_WIRE_SDA    29
#define PIN_WIRE_SCL    31

#define LOW_BATT_CUTOFF  3.4f      //volts
#define PI_ON_TIMEOUT    10*1000   //ms
#define PI_SAVE_TIMEOUT  30*1000   //ms
#define GPS_TIMEOUT      1*10*1000 //ms


Adafruit_GPS GPS(&Wire);

BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEBas  blebas;  // battery
BLEUart bleuart; // uart over ble

void setupAdv(void);
void setupServices(void);
void updateAdv(uint16_t);
void getStatus(char*);
void updateConnected(void);
void connect_callback(uint16_t);
void disconnect_callback(uint16_t, uint8_t);

void getGPSFix();
void debugGPS();
void enterStandby();

uint16_t i=0;
uint16_t id=0;
uint8_t mac[6];
volatile bool connected = false;
char buf[31];
float battV = 0;
uint32_t timer = millis();

void setupAdv(void) {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);

  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(1600, 8000);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(1);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void setupServices(void) {
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  blebas.begin();
  blebas.write(100);
}

void updateAdv(uint16_t n) {
  Bluefruit.ScanResponse.clearData();
  getStatus(buf);
  Bluefruit.setName(buf);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.start(0);
}

void getStatus(char* buf) {
  //new name MUST be the same length or the soft device will reject it and stop advertising
  sprintf(buf, "PiCam%03d img %04d %02d/%02d %02d:%02d", id, 0, month(), day(), hour(), minute());
}


void setup() 
{ 
  Bluefruit.begin();
  Bluefruit.autoConnLed(false);
  Bluefruit.setTxPower(0);
 
  getStatus(buf);
  Bluefruit.setName(buf);
  Bluefruit.getAddr(mac);

  setupServices();
  setupAdv();

  //These probably aren't needed, but just in case
  sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
  sd_power_dcdc0_mode_set(NRF_POWER_DCDC_ENABLE);
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
 
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  pinMode(GPS_Enable_Pin, OUTPUT);
  digitalWrite(GPS_Enable_Pin, LOW);
  pinMode(Pi_Enable_Pin, OUTPUT);
  digitalWrite(Pi_Enable_Pin, LOW);
  delay(2000); //Give everything a chance to reset


  Serial1.begin(115200);
  Serial1.write("Hello!\n");
  Serial1.flush();

  //getGPSFix();
  //enterStandby();
}

void enterStandby() {
  Serial1.end();

  // Power cycle the UARTE0 - Fix or Workaround since UARTE0 is not getting turned Off if RX is enabled
  *(volatile uint32_t *)0x40002FFC = 0;
  *(volatile uint32_t *)0x40002FFC;
  *(volatile uint32_t *)0x40002FFC = 1;

  //pinMode(GPS_Enable_Pin, INPUT_PULLDOWN);
  //pinMode(Pi_Enable_Pin, INPUT_PULLDOWN);

  //nrf_gpio_input_disconnect(GPS_Enable_Pin);
  //nrf_gpio_input_disconnect(Pi_Enable_Pin);

  digitalWrite(GPS_Enable_Pin, LOW);
  digitalWrite(Pi_Enable_Pin, LOW);  
  //nrf_gpio_cfg_default(GPS_Enable_Pin);
  //nrf_gpio_cfg_default(Pi_Enable_Pin);
  nrf_gpio_cfg_default(PIN_WIRE_SDA);
  nrf_gpio_cfg_default(PIN_WIRE_SCL);
  nrf_gpio_cfg_default(PIN_SERIAL1_RX);
  nrf_gpio_cfg_default(PIN_SERIAL1_TX);
}

void updateConnected(void) {
  while (connected) {
    blebas.write(100-i);
    getStatus(buf);
    //for (uint8_t idx=0;idx<sizeof(buf);idx+=20)
    //  bleuart.write(buf[idx], min(sizeof(buf)-idx, 20));
    //buf[0] = 0x00;
    bleuart.write(buf, sizeof(buf));
    delay(1000);
  }
}

void loop() 
{
  for (int d=0;d<60;d++) {
    delay(1000); 
    if (connected) updateConnected();
  }
  
  battV = (analogReadVDD()/1024)*100;
  blebas.write(battV);

  //Serial1.write(buf);
  //Serial1.write(battV);

  updateAdv(i++);
}


// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  connected = true;
  //Serial.print("Connected to ");
  //Serial.println(central_name);
  delay(1000);
  sprintf(buf, "Hello %s!\n", central_name);
  bleuart.write(buf, sizeof(buf));
}


/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  connected = false;
  //Serial.println();
  //Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}


void getGPSFix() {
  //Power on and init GPS
  digitalWrite(GPS_Enable_Pin, HIGH);
  digitalWrite(LED_BUILTIN, LOW);
  
  GPS.begin(0x10);  // The I2C address to use is 0x10
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate  

  timer = millis();
  while(!GPS.fix){
    char c = GPS.read();

    if (GPS.newNMEAreceived()) {
      Serial1.println(GPS.lastNMEA());
      //if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      //  break; // we can fail to parse a sentence in which case we should just wait for another
    }

    //timeout after 5 mins
    if (millis() - timer > (GPS_TIMEOUT)) {
      break;
    }
  }
  
  debugGPS();
  digitalWrite(GPS_Enable_Pin, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
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