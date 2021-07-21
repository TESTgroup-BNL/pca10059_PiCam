#include <bluetooth.h>

BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEBas  blebas;  // battery
BLEUart bleuart; // uart over ble

uint8_t mac[6];
volatile bool connected = false;
volatile uint16_t data_mtu = 20;

void setupBluetooth(char* name) {
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.autoConnLed(false);
  Bluefruit.setTxPower(0);
 
  Bluefruit.setName(name);
  Bluefruit.getAddr(mac);

  setupAdv();

  //These probably aren't needed, but just in case
  sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
  sd_power_dcdc0_mode_set(NRF_POWER_DCDC_ENABLE);
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);

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
  bleuart.bufferTXD(true);

  blebas.begin();
  blebas.write(100);
}

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

void updateAdv(char* buf) {
  Bluefruit.ScanResponse.clearData();
  Bluefruit.setName(buf);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.start(0);
}

void updateBatt(uint8_t batt) {
    blebas.write(batt);
    blebas.notify(batt);
}

void writeBLE(char* buf, bool newline, bool flush, uint32_t buf_len) {
    if (connected) {
      if (buf_len > 0) bleuart.write(buf, buf_len);
      else {
        for (int i=0;i<=strlen(buf)-1;i+=data_mtu)
          bleuart.write(&buf[i], min(strlen(buf)-i,data_mtu));
      }
      if (newline) bleuart.write('\n');
      if (flush) bleuart.flushTXD();
    }
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));
  connection->requestDataLengthUpdate();
  connection->requestMtuExchange(247);    //Request max MTU
  data_mtu = connection->getMtu() - 3;
  connected = true;
  //Serial.print("Connected to ");
  //Serial.println(central_name);
  //delay(1000);

/*
  char connbuf[64];
  sprintf(connbuf, "Hello %s!, data_mtu=%d\n", central_name, data_mtu);
  bleuart.write(connbuf, sizeof(connbuf));
  bleuart.flushTXD();*/
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