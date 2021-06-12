#ifndef _BLUETOOTH_H 
#define _BLUETOOTH_H

#include <bluefruit.h>
#include "nrf_usbd.h"
#include "nrf_nvic.h"
#include <TimeLib.h>

extern BLEUart bleuart; // uart over ble

extern uint8_t mac[6];
extern volatile bool connected;

void setupAdv(void);
void setupBluetooth(char*);
void connect_callback(uint16_t);
void disconnect_callback(uint16_t, uint8_t);

void updateAdv(char*);
void updateBatt(uint8_t);
void writeBLE(char* buf, bool newline=true, bool flush=true, uint32_t buf_len=0);

#endif