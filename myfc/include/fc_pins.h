#ifndef PINS_H
#define PINS_H

#include <stdint.h>

// ================================================================
// ESP32 classic pin map for custom drone flight controller
// Keep all board wiring here. No random pin numbers in main.cpp.
// ================================================================

// ---------------- SPI: MPU6500 ----------------
#define FC_PIN_SPI_SCK      18
#define FC_PIN_SPI_MISO     19
#define FC_PIN_SPI_MOSI     23
#define FC_PIN_SPI_CS       5

// ---------------- I2C: QMC5883 / future baro ----------------
#define FC_PIN_I2C_SDA      22
#define FC_PIN_I2C_SCL      21
// #define PIN_QMC_DRDY        36   // Doesnt work, ignore, dont DRDY interrupt

// ---------------- RX PWM input pins ----------------
// This preserves your current main.cpp mapping:
// CH1 pin 27 -> aileron
// CH2 pin 25 -> throttle
// CH3 pin 13 -> arm toggle
// CH4 pin 26 -> elevator
// CH5 pin 14 -> dial/aux
// CH6 pin 12 -> rudder
#define FC_PIN_RX_AILERON   27
#define FC_PIN_RX_THROTTLE  25
#define FC_PIN_RX_ARM       13
#define FC_PIN_RX_ELEVATOR  26
#define FC_PIN_RX_AUX       12
#define FC_PIN_RX_RUDDER    14

// ---------------- ESC output pins ----------------
#define FC_PIN_ESC_FRONT_RIGHT   32
#define FC_PIN_ESC_FRONT_LEFT    33
#define FC_PIN_ESC_BACK_LEFT      4
#define FC_PIN_ESC_BACK_RIGHT    16

// ----------------- GPS UART pins -----------------
#define FC_GPS_UART_RX_PIN       34     // TX of GPS
#define FC_GPS_UART_TX_PIN       17     // RX of GPS

// ---------------- Optional status pin ----------------
#define FC_PIN_LED_STATUS        2

#define FC_PIN_BATTERY_VOLTAGE  35

#endif // PINS_H
