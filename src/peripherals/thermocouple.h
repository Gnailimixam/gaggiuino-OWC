/* 09:32 15/03/2023 - change triggering comment */
#ifndef THERMOCOUPLE_H
#define THERMOCOUPLE_H

#include "pindef.h"

#if defined SINGLE_BOARD
#include <Adafruit_MAX31855.h>
SPIClass thermoSPI(thermoDI, thermoDO, thermoCLK);
Adafruit_MAX31855 thermocouple(thermoCS, &thermoSPI);
static inline void thermocoupleInit(void) {
  thermocouple.begin();
}

static inline float thermocoupleRead(void) {
  return thermocouple.readCelsius();
}
#endif

#if defined PT100
#include <Adafruit_MAX31865.h>
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermocouple = Adafruit_MAX31865(thermoCS, thermoDI, thermoDO, thermoCLK);
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0
static inline void thermocoupleInit(void) {
  thermocouple.begin(MAX31865_2WIRE);
}

static inline float thermocoupleRead(void) {
  return thermocouple.temperature(RNOMINAL, RREF);
}

#else
#include <max6675.h>
SPIClass thermoSPI(thermoDI, thermoDO, thermoCLK);
MAX6675 thermocouple(thermoCS, &thermoSPI);
static inline void thermocoupleInit(void) {
  thermocouple.begin();
}

static inline float thermocoupleRead(void) {
  return thermocouple.readCelsius();
}
#endif



#endif
