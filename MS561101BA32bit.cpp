/*
MS5611-01BA.cpp - Interfaces a Measurement Specialities MS5611-01BA with Arduino
See http://www.meas-spec.com/downloads/MS5611-01BA01.pdf for the device datasheet
Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>
Development of this code has been supported by the Department of Computer Science,
Universita' degli Studi di Torino, Italy within the Piemonte Project
http://www.piemonte.di.unito.it/
This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "MS561101BA32bit.h"
#define CONVERSION_TIME 10000 //9100 conversion time in microseconds
MS561101BA32bit::MS561101BA32bit() {
  ;
}
void MS561101BA32bit::init(uint8_t address) {  
  _addr =  address;
  reset(); // reset the device to populate its internal PROM registers
  delay(1000); // some safety time
  readPROM(); // reads the PROM into object variables for later use
}
float MS561101BA32bit::getPressure(uint8_t OSR) {
  // see datasheet page 7 for formulas
  uint32_t rawPress = rawPressure(OSR);
  if(rawPress == NULL) {
    return NULL;
  }
  int32_t dT = getDeltaTemp(OSR);
  if(dT == NULL) {
    return NULL;
  }
  int64_t off  = ((uint32_t)_Ct[1] <<16) + (((int64_t)dT * _Ct[3]) >> 7);
  int64_t sens = ((uint32_t)_Ct[0] <<15) + (((int64_t)dT * _Ct[2]) >> 8);
  return ((( (rawPress * sens ) >> 21) - off) >> 15) / 100.0;
}
float MS561101BA32bit::getTemperature(uint8_t OSR) {
  // see datasheet page 7 for formulas
  int64_t dT = getDeltaTemp(OSR);
  if(dT != 0) {
    return (2000 + ((dT * _Ct[5]) >> 23)) / 100.0;
  }
  else {
    return NULL;
  }
  return NULL;
}
int32_t MS561101BA32bit::getDeltaTemp(uint8_t OSR) {
  uint32_t rawTemp = rawTemperature(OSR);
  if(rawTemp != 0) {
    return (int32_t)(rawTemp - ((uint32_t)_Ct[4] << 8));
  }
  else {
    return NULL;
  }
  return NULL;
}
//TODO: avoid duplicated code between rawPressure and rawTemperature methods
//TODO: possible race condition between readings.. serious headache doing this.. help appreciated!
uint32_t MS561101BA32bit::rawPressure(uint8_t OSR) {
  unsigned long now = micros();
  if(lastPresConv != 0 && (now - lastPresConv) >= CONVERSION_TIME) {
    lastPresConv = 0;
    pressCache = getConversion(MS561101BA_D1 + OSR);
  }
  else {
    if(lastPresConv == 0 && lastTempConv == 0) {
      startConversion(MS561101BA_D1 + OSR);
      lastPresConv = now;
    }
  }
  return pressCache;
}
uint32_t MS561101BA32bit::rawTemperature(uint8_t OSR) {
  unsigned long now = micros();
  if(lastTempConv != 0 && (now - lastTempConv) >= CONVERSION_TIME) {
    lastTempConv = 0;
    tempCache = getConversion(MS561101BA_D2 + OSR);
  }
  else {
    if(lastTempConv == 0 && lastPresConv == 0) { // no conversions in progress
      startConversion(MS561101BA_D2 + OSR);
      lastTempConv = now;
    }
  }
  return tempCache;
}
// see page 11 of the datasheet
void MS561101BA32bit::startConversion(uint8_t command) {
  // initialize pressure conversion
  Wire.beginTransmission(_addr);
  Wire.write(command);
  Wire.endTransmission();
}
uint32_t MS561101BA32bit::getConversion(uint8_t command) {
  union {uint32_t val; uint8_t raw[4]; } conversion = {0};
  // start read sequence
  Wire.beginTransmission(_addr);
  Wire.write(0);
  Wire.endTransmission();
  Wire.beginTransmission(_addr);
  Wire.requestFrom(_addr, (uint8_t) MS561101BA_D1D2_SIZE);
  if(Wire.available()) {
    conversion.raw[2] = Wire.read();
    conversion.raw[1] = Wire.read();
    conversion.raw[0] = Wire.read();
  }
  else {
    conversion.val = -1;
  }
  return conversion.val;
}
/**
 * Reads factory calibration and store it into object variables.
*/
int MS561101BA32bit::readPROM() {
  for (int i=0;i<MS561101BA_PROM_REG_COUNT;i++) {
    Wire.beginTransmission(_addr);
    Wire.write(MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
    Wire.endTransmission();
    Wire.beginTransmission(_addr);
    Wire.requestFrom(_addr, (uint8_t) MS561101BA_PROM_REG_SIZE);
    if(Wire.available()) {
      _Ct[i] = Wire.read() << 8 | Wire.read();
      //DEBUG_PRINT(_C[i]);
    }
    else {
      return -1; // error reading the PROM or communicating with the device
    }
  }
  return 0;
}
/**
 * Send a reset command to the device. With the reset command the device
 * populates its internal registers with the values read from the PROM.
*/
void MS561101BA32bit::reset() {
  Wire.beginTransmission(_addr);
  Wire.write(MS561101BA_RESET);
  Wire.endTransmission();
}
