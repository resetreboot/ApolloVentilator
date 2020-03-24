#ifndef PRESSUREC
#define PRESSUREC

#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "debug.cpp"

class pressureSensorBME280
{
public:
  pressureSensorBME280(byte bmeAddr = 0x76)
  {
    while(!_bme.begin(bmeAddr)) {
        TRACE("BME280 sensor not found!!!");
    }

    // set max sampling for pressure sensor
    _bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                   Adafruit_BME280::SAMPLING_X1,
                   Adafruit_BME280::SAMPLING_X16,
                   Adafruit_BME280::SAMPLING_X1,
                   Adafruit_BME280::FILTER_OFF,
                   Adafruit_BME280::STANDBY_MS_0_5);
  }

  float readHpa()
  {

    return _bme.readPressure() / 100.0;
  }

  float readMMHg()
  {
    return _bme.readPressure() / 0.00750062; //mmhg
  }

protected:
  Adafruit_BME280 _bme; // I2C
};

#endif
