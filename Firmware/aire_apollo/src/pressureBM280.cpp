#ifndef PRESSUREBME
#define PRESSUREBME

#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "debug.cpp"

class pressureSensorBME280
{
public:
  pressureSensorBME280(char bmeAddr = 0x76) : _addr(bmeAddr)
  {

  }

  bool init()
  {
    _ready = _bme.begin(_addr);
    if(_ready)
    {
      // set max sampling for pressure sensor
      _bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                     Adafruit_BME280::SAMPLING_X1,
                     Adafruit_BME280::SAMPLING_X16,
                     Adafruit_BME280::SAMPLING_X1,
                     Adafruit_BME280::FILTER_OFF,
                     Adafruit_BME280::STANDBY_MS_0_5);
     }
     else
     {
      TRACE("BMP280 ERROR!!!!!");
    }
    return _ready;
  }

  float readHpa()
  {
    return readVal() / 100.0;
  }

  float readMMHg()
  {
    return (readVal() * 0.00750062); //mmhg
  }

protected:

  float readVal()
  {
    if(!_ready) init();

    if(_ready) return _bme.readPressure();
    else return 0;
  }

  Adafruit_BME280 _bme; // I2C
  bool            _ready;
  byte            _addr;
};

#endif
