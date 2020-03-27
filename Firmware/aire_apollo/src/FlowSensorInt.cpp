#ifndef FLOWIC
#define FLOWIC

#include "Arduino.h"
#include "debug.cpp"

//ojo! para que esta clase funcione correctamente hay que llamar a pulse() desde la interrupcion del pin del sensor.
// Tambien hay que llamar a la funcion update con una interrupcion cada 1ms.

class FlowSensorInt
{
  public:
    FlowSensorInt(uint16_t pulses_per_liter = 100,uint8_t sampling_ms = 25):
    _pulsesPerLiter(pulses_per_liter), _samplingMS(sampling_ms)
    {

    }

    void pulse()
    {
      _pulsesSinceLastSample++;
      _pulseCounter++;
    }

    void update()
    {
      if( millis() % _samplingMS==0 )
      {
        _instantFlow = (_pulsesSinceLastSample / float(_pulsesPerLiter) ) * (60*(1000/float(_samplingMS))) * 1000.0 ;
        _pulsesSinceLastSample = 0;
      }
    }

    float getInstantFlow()
    {
      return _instantFlow;
    }

    float getFlow()
    {
      return _pulseCounter / float(_pulsesPerLiter) * 1000.0;
    }

    void resetFlow()
    {
      _pulseCounter = 0;
    }

  protected:
      uint16_t      _pulsesPerLiter;
      uint8_t       _samplingMS;
      uint16_t      _pulsesSinceLastSample;
      uint16_t      _instantFlow;
      unsigned long _pulseCounter;
};

#endif
