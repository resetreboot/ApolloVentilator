#ifndef VALVEC
#define VALVEC

#include "Arduino.h"

//Valve cicle for 10hz 100%
//|open    opened      close
//|25ms |    50ms   | 25ms   |
//|-----|-----------|--------|
//Valve cicle for 10hz 50%
//|open    opened      close
//|25ms |    25ms   | 25ms   |
//|-----|-----------|--------|

class ElectroValve
{
public:
  ElectroValve(uint8_t pin, uint16_t hz = 10, bool invertedLogic = false) :
  _pin(pin),_trueState(!invertedLogic),_valveCycleMs(1000/hz),
  _openDelayMS(_valveCycleMs/4),_closeDelayMS(_valveCycleMs/4)
  {
    pinMode(_pin, OUTPUT);
  }

  void open(uint8_t percent = 100)
  {
    if(percent > 100) percent = 100;
    if(percent == 0)  closeEv();
    _openPercent = percent;
    _openTimeMS  = (_valveCycleMs - _openDelayMS - _closeDelayMS) * percent / 100;
  }

  void close()
  {
    _openPercent = 0;
    closeEv();
  }

  uint8_t openPercent() {return _openPercent;}
  bool    isOpen()      {return _openPercent > 0;}

  void update()
  {
    if(!isOpen()) return;

    uint16_t timeReference = millis()%_valveCycleMs; //Hallamos en que momento del ciclo de la valvula estamos

    if( (evStatus()) && (timeReference >= (_openDelayMS + _openTimeMS) ) ) //Si la valvula esta abierta y ya ha pasado el tiempo de estar abierta la cerramos
        closeEv();
    else if(!evStatus() && (timeReference <= (_openDelayMS+_openTimeMS) ) ) // Si no está abierta y estamos dentro del tiempo de estar abierta la abrimos
        openEV();
  }

protected:

  void openEV()   {digitalWrite(_pin,_trueState);}
  void closeEv()  {digitalWrite(_pin,!_trueState);}
  bool evStatus() {return(digitalRead(_pin)==_trueState);}


  const uint8_t   _pin;
  const uint16_t  _valveCycleMs;
  const bool      _trueState;
  uint8_t         _openPercent;
  uint16_t        _openDelayMS;
  uint16_t        _closeDelayMS;
  uint16_t        _openTimeMS;
};


#endif
