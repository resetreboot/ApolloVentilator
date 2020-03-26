#ifndef VALVEC
#define VALVEC

#include "Arduino.h"
#include "debug.cpp"


class ElectroValve
{
public:
  ElectroValve(uint8_t pin, uint16_t openDelay = 30 , uint16_t closeDelay = 10 , bool invertedLogic = false) :
  _pin(pin),_trueState(!invertedLogic),
  _openDelayMS(openDelay),_closeDelayMS(closeDelay)
  {
    pinMode(_pin, OUTPUT);
    //close(); ///Aqui motherfuckers
  }

  void open(uint8_t percent = 100)
  {
    if(percent > 100) percent = 100;
    if(_openPercent == 0)//Si estabamos cerrados primero cargamos la bobina de la valvula
    {
        magnetize();
        delay(_openDelayMS);
    }
    if(percent == 0)  close();
    _openPercent = percent;
    _deMagnetizedTimeMS  =  (100-percent)/5;
    _magnetizedTimeMS = _deMagnetizedTimeMS * (_openDelayMS/float(_closeDelayMS));
    if(_openPercent == 100) _magnetizedTimeMS = 100;
    _cycleTimeMS = _magnetizedTimeMS + _deMagnetizedTimeMS;
    TRACE("mTime:" + String(_magnetizedTimeMS) + "dTime:" + String(_deMagnetizedTimeMS) + "cyclems:" + String(_cycleTimeMS) );
  }

  void fullOpen()
  {
    close();
    magnetize();
  }

  void close()
  {
    _openPercent = 0;
    demagnetize();
    delay(_closeDelayMS);
  }

  uint8_t openPercent() {return _openPercent;}
  bool    isOpen()      {return _openPercent > 0;}

  void update()
  {
    if(!isOpen()) return;

    uint16_t timeReference = millis()%_cycleTimeMS; //Hallamos en que momento del ciclo de la valvula estamos

    if( isMagnetized() && ( timeReference >= _magnetizedTimeMS ) ) //Si la valvula esta abierta y ya ha pasado el tiempo de estar abierta la cerramos
        demagnetize();
    else if(!isMagnetized() && (timeReference <= _magnetizedTimeMS) ) // Si no está abierta y estamos dentro del tiempo de estar abierta la abrimos
        magnetize();
  }

protected:

  void magnetize()    {digitalWrite(_pin,_trueState);}
  void demagnetize()  {digitalWrite(_pin,!_trueState);}
  bool isMagnetized() {return(digitalRead(_pin)==_trueState);}


  const uint8_t   _pin;
  const bool      _trueState;
  uint8_t         _openPercent;
  const uint16_t  _openDelayMS;
  const uint16_t  _closeDelayMS;
  uint16_t        _cycleTimeMS;
  uint16_t        _magnetizedTimeMS;
  uint16_t        _deMagnetizedTimeMS;
};


#endif
