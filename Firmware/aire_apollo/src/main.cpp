/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.
 ***************************************************************************/
#define DEBUG
#include "debug.cpp"

#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MsTimer2.h>


#include "pressureBM280.cpp"
#include "ElectroValve.cpp"
#include "FlowSensorInt.cpp"



#define INTAKE_EV_PIN    10 //ElectroValvula - Entrada
#define EXIT_EV_PIN      8  //ElectroValvula - Salida

#define INTAKE_FLOW_PIN  4  //Sensor de Flujo - Entrada
#define EXIT_FLOW_PIN    5  //Sendor de Flujo - Salida

#define LOG_INTERVAL    10 //milliseconds

//#define PRESSURE_SENSOR_PIN      ??
#define BME280_ADDR                0x76

#define SEALEVELPRESSURE_HPA (1013.25)

#define INSPIRATION_TIME 1000
#define ESPIRATION_TIME 4000
#define INSPIRATION_THRESHOLD 10  //Descenso en la presion que marca el inicio de la respiracion


float volmax;
float presmax;
float o2insp;
float limitO2;
float co2esp;
float limitCO2;
float volumeentry;
float volumeexit;
int ppeak = 23;
int pplat = 18;
int peep = 5;

// Parameters
int weight = 70; // kg
int volc = weight * 6; // weight * 6u8 (mL) Volumen tidal
int bpm = 10; // breaths per minute
int voli = volc * bpm;
//int timep; // good to have, pause
unsigned long lastInspirationStart = 0; // (s)
unsigned long lastEspirationStart = 0; // (s)
uint16_t inspirationTimeout = 0;
uint16_t inspTime = 0;
uint16_t espTime = 0;



//OJO a los contadores que se desbordan en algun momento!!!!
// Gestion rtc? / deteccion de desbordamiento?

//uint8_t power;
char          logBuffer[50];
//unsigned long lastLogTime=0;

enum respiratorStatus
{
  RESPIRATOR_PAUSED,
  WAIT_FOR_INSPIRATION,
  INSPIRATION_CICLE,
  ESPIRATION_CICLE,
};

respiratorStatus status = WAIT_FOR_INSPIRATION;

pressureSensorBME280  pSensor(BME280_ADDR);
ElectroValve          IntakeValve(INTAKE_EV_PIN);
ElectroValve          ExitValve(EXIT_EV_PIN);
ElectroValve          hackValve(9);
FlowSensorInt         fSensor(1000,250);

void flowInterrupt()
{
  fSensor.pulse();
}




// BEGIN Sensors and actuators


// Get metric from entry flow mass sensor
float getMetricVolumeEntry(){
  float v;
  //v=analogRead(ENTRY_FLOW_PIN)*0.0049F;

  // Simulate sensor, if valve is open just return 52 l/m
  // to see if it helps displaying graphs

  if(status==INSPIRATION_CICLE)
    v = 52.00F;
  else
    v = 5.00F;

 return v;

}

// Get metric from exit flow mass sensor
float getMetricVolumeExit(){
 float v = analogRead(EXIT_FLOW_PIN)*0.0049F;
 return v;
}


// END sensors and actuator

int getMetricPpeak(){return 22;}
int getMetricPplat(){return 22;}

int calculateResistance (int ppeak, int pplat) {
	return ppeak - pplat;
}

int getMetricPeep(){return 22;}

void checkLeak(float volEntry, float volExit){}
float getMetricVolMax() {return 22;}
float getMetricPresMax(){return 22;}

int calculateCompliance (int pplat, int peep) {
	return pplat - peep;
}


void logData()
{
    if( millis()% LOG_INTERVAL == 0)
    {
      String pVal    = String(pSensor.readMMHg());
      String fInVal  = String(fSensor.getInstantFlow());
      String fOutVal = String(fSensor.getFlow());
      String inValveVal = String(IntakeValve.openPercent());
      String outValveVal = String(ExitValve.openPercent());
      String result = "DATA:" + pVal + "," + fInVal + "," + fOutVal + "," + inValveVal + "," + outValveVal;
      Serial.println(result);
    }
}

void setBPM(uint8_t CiclesPerMinute)
{
  inspTime            = 60000.0/float(bpm)*0.25;
  espTime             = 60000.0/float(bpm)*0.75;
  TRACE("BPM set: iTime:"+String(inspTime)+", eTime:"+String(espTime));
}


void pollSensors()
{
  IntakeValve.update();
  ExitValve.update();
  hackValve.update();
  fSensor.update();
}

void setup() {
    Serial.begin(115200);
    while(!Serial);    // time to get serial running
    TRACE("INIT...!");
    yield();
    IntakeValve.setup();
    ExitValve.setup();
    pinMode(3, INPUT_PULLUP);
    setBPM(8);
    //ExitValve.open();

//ISRs
    MsTimer2::set(1, pollSensors); // 500ms period
    MsTimer2::start();
    attachInterrupt(digitalPinToInterrupt(3), flowInterrupt, RISING);
}

void beginInspiration()
{
  lastInspirationStart = millis();
  status = INSPIRATION_CICLE;
  hackValve.close();
  IntakeValve.open();
  ExitValve.close();
}

void beginEspiration()
{
  lastEspirationStart = millis();
  status = ESPIRATION_CICLE;
  IntakeValve.close();
  hackValve.open();
  ExitValve.open();
}


bool checkForPatientInspiration()
{
  float pressureReference = 950;
//  if ( (pressureReference - getMetricPressureEntry()) > INSPIRATION_THRESHOLD )
    //return true;
//  else
    return false;
}


// Send alarm to GUI by Serial
void alarm(const char* value)
{
  // Raise alarm localy may be with sound?
  // local sound alarm?
  Serial.println("ALARM: " + String(value));
}


void loop() {
/*
//  Serial.println("LOOPPP!!!");
// Control del ciclo de respiracion
  if(status == RESPIRATOR_PAUSED)
  {
    TRACE("PAUSED...");
  }
  else if(status == WAIT_FOR_INSPIRATION)
  {
    if(checkForPatientInspiration())
    {
      TRACE("Insp DETECTED!");
      beginInspiration();
    }
    else if(millis() - (lastInspirationStart + inspTime + espTime) >= inspirationTimeout)
    {
      TRACE("FORCING Insp");
      beginInspiration();
    }
  }
  else if(status == INSPIRATION_CICLE)
  {
    if(fSensor.getFlow() >= 150)
    {
      fSensor.resetFlow();
      TRACE("BeginEspiration");
      beginEspiration();
    }
  }
  else if(status == ESPIRATION_CICLE)
  {
    if(millis() - lastEspirationStart >= espTime)
    {
      status = WAIT_FOR_INSPIRATION;
      TRACE("Cicle Done! :), Wait for inspiration");
    }
  }

//Comprobacion de alarmas

	ppeak = getMetricPpeak();
	pplat = getMetricPplat();

  calculateResistance(ppeak, pplat);

  if (ppeak > 40)
  {
    alarm("PRESSURE ALERT");
	}

  int peep = getMetricPeep();
//  float volExit = getMetricVolumeExit();
//	checkLeak(volc, volExit);
	calculateCompliance(pplat, peep);
// envio de datos
  logData();
*/
//hackValve.open();
delay(5000);
Serial.println("Start:");
fSensor.resetFlow();
//hackValve.close();
IntakeValve.open(10);
delay(5000);
IntakeValve.close();
//hackValve.open();
Serial.println("Flow:"+String(fSensor.getFlow()));
}
