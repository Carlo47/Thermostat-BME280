/**
 * Program      A thermostat with a sensor interface wich enables the use of different sensors
 * 
 * Author       2024-10-12 Charles Geiser
 * 
 * Purpose      The program shows how a Bosch BME280 temperature, humidity and air pressure sensor 
 *              is queried and how these measurements are used to control an output pin which switches
 *              on and off a relais or whatever you want. 
 * 
 *              From the measured temperature and humidity we calculate the dewpoint. This is the temperature 
 *              at which the air becomes saturated and it "starts to rain". 
 *              When a local altitude above sealevel is supplied we calculate the normal atmospheric pressure
 *              for this hight.
 *   
 *              The sensor updates the measured values every msRefresh milliseconds and the user supplied
 *              method processData() is called. 
 * 
 * Board        ESP32 DoIt DevKit V1 
 * 
 * Wiring                                                                                  Solid State Relay
 *                                                                                            .---------.      
 *                                                                        5V(Vin of ESP32) ---|+       ~|------------- L 
 *                         -----.                   .----------.                              |   SSR   |    .-.        ~230V 
 *              BME280 Sensor   |-- Vcc --> 3.3 V --|  ESP32   |-- 2 Heartbeat           .----|-       ~|---( L )----- N
 *              I2C             |-- GND --> GND   --|          |                         |    `---------´    `-´
 *              Temperature     |-- SCL --> 22    --|          |                     |¦--'                  Load
 *              rel Humidity    |-- SDA --> 21    --|          |                     |¦<-.  2N7000   
 *              air pressure    |-- CSB --> nc      |          |--  4 --> heating ---|¦--|  N-CH MOSFET        
 *                              |-- SDO --> nc      |          |                         | 
 *                         -----´                   ´----------´                   GND --+--  
 * 
 *              The solid state relay should switch on at a voltage of 3V, but direct control with an output of the ESP32 
 *              does not work. Therefore the relay is connected to Vin (5 V) and switched on with the N-Channel MOSFET against GND 
 *                .---.
 *              /_____/|
 *              | 2N  ||
 *              | 7000||
 *              |_____|/
 *               | | |
 *               S G D
 * 
 *  
 * Installing   1. Erase flash memory
 *              2. Uploade flash file system (content of data folder)
 *              3. Compile and upload program code
 * 
 * Remarks      1 Pa   = 7.50064 E-3 mmHg
 *              1 hPa  = 0.75 mmHg
 *              1 mmHg = 133.322 Pa
 * 
 *              kapa = 1.235
 *              K0 = kapa / (kapa -1) = 5.255
 *              1/K0 = 0.1903
 *              T0 = 288.15 °K = 15 °C (According an international convention this temperatur is used as reference)
 *              gradT = 0.0065 K/m (Temperature gradient Kelvin / Meter
 *              H0 = T0 / gradT = 44330 m
 * 
 *              Sea level air pressure at given local altitude:
 *              Used to calibrate the barometer to a known height
 *              P0 = pLocal / ((1 - hLocal/H0) ^ K0)
 *              P0 = 1013.25 hPa (nach int.Konvention, P0 = pSeaLevel))
 * 
 *              Local altitude for a given local air pressure:
 *              hLocal = H0 * (1 - (pLocal/P0) ^ 1/K0)
 * 
 *              Local air pressure at a given local altitude (international barometric hight formula):
 *              pLocal = P0 * (1 - h/H0) ^ K0
 *                        
 * 
 * References   
 */

#include <Arduino.h>
#include "Thermostat.h"

#define PIN_THERMOSTAT  GPIO_NUM_4   // pin to turn on/off the heating
#define PIN_HEARTBEAT   LED_BUILTIN  // indicates normal operation with 1 beat/sec or error state with 5 beats / sec
#define PIN_BOILER      GPIO_NUM_18  // pin to turn on/off the boiler
#define BME280_I2C_ADDR 0x76


extern void heartbeat(uint8_t pin, uint8_t nBeats, uint8_t t, uint8_t duty);
extern void doMenu();
extern void setLowerLimit();
extern void setUpperLimit();
extern void setTempDelta();
extern void setAltitude();
extern void setInterval();
extern void toggleThermostat();
extern void showValues();
extern void showMenu();

bool heatingIsOn = false; 

SensorData   sensorData; // holds measured and calculated sensor values (see Isensor.h)
BME280Sensor sensor(BME280_I2C_ADDR, sensorData); // sensor used for thermostat

// Forward declaration of the handler functions for the thermostat
void processData();
void turnHeatingOn();
void turnHeatingOff();

Thermostat thermostat(sensor, processData, turnHeatingOn, turnHeatingOff);
//const BME280Data& sDataRef = myThermostat.getSensorDataRef();  // const makes access by reference readonly

// Called as onDataReady() when refresh intervall expires
void processData()
{
  sensor.readSensor();
  //sensor.printData();
  //thermostat.printSettings(); 
  showValues();
}

// Called as onLowTemp() when the temperature falls below the set limit
void turnHeatingOn()
{
  if (! heatingIsOn)
  {
    log_i("===> switch on heating, it is: %s", heatingIsOn ? "on" : "off");
    digitalWrite(PIN_THERMOSTAT, HIGH);
    heatingIsOn = true;
  }
}

// Called as onHighTemp() when the temperature rises above the set limit
void turnHeatingOff()
{
  if (heatingIsOn)
  {
  log_i("===> switch off heating, it is: %s", heatingIsOn ? "on" : "off");
  digitalWrite(PIN_THERMOSTAT, LOW);
  heatingIsOn = false;
  }
}


void initOutputPins()
{
  pinMode(PIN_HEARTBEAT, OUTPUT);
  pinMode(PIN_THERMOSTAT, OUTPUT);
  pinMode(PIN_BOILER,     OUTPUT);
  digitalWrite(PIN_THERMOSTAT, LOW);
  digitalWrite(PIN_BOILER, LOW);
  log_i("===> done");  
}


/**
 * Checks whether a temperature sensor is connected and stops if none is found
 */
void initThermostat()
{
  thermostat.setup();
  thermostat.enable();
  log_i("===> done");
}


void setup() 
{
  Serial.begin(115200);

  sensor.setLocalAltitude(410.0);
  initOutputPins(); 
  initThermostat();
  showMenu();
}

void loop() 
{
  if(Serial.available()) doMenu();
  thermostat.loop();
  heartbeat(PIN_HEARTBEAT, 1, 1, 5);
}
