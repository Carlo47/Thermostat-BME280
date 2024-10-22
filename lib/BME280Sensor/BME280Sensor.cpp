/**
 * Class        BME280Sensor
 * Author       2022-01-24 Charles Geiser
 * 
 * Purpose      A class to measure temperature, relative humidity and air pressure
 *              by means of a BME280 sensor. It also calculates the dewpoint and 
 *              the local standard air pressure when local altitude is known.
 *              If the local altitude is known, the sensor can be calibrated to 
 *              this height and used as an altimeter.  
 * Remarks
 * References   
 */ 
#include "BME280sensor.h"

/** Initializes the sensor and reads the measurement data 
 * or gives an error message if the sensor was not found 
 * and stops the program
*/
void BME280Sensor::setup()
{
    int tries = 5;
    while (! begin(_i2cAddress) && (tries > 0))
    {
        delay(1000);
        tries--;
    }
    if(tries <= 0)
    {
        Serial.printf("BME280 not found at i2c address %#x", _i2cAddress);
        while(true) delay(10000UL);
    }
    readSensor();
    log_i("==> done");
}


/**
 * Read the sensor and calculate local normal pressure
 * and dew point
 */
void BME280Sensor::readSensor()
{
    _sData.pLocal      = readPressure() / 100.0;
    _sData.relHumidity = readHumidity();
    _sData.tCelsius    = readTemperature();
    _sData.tFahrenheit = _sData.tCelsius * 9.0 / 5.0 + 32.0;
    _sData.tKelvin     = _sData.tCelsius + 273.15;
    _sData.npLocal     = _calculateNpLocal();
    _sData.dewPoint    = _calculateDewPoint();      
}


/**
 * Set the local altitude in meter above sea level 
 */
void BME280Sensor::setLocalAltitude(float altitude)
{
    _sData.altLocal = altitude;
}

/**
 * Calculate dew point in °C from temperature and humidity
 */
float BME280Sensor::_calculateDewPoint() 
{
    float k;
    k = log(_sData.relHumidity/100) + (17.62 * _sData.tCelsius) / (243.12 + _sData.tCelsius);
    return 243.12 * k / (17.62 - k);
}

/**
 * Calculate local normal air pressure at given altitude
 * 
 * kapa = 1.235
 * K0 = kapa / (kapa -1) = 5.255
 * T0 = 288.15 °K = 15 °C (According an international convention this temperatur is used as reference)
 * gradT = 0.0065 K/m (Temperature gradient Kelvin / Meter
 * P0 = 1013.25 hPa (accord. int. convention, P0 = pSeaLevel))
 * H0 = T0 / gradT = 44330 m
 * pLocal = P0 * (1 - h/H0) ^ K0
 */ 
float BME280Sensor::_calculateNpLocal()
{
    _sData.npLocal = 1013.25 * pow( (1.0 - _sData.altLocal / 44330.0), 5.255);
    return _sData.npLocal;
}


float BME280Sensor::getCelsius()
{
    return _sData.tCelsius;
}


/**
 * Returns a reference to the sensor data struct
 */
SensorData&  BME280Sensor::getDataReference()
{
    return(_sData);
}


void BME280Sensor::printData()
{
    readSensor();

    Serial.printf(R"(---   Sensor Readings   ---
Tc               %6.1f °C
Tf               %6.1f °F
Tk               %6.1f °K
Dewpoint         %6.1f °C
Humidity         %6.1f %%rH
Local pressure   %6.1f hPa
Local altitude   %6.1f m.a.s.l.
nP at altitude   %6.1f hPa
nP at sea level  %6.1f hPa

)", _sData.tCelsius, _sData.tFahrenheit, _sData.tKelvin, _sData.dewPoint, _sData.relHumidity,  
    _sData.pLocal, _sData.altLocal, _sData.npLocal,  _sData.pSeaLevel);
}
