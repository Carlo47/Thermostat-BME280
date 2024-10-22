/**
 * BME280Sensor.h
 * 
 * Declaration of the class BME280Sensor. It inherits a generic  
 * interface and from Adafruit_BME280.
 * The constructors argument are the I2C address of the sensor and a
 * reference to a data struct to hold the measuremnts. 
 */ 
#pragma once

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "ISensor.h"


class BME280Sensor : public ISensor,  Adafruit_BME280
{  
    public:
        BME280Sensor(uint8_t i2cAddress, SensorData& sensorData) : 
            Adafruit_BME280(), _i2cAddress(i2cAddress), _sData(sensorData)
        {}

        void  setup() override;      // initialize the i2c bus and read the sensor
        void  readSensor() override; // refresh the sensor readings
        float getCelsius() override;
        void  printData() override;  // prints the sensor readings from sensor data struct
        SensorData& getDataReference() override;  // returns a referenc to the sensor data struct which holds the sensor readings
        void setLocalAltitude(float altitude); // provides the local altitude for calculating the local air pressure

    private:
        float _calculateNpLocal();
        float _calculateDewPoint();
        uint8_t _i2cAddress;
        SensorData& _sData;
};
