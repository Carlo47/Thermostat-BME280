#pragma once

// SensorData stores the values read by the sensor 
// and calculated on the basis of the measurements.
// This data structure depends on the capabilities of 
// the sensor and must be adapted accordingly.
using SensorData = struct sDat
{
    float tCelsius;         // sensor reading
    float tFahrenheit;      // calculated
    float tKelvin;          // calculated
    float relHumidity;      // sensor reading
    float pLocal;           // sensor reading
    float dewPoint;         // calculated dewpoint for e.g. temperature=20°C and humidity=40.0%
    float altLocal = 453.0; // user defined local altitude
    float npLocal;          // calculated normal air pressure for the local altitude
    const float pSeaLevel   = 1013.25; // standard air pressure at sealevel
};
