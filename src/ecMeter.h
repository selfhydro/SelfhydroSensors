#include <Arduino.h>
#include <OneWire.h>

#include "analogDigitalConverter.h"

#define StartConvert 0
#define ReadTemperature 1
#define SENSOR_CHANNEL 0

#define NumberOfReadings 20
#define ECSensorPin 1

class ECMeter
{
    public:
        ECMeter() {};
        void Setup();
        float GetReading(float temperature);

    private:
        AnalogDigitalConverter adc;
        unsigned int readings[NumberOfReadings];     
        byte index = 0;                  
        unsigned long AnalogValueTotal = 0;                 
        unsigned int AnalogAverage = 0,averageVoltage=0;               
        float temperature,ECcurrent;

};