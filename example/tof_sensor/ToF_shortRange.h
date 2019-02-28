#ifndef _TOF_SHORT_RANGE_H
#define _TOF_SHORT_RANGE_H

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "VL6180X.h"


typedef uint8_t SensorValue;
enum SensorMetadata
{
    SENSOR_DEAD = 0x00,
    SENSOR_NOT_UPDATED = 0x01,
    OBSTACLE_TOO_CLOSE = 0x02,
    NO_OBSTACLE = 0x03
};


class ToF_shortRange
{
public:
    ToF_shortRange()
    {
        name = "";
        i2cAddress = 80;
        pinStandby = 13;
        minRange = 0;
        maxRange = 0;
        isON = false;
        sensorValue = (SensorValue)SENSOR_DEAD;
        initialized = false;
    }

	ToF_shortRange(const char* name, uint8_t id, uint8_t pinStandby, uint16_t minRange, uint16_t maxRange) :
        name(name)
	{
        sensorValue = (SensorValue)SENSOR_DEAD;
		i2cAddress = id;
		this->pinStandby = pinStandby;
        this->minRange = minRange;
        this->maxRange = maxRange;
		standby();
		vlSensor.setTimeout(500);
        initialized = true;
	}

	SensorValue getMesure()
	{
        if (!initialized)
        {
            return (SensorValue)SENSOR_DEAD;
        }
		if (isON)
		{
			uint16_t distance = vlSensor.readRangeContinuousMillimeters();
			if (vlSensor.timeoutOccurred() || vlSensor.last_status != 0)
			{
                sensorValue = (SensorValue)SENSOR_DEAD;
                standby();
                Serial.print("Sensor ");
                Serial.print(name);
                Serial.println(" timed out, RIP");
			}
            else if (distance > maxRange)
            {
                sensorValue = (SensorValue)NO_OBSTACLE;
            }
            else if (distance < minRange)
            {
                sensorValue = (SensorValue)OBSTACLE_TOO_CLOSE;
            }
            else
            {
                sensorValue = (SensorValue)(constrain(distance, 0, UINT8_MAX));
            }
		}
		else
		{
            sensorValue = (SensorValue)SENSOR_DEAD;
		}
		return sensorValue;
	}

	void standby()
	{
        if (initialized)
        {
		    pinMode(pinStandby, OUTPUT);
		    digitalWrite(pinStandby, LOW);
		    isON = false;
        }
	}

	int powerON()
	{
        if (!initialized)
        {
            return -1;
        }
		Serial.print("PowerOn ToF ");
		Serial.print(name);
		Serial.print("...");
		pinMode(pinStandby, INPUT);
		delay(50);
        if (vlSensor.init())
        {
            vlSensor.configureDefault();
            vlSensor.setAddress(i2cAddress);

            vlSensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 12);

            vlSensor.stopContinuous();
            delay(100);
            vlSensor.startRangeContinuous(20);
            isON = true;
            Serial.println("OK");
            return 0;
        }
        else
        {
            standby();
            Serial.println("NOT OK");
            return -1;
        }
	}

private:
    bool initialized;
	uint8_t i2cAddress, pinStandby;
    SensorValue sensorValue;
    uint16_t minRange;  // [mm] Toute valeur strictement inférieure est considérée comme un obstacle trop proche
    uint16_t maxRange;  // [mm] Toute valeur strictement supérieure est considérée comme une absence d'obstacle
	bool isON;
	VL6180X vlSensor;

public:
    const char* name;
};

#endif

