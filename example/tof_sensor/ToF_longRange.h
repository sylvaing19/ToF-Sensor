#ifndef _TOF_LONGRANGE_h
#define _TOF_LONGRANGE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "VL53L0X.h"
#include "ToF_shortRange.h"
#include "Median.h"


class ToF_longRange
{
public:
	ToF_longRange(uint8_t id, uint8_t pinStandby, uint16_t minDist = 30, uint16_t maxDist = 700)
	{
		i2cAddress = id;
		this->pinStandby = pinStandby;
        minRange = minDist;
        maxRange = maxDist;
        sensorValue = (int32_t)SENSOR_DEAD;
		standby();
		vlSensor.setTimeout(500);
	}

    int32_t getMesure()
	{
		if (isON)
		{
            medianDistance.add(vlSensor.readRangeContinuousMillimeters());
			uint16_t distance = medianDistance.value();
			if (vlSensor.timeoutOccurred())
			{
				sensorValue = (int32_t)SENSOR_DEAD;
			}
            else if (distance < minRange)
            {
                sensorValue = (int32_t)OBSTACLE_TOO_CLOSE;
            }
            else if (distance > maxRange)
            {
                sensorValue = (int32_t)NO_OBSTACLE;
            }
            else
            {
                sensorValue = (int32_t)distance;
            }
		}
		else
		{
			sensorValue = (int32_t)SENSOR_DEAD;
		}

		return sensorValue; /* en mm */
	}

	void standby()
	{
		pinMode(pinStandby, OUTPUT);
		digitalWrite(pinStandby, LOW);
		isON = false;
	}

	void powerON(const char * name = "")
	{
		Serial.print("PowerOn ToF ");
		Serial.print(name);
		Serial.print("...");
		pinMode(pinStandby, INPUT);
		delay(50);
		if (vlSensor.init())
		{
			vlSensor.setAddress(i2cAddress);

			vlSensor.stopContinuous();
			delay(50);
			vlSensor.startContinuous();
			isON = true;
			Serial.println("OK");
		}
		else
		{
			Serial.println("FAILED");
		}
	}

private:
	uint8_t i2cAddress, pinStandby;
	int32_t sensorValue;
    uint16_t maxRange;
    uint16_t minRange;
	bool isON;
	VL53L0X vlSensor;
    Median<uint16_t, 3> medianDistance;
};


#endif
