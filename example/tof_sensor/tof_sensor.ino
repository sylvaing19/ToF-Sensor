/*
    Name:       tof_sensor.ino
    Created:	28/02/2019 17:09:38
    Author:     SG-DELL\Sylvain
*/

#include <Wire.h>
#include "ToF_sensor.h"

ToF_shortRange sensor1; // Uninitialized sensor (will do nothing)
ToF_shortRange sensor2(42, 13); // Default short range sensor with address 42, on pin 13
ToF_shortRange sensor3(43, 14, 20, 150, "sensor", &Serial); // Named sensor with a Stream to output logs
ToF_longRange sensor4(44, 15); // Default long range sensor
ToF_longRange_med<3> sensor5(45, 16); // Default long range sensor with median filter

void setup()
{
	Wire.begin();
    sensor1.powerON();
    sensor2.powerON();
    sensor3.powerON();
    sensor4.powerON();
    sensor5.powerON();
}

void loop()
{
    Serial.println(sensor1.getMeasure());
    Serial.println(sensor2.getMeasure());
    Serial.println(sensor3.getMeasure());
    Serial.println(sensor4.getMeasure());
    Serial.println(sensor5.getMeasure());
    delay(100);
}

