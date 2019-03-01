/*
    Name:       tof_sensor.ino
    Created:	28/02/2019 17:09:38
    Author:     SG-DELL\Sylvain
*/

#include <Wire.h>
#include "ToF_sensor.h"

#define PIN_STANDBY 2
#define MIN_DISTANCE 20
#define MAX_DISTANCE 800

ToF_shortRange sensor1;
ToF_shortRange sensor2(42, 13);
ToF_shortRange sensor3(43, 14, 20, 150, "sensor", &Serial);

ToF_longRange_med<3> sensor4;
ToF_longRange_med<3> sensor5(42, 13);
ToF_longRange_med<3> sensor6(43, 14, 20, 150, "sensor", &Serial);

void setup()
{
    sensor1.powerON();
    sensor2.powerON();
    sensor3.powerON();
    sensor4.powerON();
    sensor5.powerON();
    sensor6.powerON();
}

void loop()
{
    Serial.println(sensor1.getMeasure());
    Serial.println(sensor2.getMeasure());
    Serial.println(sensor3.getMeasure());
    Serial.println(sensor4.getMeasure());
    Serial.println(sensor5.getMeasure());
    Serial.println(sensor6.getMeasure());
    delay(100);
}
