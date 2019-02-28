/*
    Name:       tof_sensor.ino
    Created:	28/02/2019 17:09:38
    Author:     SG-DELL\Sylvain
*/

#include <Wire.h>
#include "ToF_longRange.h"

#define PIN_STANDBY 2
#define MIN_DISTANCE 20
#define MAX_DISTANCE 800

ToF_longRange sensor(40, PIN_STANDBY, MIN_DISTANCE, MAX_DISTANCE);

void setup()
{
    sensor.powerON("sensor");
}

void loop()
{
    Serial.println(sensor.getMesure());
    delay(100);
}
