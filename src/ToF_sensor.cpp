#include "ToF_sensor.h"

#define TOF_SENSOR_I2C_TIMEOUT          50  // ms
#define TOF_SENSOR_I2C_TIMEOUT_STARTUP  200 // ms
#define TOF_SENSOR_INIT_DELAY           50  // ms
#define TOF_SENSOR_SHORT_RANGE_UPDATE_PERIOD    20  // ms

ToF_sensor::ToF_sensor()
{
    name = "";
    i2cAddress = 0;
    pinStandby = 0;
    minRange = 0;
    maxRange = 0;
    isON = false;
    debug_stream = nullptr;
    fully_defined = false;
}

ToF_sensor::ToF_sensor(uint8_t address, uint8_t pinStandby, int32_t minRange,
    int32_t maxRange, const char* name, Stream *debug) :
        i2cAddress(address), pinStandby(pinStandby), minRange(minRange),
        maxRange(maxRange), debug_stream(debug), name(name)
{
    isON = false;
    fully_defined = true;
    standby();
}

SensorValue ToF_sensor::getMeasure()
{
    SensorValue sensorValue = (SensorValue)SENSOR_DEAD;
    if (!fully_defined || !isON)
    {
        return sensorValue;
    }

    int32_t distance = 0;
    int ret = measureDistance(distance);

    if (ret != EXIT_SUCCESS)
    {
        sensorValue = (SensorValue)SENSOR_DEAD;
        standby();
        print("Sensor ");
        print(name);
        print(" timed out, RIP\n");
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
        sensorValue = (SensorValue)distance;
    }

    return sensorValue;
}

void ToF_sensor::standby()
{
    if (fully_defined)
    {
        pinMode(pinStandby, OUTPUT);
        digitalWrite(pinStandby, LOW);
        isON = false;
    }
}

int ToF_sensor::powerON()
{
    if (!fully_defined)
    {
        return EXIT_FAILURE;
    }
    setTimeout(TOF_SENSOR_I2C_TIMEOUT_STARTUP);
    print("PowerOn ToF ");
    print(name);
    print("...");
    pinMode(pinStandby, INPUT);
    delay(TOF_SENSOR_INIT_DELAY);

    int ret = init();
    if (ret == EXIT_SUCCESS)
    {
        isON = true;
        print("OK\n");
    }
    else
    {
        standby();
        print("NOT OK\n");
    }
    setTimeout(TOF_SENSOR_I2C_TIMEOUT);
    return ret;
}

int ToF_shortRange::init()
{
    int ret = vlSensor.init();
    if (ret != EXIT_SUCCESS)
    {
        return ret;
    }
    vlSensor.configureDefault();
    vlSensor.setAddress(i2cAddress);
    vlSensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 12);
    vlSensor.stopContinuous();
    delay(TOF_SENSOR_INIT_DELAY);
    vlSensor.startRangeContinuous(TOF_SENSOR_SHORT_RANGE_UPDATE_PERIOD);
    return ret;
}

int ToF_shortRange::measureDistance(int32_t &distance)
{
    distance = vlSensor.readRangeContinuousMillimeters();
    if (vlSensor.timeoutOccurred() || vlSensor.last_status != 0)
    {
        return EXIT_FAILURE;
    }
    else
    {
        return EXIT_SUCCESS;
    }
}

int ToF_longRange::init()
{
    if (vlSensor.init())
    {
        vlSensor.setAddress(i2cAddress);
        vlSensor.stopContinuous();
        delay(TOF_SENSOR_INIT_DELAY);
        vlSensor.startContinuous();
        return EXIT_SUCCESS;
    }
    else
    {
        return EXIT_FAILURE;
    }
}

int ToF_longRange::measureDistance(int32_t &distance)
{
    distance = vlSensor.readRangeContinuousMillimeters();
    if (vlSensor.timeoutOccurred() || vlSensor.last_status != 0)
    {
        return EXIT_FAILURE;
    }
    else
    {
        return EXIT_SUCCESS;
    }
}
