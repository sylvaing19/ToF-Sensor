#include "ToF_sensor.h"

#define TOF_SENSOR_I2C_TIMEOUT          50  // ms
#define TOF_SENSOR_I2C_TIMEOUT_STARTUP  200 // ms
#define TOF_SENSOR_INIT_DELAY           50  // ms
#define TOF_SENSOR_SHORT_RANGE_DEF_UP_P 20  // ms (default update period for short range sensor)

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
    started = false;
}

ToF_sensor::ToF_sensor(uint8_t address, uint8_t pinStandby, int32_t minRange,
    int32_t maxRange, const char* name, Stream *debug) :
        i2cAddress(address), pinStandby(pinStandby), minRange(minRange),
        maxRange(maxRange), debug_stream(debug), name(name)
{
    isON = false;
    fully_defined = true;
    started = false;
    writeStandby();
}

SensorValue ToF_sensor::getMeasure()
{
    if (!fully_defined || !isON) {
        return (SensorValue)SENSOR_DEAD;
    }

    int32_t distance;
    int ret = measureDistance(distance);

    if (ret != EXIT_SUCCESS) {
        return (SensorValue)SENSOR_NOT_UPDATED;
    }
    else {
        return interpretMeasure(distance);
    }
}

SensorValue ToF_sensor::interpretMeasure(int32_t distance)
{
    if (distance > maxRange)
    {
        return (SensorValue)NO_OBSTACLE;
    }
    else if (distance < minRange)
    {
        return (SensorValue)OBSTACLE_TOO_CLOSE;
    }
    else
    {
        return (SensorValue)distance;
    }
}

void ToF_sensor::writeStandby()
{
    if (fully_defined)
    {
        pinMode(pinStandby, OUTPUT);
        digitalWrite(pinStandby, LOW);
        isON = false;
        started = false;
    }
}

int ToF_sensor::powerON(bool autoStart, uint32_t period)
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

    int ret = init(autoStart, period);
    if (ret == EXIT_SUCCESS)
    {
        isON = true;
        print("OK\n");
    }
    else
    {
        writeStandby();
        print("NOT OK\n");
    }
    setTimeout(TOF_SENSOR_I2C_TIMEOUT);
    return ret;
}

void ToF_sensor::startMeasurement(uint32_t period)
{
    if (fully_defined && isON) {
        start(period);
        started = true;
    }
}

void ToF_sensor::stopMeasurement()
{
    if (fully_defined && isON) {
        stop();
        started = false;
    }
}

void ToF_sensor::setRange(int32_t aMin, int32_t aMax)
{
    minRange = aMin;
    maxRange = aMax;
}

int ToF_shortRange::init(bool autoStart, uint32_t period)
{
    int ret = vlSensor.init();
    if (ret != EXIT_SUCCESS)
    {
        return ret;
    }
    vlSensor.configureDefault();
    vlSensor.setAddress(i2cAddress);
    vlSensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 12);
    stop();
    if (autoStart) {
        delay(TOF_SENSOR_INIT_DELAY);
        start(period);
    }
    return ret;
}

void ToF_shortRange::start(uint32_t period)
{
    uint16_t p;
    if (period == 0) {
        p = TOF_SENSOR_SHORT_RANGE_DEF_UP_P;
    }
    else if (period > UINT16_MAX) {
        p = UINT16_MAX;
    }
    else {
        p = period;
    }
    vlSensor.startRangeContinuous(p);
}

void ToF_shortRange::stop()
{
    vlSensor.stopContinuous();
}

int ToF_shortRange::measureDistance(int32_t &distance)
{
    uint16_t range;
    if (vlSensor.fastReadRangeMillimeters(range))
    {
        distance = (int32_t)range;
        return EXIT_SUCCESS;
    }
    else
    {
        return EXIT_FAILURE;
    }
}

int ToF_longRange::init(bool autoStart, uint32_t period)
{
    if (vlSensor.init())
    {
        vlSensor.setAddress(i2cAddress);
        stop();
        if (autoStart) {
            delay(TOF_SENSOR_INIT_DELAY);
            start(period);
        }
        return EXIT_SUCCESS;
    }
    else
    {
        return EXIT_FAILURE;
    }
}

void ToF_longRange::start(uint32_t period)
{
    vlSensor.startContinuous(period);
}

void ToF_longRange::stop()
{
    vlSensor.stopContinuous();
}

int ToF_longRange::getFullMeasure(SensorValue &range, uint16_t &raw_range, uint16_t &quality)
{
    if (!fully_defined || !isON) {
        return EXIT_FAILURE;
    }
    int32_t distance;
    int ret = getRawMeasure(distance, quality, raw_range);
    if (ret == EXIT_SUCCESS) {
        range = interpretMeasure(distance);
    }
    return ret;
}

int ToF_longRange::measureDistance(int32_t &distance)
{
    uint16_t quality;
    uint16_t raw_range;
    return getRawMeasure(distance, quality, raw_range);
}

int ToF_longRange::getRawMeasure(int32_t &distance, uint16_t &quality, uint16_t &raw_range)
{
    uint8_t status;
    if (vlSensor.readAllRangeData(status, quality, raw_range)) {
        if (status & 0x04) {
            return EXIT_FAILURE;
        }
        distance = computeQuality(status, quality, raw_range);
        return EXIT_SUCCESS;
    }
    else {
        return EXIT_FAILURE;
    }
}

int32_t ToF_longRange::computeQuality(uint8_t status, uint16_t quality, uint16_t raw_range)
{
    if ((status & 0x78) >> 3 == 0x0B && quality > qualityThreshold) {
        return (int32_t)raw_range;
    }
    else {
        return INT32_MAX;
    }
}
