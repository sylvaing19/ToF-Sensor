#ifndef TOF_SENSOR_h
#define TOF_SENSOR_h

#include <Arduino.h>
#include "VL6180X.h"
#include "VL53L0X.h"
#include "Median.h"

typedef int32_t SensorValue;
enum SensorMetadata
{
    SENSOR_DEAD = 0x00,
    SENSOR_NOT_UPDATED = 0x01,
    OBSTACLE_TOO_CLOSE = 0x02,
    NO_OBSTACLE = 0x03
};

class ToF_sensor
{
public:
    ToF_sensor();
    ToF_sensor(uint8_t address, uint8_t pinStandby, int32_t minRange,
        int32_t maxRange, const char* name = "", Stream *debug = nullptr);

    /* Set I2C timeout (ms) */
    virtual void setTimeout(uint16_t timeout) = 0;

    SensorValue getMeasure();
    virtual void standby() = 0;
    int powerON();

protected:
    virtual int init() = 0;
    virtual int measureDistance(int32_t &distance) = 0;
    void writeStandby();

    size_t print(const char *str)
    {
        if (debug_stream == nullptr)
        {
            return 0;
        }
        else
        {
            return debug_stream->print(str);
        }
    }

    bool fully_defined;
    uint8_t i2cAddress;
    uint8_t pinStandby;
    int32_t minRange;  // [mm] Toute valeur strictement inférieure est considérée comme un obstacle trop proche
    int32_t maxRange;  // [mm] Toute valeur strictement supérieure est considérée comme une absence d'obstacle
    bool isON;
    Stream *debug_stream;

public:
    const char* name;
};

class ToF_shortRange : public ToF_sensor
{
public:
    ToF_shortRange() {}
    ToF_shortRange(uint8_t address, uint8_t pinStandby, int32_t minRange = 15,
        int32_t maxRange = 200, const char* name = "", Stream *debug = nullptr) :
        ToF_sensor(address, pinStandby, minRange, maxRange, name, debug)
    {}

    void setTimeout(uint16_t timeout)
    {
        vlSensor.setTimeout(timeout);
    }

    void standby()
    {
        writeStandby();
        vlSensor.resetAddress();
    }

private:
    int init();
    int measureDistance(int32_t &distance);

    VL6180X vlSensor;
};

template<size_t NB_VALUES>
class ToF_shortRange_med : public ToF_shortRange
{
public:
    ToF_shortRange_med() {}
    ToF_shortRange_med(uint8_t address, uint8_t pinStandby, int32_t minRange = 15,
        int32_t maxRange = 200, const char* name = "", Stream *debug = nullptr) :
        ToF_shortRange(address, pinStandby, minRange, maxRange, name, debug)
    {}

    SensorValue getMeasure()
    {
        m_median.add(ToF_shortRange::getMeasure());
        return m_median.value();
    }

private:
    Median<SensorValue, NB_VALUES> m_median;
};

class ToF_longRange : public ToF_sensor
{
public:
    ToF_longRange() {}
    ToF_longRange(uint8_t address, uint8_t pinStandby, int32_t minRange = 30,
        int32_t maxRange = 700, const char* name = "", Stream *debug = nullptr) :
        ToF_sensor(address, pinStandby, minRange, maxRange, name, debug)
    {}

    void setTimeout(uint16_t timeout)
    {
        vlSensor.setTimeout(timeout);
    }

    void standby()
    {
        writeStandby();
        vlSensor.resetAddress();
    }

private:
    int init();
    int measureDistance(int32_t &distance);

    VL53L0X vlSensor;
};

template<size_t NB_VALUES>
class ToF_longRange_med : public ToF_longRange
{
public:
    ToF_longRange_med() {}
    ToF_longRange_med(uint8_t address, uint8_t pinStandby, int32_t minRange = 30,
        int32_t maxRange = 700, const char* name = "", Stream *debug = nullptr) :
        ToF_longRange(address, pinStandby, minRange, maxRange, name, debug)
    {}

    SensorValue getMeasure()
    {
        m_median.add(ToF_longRange::getMeasure());
        return m_median.value();
    }

private:
    Median<SensorValue, NB_VALUES> m_median;
};


#endif
