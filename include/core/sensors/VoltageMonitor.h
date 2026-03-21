#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

class VoltageMonitor {

  public:
    explicit VoltageMonitor(const device *vbatDev, const device *vccDev);
    int init() const;
    int sample();
    bool isVbatOk() const;
    int vccMv() const;
    int vbatMv() const;

  private:
    const device *vccDev;
    const device *vbatDev;

    sensor_value vcc{};
    sensor_value vbat{};

    int lastError{0};

    static int sensorValueToMilliVolts(const sensor_value &value);
};
