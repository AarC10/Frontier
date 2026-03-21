#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

struct BaroSample {
    sensor_value pressure;
    sensor_value temperature;
};

class Barometer {
  public:
    explicit Barometer(const device *dev);

    int init() const;
    BaroSample sample();
    float altitudeMeters(float referencePressurePa) const;

  private:
    const device *dev;
    BaroSample lastSample{};
    int lastError{0};
};
