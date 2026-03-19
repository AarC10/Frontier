#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

struct ImuSample {
	sensor_value accelX;
	sensor_value accelY;
	sensor_value accelZ;
	sensor_value gyroX;
	sensor_value gyroY;
	sensor_value gyroZ;
};

class Imu {
public:
	explicit Imu(const device *dev);

	int init(sensor_trigger_handler_t dataReadyHandler);
	ImuSample sample();
	float accelMagnitudeMg() const;

private:
	const device *dev;
	ImuSample lastSample{};
	int lastError{0};
};
