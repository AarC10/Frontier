#include "core/sensors/Imu.h"

#include <cmath>
#include <zephyr/kernel.h>

Imu::Imu(const device *dev) : dev(dev) {}

int Imu::init(sensor_trigger_handler_t dataReadyHandler) {
	if (dev == nullptr) {
		return -EINVAL;
	}

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	if (dataReadyHandler == nullptr) {
		return -EINVAL;
	}

	sensor_trigger trigger{};
	trigger.type = SENSOR_TRIG_DATA_READY;
	trigger.chan = SENSOR_CHAN_ACCEL_XYZ;

	lastError = sensor_trigger_set(dev, &trigger, dataReadyHandler);
	return lastError;
}

ImuSample Imu::sample() {
	if (dev == nullptr || !device_is_ready(dev)) {
		lastError = -ENODEV;
		return lastSample;
	}

	lastError = sensor_sample_fetch(dev);
	if (lastError != 0) {
		return lastSample;
	}

	sensor_value accel[3]{};
	sensor_value gyro[3]{};

	lastError = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (lastError != 0) {
		return lastSample;
	}

	lastError = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (lastError != 0) {
		return lastSample;
	}

	lastSample.accelX = accel[0];
	lastSample.accelY = accel[1];
	lastSample.accelZ = accel[2];
	lastSample.gyroX = gyro[0];
	lastSample.gyroY = gyro[1];
	lastSample.gyroZ = gyro[2];

	return lastSample;
}

float Imu::accelMagnitudeMg() const {
	const double ax = sensor_value_to_double(&lastSample.accelX);
	const double ay = sensor_value_to_double(&lastSample.accelY);
	const double az = sensor_value_to_double(&lastSample.accelZ);

	return static_cast<float>(std::sqrt(ax * ax + ay * ay + az * az) * 1000.0);
}
