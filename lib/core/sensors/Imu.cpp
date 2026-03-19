#include "core/sensors/Imu.h"

#include <cmath>
#include <zephyr/kernel.h>

Imu::Imu(const device *dev) : dev_(dev) {}

int Imu::init(sensor_trigger_handler_t dataReadyHandler) {
	if (dev_ == nullptr) {
		return -EINVAL;
	}

	if (!device_is_ready(dev_)) {
		return -ENODEV;
	}

	if (dataReadyHandler == nullptr) {
		return -EINVAL;
	}

	sensor_trigger trigger{};
	trigger.type = SENSOR_TRIG_DATA_READY;
	trigger.chan = SENSOR_CHAN_ACCEL_XYZ;

	lastError_ = sensor_trigger_set(dev_, &trigger, dataReadyHandler);
	return lastError_;
}

ImuSample Imu::sample() {
	if (dev_ == nullptr || !device_is_ready(dev_)) {
		lastError_ = -ENODEV;
		return lastSample_;
	}

	lastError_ = sensor_sample_fetch(dev_);
	if (lastError_ != 0) {
		return lastSample_;
	}

	sensor_value accel[3]{};
	sensor_value gyro[3]{};

	lastError_ = sensor_channel_get(dev_, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (lastError_ != 0) {
		return lastSample_;
	}

	lastError_ = sensor_channel_get(dev_, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (lastError_ != 0) {
		return lastSample_;
	}

	lastSample_.accelX = accel[0];
	lastSample_.accelY = accel[1];
	lastSample_.accelZ = accel[2];
	lastSample_.gyroX = gyro[0];
	lastSample_.gyroY = gyro[1];
	lastSample_.gyroZ = gyro[2];

	return lastSample_;
}

float Imu::accelMagnitudeMg() const {
	const double ax = sensor_value_to_double(&lastSample_.accelX);
	const double ay = sensor_value_to_double(&lastSample_.accelY);
	const double az = sensor_value_to_double(&lastSample_.accelZ);

	return static_cast<float>(std::sqrt(ax * ax + ay * ay + az * az) * 1000.0);
}
