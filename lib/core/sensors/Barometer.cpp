#include "core/sensors/Barometer.h"

#include <cmath>
#include <zephyr/kernel.h>

Barometer::Barometer(const device *dev) : dev(dev) {}

int Barometer::init() const {
	if (dev == nullptr) {
		return -EINVAL;
	}

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	return 0;
}

BaroSample Barometer::sample() {
	if (dev == nullptr || !device_is_ready(dev)) {
		lastError = -ENODEV;
		return lastSample;
	}

	lastError = sensor_sample_fetch(dev);
	if (lastError != 0) {
		return lastSample;
	}

	lastError = sensor_channel_get(dev, SENSOR_CHAN_PRESS, &lastSample.pressure);
	if (lastError != 0) {
		return lastSample;
	}

	lastError = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &lastSample.temperature);
	return lastSample;
}

float Barometer::altitudeMeters(float referencePressurePa) const {
	if (referencePressurePa <= 0.0f) {
		return NAN;
	}

	const double pressurePa = sensor_value_to_double(&lastSample.pressure) * 1000.0;
	if (pressurePa <= 0.0) {
		return NAN;
	}

	// ISA pressure altitude approximation.
	return static_cast<float>(44330.0 * (1.0 - std::pow(pressurePa / referencePressurePa, 0.19029495718)));
}
