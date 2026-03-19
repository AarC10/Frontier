#include "core/sensors/VoltageMonitor.h"

#include <zephyr/autoconf.h>
#include <zephyr/kernel.h>

VoltageMonitor::VoltageMonitor(const device *vbatDev, const device *vccDev)
	: vccDev(vccDev), vbatDev(vbatDev) {}


int VoltageMonitor::init() const {
	if (vbatDev == nullptr || vccDev == nullptr) {
		return -EINVAL;
	}

	if (!device_is_ready(vbatDev) || !device_is_ready(vccDev)) {
		return -ENODEV;
	}

	return 0;
}

int VoltageMonitor::sample() {
	if (vbatDev == nullptr || vccDev == nullptr || !device_is_ready(vbatDev) || !device_is_ready(vccDev)) {
		lastError = -ENODEV;
		return lastError;
	}

	lastError = sensor_sample_fetch(vbatDev);
	if (lastError != 0) {
		return lastError;
	}

	lastError = sensor_channel_get(vbatDev, SENSOR_CHAN_VOLTAGE, &vbat);
	if (lastError != 0) {
		return lastError;
	}

	lastError = sensor_sample_fetch(vccDev);
	if (lastError != 0) {
		return lastError;
	}

	lastError = sensor_channel_get(vccDev, SENSOR_CHAN_VOLTAGE, &vcc);
	return lastError;
}


bool VoltageMonitor::isVbatOk() const {
	return vbatMv() >= CONFIG_VBAT_MIN_MV;
}

int VoltageMonitor::vccMv() const {
	return sensorValueToMilliVolts(vcc);
}

int VoltageMonitor::vbatMv() const {
	return sensorValueToMilliVolts(vbat);
}

int VoltageMonitor::sensorValueToMilliVolts(const sensor_value &value) {
	const int64_t wholeMv = static_cast<int64_t>(value.val1) * 1000;
	const int64_t fracMv = value.val2 / 1000;
	return static_cast<int>(wholeMv + fracMv);
}

