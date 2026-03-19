#pragma once

#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>

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


