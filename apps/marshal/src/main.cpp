/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log_ctrl.h>

#include <core/Settings.h>
#include <core/sensors/Barometer.h>
#include <core/sensors/Imu.h>

int main() {
    Barometer barometer(DEVICE_DT_GET(DT_ALIAS(barometer)));
    Imu imu(DEVICE_DT_GET(DT_ALIAS(imu)));
    return 0;
}
