/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <core/io/Led.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

LOG_MODULE_REGISTER(marshal, LOG_LEVEL_INF);

static gpio_dt_spec statusLedSpec = GPIO_DT_SPEC_GET(DT_ALIAS(led), gpios);
static Led statusLed(&statusLedSpec);

int main() {
    statusLed.init();
    LOG_INF("Hello World!");
    while (true) {
        statusLed.toggle();
        k_msleep(1000);
    }

    return 0;
}