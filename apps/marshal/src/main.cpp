/*
 * Copyright (c) 2024 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log_ctrl.h>
#include <core/time.h>
#include <core/defs.h>
#include <core/tdma.h>
#include <core/LoraTransceiver.h>
#include "state_machine.h"
#include <core/GnssReceiver.h>
#include <core/Settings.h>

LOG_MODULE_REGISTER(main);
GNSS_DATA_CALLBACK_DEFINE(DEVICE_DT_GET(DT_ALIAS(gnss)), gnssCallback);

int main() {
    static const gpio_dt_spec pps_spec = GPIO_DT_SPEC_GET(DT_ALIAS(pps), gpios);
    static const gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    if (!device_is_ready(led.port)) {
        LOG_ERR("LED GPIO device not ready\n");
    }

    Settings::load();
    const uint8_t nodeId = Settings::getNodeId();
    const uint32_t freqHz = Settings::getFrequency();
    const float freqMHz = static_cast<float>(freqHz) / 1'000'000;
#ifdef CONFIG_LICENSED_FREQUENCY
    const char callsign[Settings::CALLSIGN_LEN] = {};
    Settings::getCallsign(const_cast<char*>(callsign));
    LOG_INF("Callsign: %.6s", callsign);
    StateMachine sm(nodeId, freqMHz, callsign);
#else
    StateMachine sm(nodeId, freqMHz);
#endif

    time_setup_pps(&pps_spec);

    while (true) {
        const int ret = sm.run();
        if (ret != 0) {
            LOG_ERR("state_machine_run returned %d", ret);
            k_sleep(K_SECONDS(1));
        }

        k_sleep(K_MSEC(100));
    }

    return 0;
}
