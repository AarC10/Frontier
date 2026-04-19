/*
 * Copyright (c) 2024 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "core/LoraTransceiver.h"
#include "core/Settings.h"

#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

LOG_MODULE_REGISTER(main);

int main(void) {
    const int ret = Settings::load();
    if (ret != 0) {
        LOG_ERR("Settings::load failed: %d", ret);
    }

    const uint32_t freqHz = Settings::getFrequency();
    const float freqMhz = static_cast<float>(freqHz) / 1'000'000.0f;
    LoraTransceiver lora(0, freqMhz);
    lora.awaitRxPacket();

    const device *wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
    int wdt_channel_id = -1;
    if (!device_is_ready(wdt)) {
        LOG_ERR("Watchdog device not ready");
    } else {
        wdt_timeout_cfg wdt_cfg{};
        wdt_cfg.window.min = 0U;
        wdt_cfg.window.max = 60'000U;
        wdt_cfg.callback = nullptr;
        wdt_cfg.flags = WDT_FLAG_RESET_SOC;
        wdt_channel_id = wdt_install_timeout(wdt, &wdt_cfg);
        if (wdt_channel_id < 0) {
            LOG_ERR("wdt_install_timeout failed: %d", wdt_channel_id);
        } else if (wdt_setup(wdt, 0) != 0) {
            LOG_ERR("wdt_setup failed");
            wdt_channel_id = -1;
        }
    }

    while (true) {
        if (wdt_channel_id >= 0) {
            wdt_feed(wdt, wdt_channel_id);
        }
        k_sleep(K_SECONDS(30));
        lora.awaitCancel();
        lora.awaitRxPacket();
    }

    return 0;
}
