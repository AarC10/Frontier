/*
 * Copyright (c) 2024 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "core/LoraTransceiver.h"
#include "core/Settings.h"

#include <zephyr/device.h>
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

    while (true) {
        k_sleep(K_SECONDS(6));
    }

    return 0;
}
