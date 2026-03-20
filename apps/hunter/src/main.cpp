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
    // Settings::load();
    // float freqMHz = static_cast<float>(OutlawSettings::getFrequency()) /
    // 1'000'000;
#ifdef CONFIG_LICENSED_FREQUENCY
    float freqMhz = 435.0f;
#else
    float freqMhz = 903.0f;
#endif
    LoraTransceiver lora(0, freqMhz);
    lora.awaitRxPacket();

    while (true) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
