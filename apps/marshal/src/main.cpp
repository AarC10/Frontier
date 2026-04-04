/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <core/settings/FlightComputerSettings.h>
#include <marshal/device_runtime.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(marshal, LOG_LEVEL_INF);

int main() {
    int ret = FlightComputerSettings::load();
    if (ret != 0) {
        LOG_ERR("Failed to load settings: %d", ret);
    }

    ret = marshal::initRuntime();
    if (ret != 0) {
        LOG_ERR("Runtime init completed with error: %d", ret);
    }

    marshal::startRuntimeThreads();
    marshal::runStatusLoop();

    return 0;
}
