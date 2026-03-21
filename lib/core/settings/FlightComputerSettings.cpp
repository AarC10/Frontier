/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "core/FlightComputerSettings.h"

#include <cstdlib>
#include <cstring>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#ifdef CONFIG_SHELL
#include <zephyr/shell/shell.h>
#endif

LOG_MODULE_REGISTER(FlightComputerSettings);

// RAM copy

static uint8_t cfgDeployMode = static_cast<uint8_t>(FlightComputerSettings::DEFAULT_DEPLOY_MODE);
static uint16_t cfgMainDeployAlt = FlightComputerSettings::DEFAULT_MAIN_DEPLOY_ALT_M;
static uint16_t cfgArmingAlt = FlightComputerSettings::DEFAULT_ARMING_ALT_M;
static uint16_t cfgApogeeDelay = FlightComputerSettings::DEFAULT_APOGEE_DELAY_MS;
static uint16_t cfgMinBattery = FlightComputerSettings::DEFAULT_MIN_BATTERY_MV;
static uint32_t cfgFlightCounter = FlightComputerSettings::DEFAULT_FLIGHT_COUNTER;

// handlers
static int fcSettingsSet(const char *name, size_t len, settings_read_cb readCb, void *cbArg) {
    if (strcmp(name, "mode") == 0) {
        if (len != sizeof(uint8_t)) return -EINVAL;
        readCb(cbArg, &cfgDeployMode, sizeof(cfgDeployMode));
        return 0;
    }

    if (strcmp(name, "main_alt") == 0) {
        if (len != sizeof(uint16_t)) return -EINVAL;
        readCb(cbArg, &cfgMainDeployAlt, sizeof(cfgMainDeployAlt));
        return 0;
    }

    if (strcmp(name, "arm_alt") == 0) {
        if (len != sizeof(uint16_t)) return -EINVAL;
        readCb(cbArg, &cfgArmingAlt, sizeof(cfgArmingAlt));
        return 0;
    }

    if (strcmp(name, "apo_dly") == 0) {
        if (len != sizeof(uint16_t)) return -EINVAL;
        readCb(cbArg, &cfgApogeeDelay, sizeof(cfgApogeeDelay));
        return 0;
    }

    if (strcmp(name, "bat_min") == 0) {
        if (len != sizeof(uint16_t)) return -EINVAL;
        readCb(cbArg, &cfgMinBattery, sizeof(cfgMinBattery));
        return 0;
    }

    if (strcmp(name, "flt_cnt") == 0) {
        if (len != sizeof(uint32_t)) return -EINVAL;
        readCb(cbArg, &cfgFlightCounter, sizeof(cfgFlightCounter));
        return 0;
    }

    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(fc, "fc", nullptr, fcSettingsSet, nullptr, nullptr);

namespace FlightComputerSettings {

int load() {
    int ret = settings_subsys_init();
    if (ret != 0) {
        LOG_ERR("settings_subsys_init failed: %d", ret);
        return ret;
    }

    ret = settings_load_subtree("fc");
    if (ret != 0) {
        LOG_ERR("settings_load_subtree(fc) failed: %d", ret);
        return ret;
    }

    LOG_INF("Settings loaded: mode=%u main=%u m arm=%u m apoDly=%u ms bat=%u mV flights=%u", cfgDeployMode,
            cfgMainDeployAlt, cfgArmingAlt, cfgApogeeDelay, cfgMinBattery, cfgFlightCounter);

    return 0;
}

DeployMode deployMode() { return static_cast<DeployMode>(cfgDeployMode); }
uint16_t mainDeployAltM() { return cfgMainDeployAlt; }
uint16_t armingAltM() { return cfgArmingAlt; }
uint16_t apogeeDelayMs() { return cfgApogeeDelay; }
uint16_t minBatteryMv() { return cfgMinBattery; }
uint32_t flightCounter() { return cfgFlightCounter; }

int setDeployMode(DeployMode mode) {
    cfgDeployMode = static_cast<uint8_t>(mode);
    const int ret = settings_save_one("fc/mode", &cfgDeployMode, sizeof(cfgDeployMode));
    if (ret != 0) {
        LOG_ERR("Failed to save deploy mode: %d", ret);
    }
    return ret;
}

int setMainDeployAltM(uint16_t altMeters) {
    cfgMainDeployAlt = altMeters;
    const int ret = settings_save_one("fc/main_alt", &cfgMainDeployAlt, sizeof(cfgMainDeployAlt));
    if (ret != 0) {
        LOG_ERR("Failed to save main deploy alt: %d", ret);
    }
    return ret;
}

int setArmingAltM(uint16_t altMeters) {
    cfgArmingAlt = altMeters;
    const int ret = settings_save_one("fc/arm_alt", &cfgArmingAlt, sizeof(cfgArmingAlt));
    if (ret != 0) {
        LOG_ERR("Failed to save arming alt: %d", ret);
    }
    return ret;
}

int setApogeeDelayMs(uint16_t delayMs) {
    cfgApogeeDelay = delayMs;
    const int ret = settings_save_one("fc/apo_dly", &cfgApogeeDelay, sizeof(cfgApogeeDelay));
    if (ret != 0) {
        LOG_ERR("Failed to save apogee delay: %d", ret);
    }
    return ret;
}

int setMinBatteryMv(uint16_t mv) {
    cfgMinBattery = mv;
    const int ret = settings_save_one("fc/bat_min", &cfgMinBattery, sizeof(cfgMinBattery));
    if (ret != 0) {
        LOG_ERR("Failed to save min battery: %d", ret);
    }
    return ret;
}

uint32_t incrementFlightCounter() {
    cfgFlightCounter++;
    const int ret = settings_save_one("fc/flt_cnt", &cfgFlightCounter, sizeof(cfgFlightCounter));
    if (ret != 0) {
        LOG_ERR("Failed to save flight counter: %d", ret);
    }
    return cfgFlightCounter;
}

}
