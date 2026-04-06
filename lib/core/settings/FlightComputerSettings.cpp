/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "core/settings/FlightComputerSettings.h"

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
static uint16_t cfgMainDeployAlt = FlightComputerSettings::DEFAULT_MAIN_DEPLOY_ALT_FT;
static uint16_t cfgArmingAlt = FlightComputerSettings::DEFAULT_ARMING_ALT_FT;
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

    LOG_INF("Settings loaded: mode=%u main=%u ft arm=%u ft apoDly=%u ms bat=%u mV flights=%u", cfgDeployMode,
            cfgMainDeployAlt, cfgArmingAlt, cfgApogeeDelay, cfgMinBattery, cfgFlightCounter);

    return 0;
}

DeployMode deployMode() { return static_cast<DeployMode>(cfgDeployMode); }
uint16_t mainDeployAltFt() { return cfgMainDeployAlt; }
uint16_t armingAltFt() { return cfgArmingAlt; }
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

int setMainDeployAltFt(uint16_t altFeet) {
    cfgMainDeployAlt = altFeet;
    const int ret = settings_save_one("fc/main_alt", &cfgMainDeployAlt, sizeof(cfgMainDeployAlt));
    if (ret != 0) {
        LOG_ERR("Failed to save main deploy alt: %d", ret);
    }
    return ret;
}

int setArmingAltFt(uint16_t altFeet) {
    cfgArmingAlt = altFeet;
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

float mainDeployAltM() { return static_cast<float>(cfgMainDeployAlt) * FT_TO_M; }
float armingAltM() { return static_cast<float>(cfgArmingAlt) * FT_TO_M; }
} // namespace FlightComputerSettings

#ifdef CONFIG_SHELL

static int cmdConfigRoot(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argv);

    if (argc != 1) {
        shell_print(sh, "ERR invalid arguments");
        return -EINVAL;
    }

    shell_print(sh, "supported mode");
    shell_print(sh, "supported main_alt");
    shell_print(sh, "supported arm_alt");
    shell_print(sh, "supported apogee_delay");
    shell_print(sh, "supported bat_min");
    shell_print(sh, "identity marshal");

    return 0;
}

static int cmdShow(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    const char *modeStr;
    switch (FlightComputerSettings::deployMode()) {
        case FlightComputerSettings::DeployMode::DUAL_DEPLOY:
            modeStr = "dual_deploy";
            break;
        case FlightComputerSettings::DeployMode::DROGUE_ONLY:
            modeStr = "drogue_only";
            break;
        case FlightComputerSettings::DeployMode::MAIN_ONLY:
            modeStr = "main_only";
            break;
        default:
            modeStr = "unknown";
            break;
    }

    shell_print(sh, "Deploy mode:      %s", modeStr);
    shell_print(sh, "Main deploy alt:  %u ft", FlightComputerSettings::mainDeployAltFt());
    shell_print(sh, "Arming altitude:  %u ft", FlightComputerSettings::armingAltFt());
    shell_print(sh, "Apogee delay:     %u ms", FlightComputerSettings::apogeeDelayMs());
    shell_print(sh, "Min battery:      %u mV", FlightComputerSettings::minBatteryMv());
    shell_print(sh, "Flight counter:   %u", FlightComputerSettings::flightCounter());

    return 0;
}

static int cmdMode(const struct shell *sh, size_t argc, char **argv) {
    if (strcmp(argv[1], "dual_deploy") == 0) {
        const int ret = FlightComputerSettings::setDeployMode(FlightComputerSettings::DeployMode::DUAL_DEPLOY);
        if (ret == 0) {
            shell_print(sh, "OK");
        } else {
            shell_error(sh, "ERR save failed");
        }
        return ret;
    }
    if (strcmp(argv[1], "drogue_only") == 0) {
        const int ret = FlightComputerSettings::setDeployMode(FlightComputerSettings::DeployMode::DROGUE_ONLY);
        if (ret == 0) {
            shell_print(sh, "OK");
        } else {
            shell_error(sh, "ERR save failed");
        }
        return ret;
    }
    if (strcmp(argv[1], "main_only") == 0) {
        const int ret = FlightComputerSettings::setDeployMode(FlightComputerSettings::DeployMode::MAIN_ONLY);
        if (ret == 0) {
            shell_print(sh, "OK");
        } else {
            shell_error(sh, "ERR save failed");
        }
        return ret;
    }

    shell_error(sh, "ERR invalid mode");
    return -EINVAL;
}

static int cmdMainAlt(const struct shell *sh, size_t argc, char **argv) {
    char *end;
    const unsigned long alt = strtoul(argv[1], &end, 10);
    if (*end != '\0' || alt > 30000) {
        shell_error(sh, "ERR invalid main_alt");
        return -EINVAL;
    }

    const int ret = FlightComputerSettings::setMainDeployAltFt(static_cast<uint16_t>(alt));
    if (ret == 0) {
        shell_print(sh, "OK");
    } else {
        shell_error(sh, "ERR save failed");
    }
    return ret;
}

static int cmdArmAlt(const struct shell *sh, size_t argc, char **argv) {
    char *end;
    const unsigned long alt = strtoul(argv[1], &end, 10);
    if (*end != '\0' || alt > 3000) {
        shell_error(sh, "ERR invalid arm_alt");
        return -EINVAL;
    }

    const int ret = FlightComputerSettings::setArmingAltFt(static_cast<uint16_t>(alt));
    if (ret == 0) {
        shell_print(sh, "OK");
    } else {
        shell_error(sh, "ERR save failed");
    }
    return ret;
}

static int cmdApogeeDelay(const struct shell *sh, size_t argc, char **argv) {
    char *end;
    const unsigned long delay = strtoul(argv[1], &end, 10);
    if (*end != '\0' || delay > 30000) {
        shell_error(sh, "ERR invalid apogee_delay");
        return -EINVAL;
    }

    const int ret = FlightComputerSettings::setApogeeDelayMs(static_cast<uint16_t>(delay));
    if (ret == 0) {
        shell_print(sh, "OK");
    } else {
        shell_error(sh, "ERR save failed");
    }
    return ret;
}

static int cmdBatMin(const struct shell *sh, size_t argc, char **argv) {
    char *end;
    const unsigned long mv = strtoul(argv[1], &end, 10);
    if (*end != '\0' || mv > 10000) {
        shell_error(sh, "ERR invalid bat_min");
        return -EINVAL;
    }

    const int ret = FlightComputerSettings::setMinBatteryMv(static_cast<uint16_t>(mv));
    if (ret == 0) {
        shell_print(sh, "OK");
    } else {
        shell_error(sh, "ERR save failed");
    }
    return ret;
}

static int cmdDefaults(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    int ret = 0;
    ret |= FlightComputerSettings::setDeployMode(FlightComputerSettings::DEFAULT_DEPLOY_MODE);
    ret |= FlightComputerSettings::setMainDeployAltFt(FlightComputerSettings::DEFAULT_MAIN_DEPLOY_ALT_FT);
    ret |= FlightComputerSettings::setArmingAltFt(FlightComputerSettings::DEFAULT_ARMING_ALT_FT);
    ret |= FlightComputerSettings::setApogeeDelayMs(FlightComputerSettings::DEFAULT_APOGEE_DELAY_MS);
    ret |= FlightComputerSettings::setMinBatteryMv(FlightComputerSettings::DEFAULT_MIN_BATTERY_MV);

    if (ret == 0) {
        shell_print(sh, "All settings reset to defaults");
    } else {
        shell_error(sh, "Some settings failed to save: %d", ret);
    }
    return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
    subConfig, SHELL_CMD(show, NULL, "Display current settings", cmdShow),
    SHELL_CMD_ARG(mode, NULL, "Set deploy mode (dual_deploy|drogue_only|main_only)", cmdMode, 2, 0),
    SHELL_CMD_ARG(main_alt, NULL, "Set main deploy altitude in feet AGL (default 500)", cmdMainAlt, 2, 0),
    SHELL_CMD_ARG(arm_alt, NULL, "Set arming altitude in feet AGL (default 100)", cmdArmAlt, 2, 0),
    SHELL_CMD_ARG(apogee_delay, NULL, "Set apogee-to-drogue delay in ms (default 0)", cmdApogeeDelay, 2, 0),
    SHELL_CMD_ARG(bat_min, NULL, "Set minimum battery voltage lockout in mV (default 3300)", cmdBatMin, 2, 0),
    SHELL_CMD(defaults, NULL, "Reset all settings to defaults", cmdDefaults), SHELL_SUBCMD_SET_END);

SHELL_CMD_ARG_REGISTER(config, &subConfig, "Flight computer settings", cmdConfigRoot, 1, 0);

#endif // CONFIG_SHELL
