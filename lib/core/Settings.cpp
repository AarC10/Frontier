/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "core/Settings.h"

#include <cstdlib>
#include <cstring>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#ifdef CONFIG_SHELL
#include <zephyr/shell/shell.h>
#endif

#if defined(CONFIG_SHELL_FREQUENCY) || defined(CONFIG_LICENSED_FREQUENCY) || defined(CONFIG_SHELL_NODE_ID)

LOG_MODULE_REGISTER(Settings);

static uint32_t CONFIGURED_FREQUENCY = Settings::DEFAULT_FREQUENCY;
#ifdef CONFIG_LICENSED_FREQUENCY
static char CONFIGURED_CALLSIGN[Settings::CALLSIGN_LEN] = {};
#endif
static uint8_t CONFIGURED_NODE_ID = Settings::DEFAULT_NODE_ID;

static int settings_set_handler(const char *name, size_t len, settings_read_cb readCallback, void *callbackArgs) {
#ifdef CONFIG_SHELL_FREQUENCY
    if (strcmp(name, "freq") == 0) {
        if (len != sizeof(uint32_t)) {
            return -EINVAL;
        }
        readCallback(callbackArgs, &CONFIGURED_FREQUENCY, sizeof(uint32_t));
        return 0;
    }
#endif

#ifdef CONFIG_LICENSED_FREQUENCY
    if (strcmp(name, "cs") == 0) {
        const size_t to_read =
            len < static_cast<size_t>(Settings::CALLSIGN_LEN) ? len : static_cast<size_t>(Settings::CALLSIGN_LEN);
        readCallback(callbackArgs, CONFIGURED_CALLSIGN, to_read);
        return 0;
    }
#endif

#ifdef CONFIG_SHELL_NODE_ID
    if (strcmp(name, "nid") == 0) {
        if (len != sizeof(uint8_t)) return -EINVAL;
        readCallback(callbackArgs, &CONFIGURED_NODE_ID, sizeof(uint8_t));
        return 0;
    }
#endif
    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(config, "config", nullptr, settings_set_handler, nullptr, nullptr);

namespace Settings {

int load() {
    int ret = settings_subsys_init();
    if (ret != 0) {
        LOG_ERR("settings_subsys_init failed: %d", ret);
        return ret;
    }
    ret = settings_load_subtree("config");
    if (ret != 0) {
        LOG_ERR("settings_load_subtree failed: %d", ret);
    }
    return ret;
}

#ifdef CONFIG_SHELL_FREQUENCY
uint32_t getFrequency() {
    LOG_INF("Loaded frequency: %f MHz", static_cast<double>(CONFIGURED_FREQUENCY) / 1'000'000);
    return CONFIGURED_FREQUENCY;
}

int saveFrequency(uint32_t frequency) {
    CONFIGURED_FREQUENCY = frequency;
    const int ret = settings_save_one("config/freq", &frequency, sizeof(frequency));
    if (ret != 0) {
        LOG_ERR("settings_save_one(config/freq) failed: %d", ret);
    }
    return ret;
}
#endif

#ifdef CONFIG_LICENSED_FREQUENCY
void getCallsign(char out[CALLSIGN_LEN]) { memcpy(out, CONFIGURED_CALLSIGN, CALLSIGN_LEN); }

int saveCallsign(const char callsign[CALLSIGN_LEN]) {
    memcpy(CONFIGURED_CALLSIGN, callsign, CALLSIGN_LEN);
    const int ret = settings_save_one("config/cs", callsign, CALLSIGN_LEN);
    if (ret != 0) {
        LOG_ERR("settings_save_one(config/cs) failed: %d", ret);
    }
    return ret;
}
#endif

#ifdef CONFIG_SHELL_NODE_ID
uint8_t getNodeId() { return CONFIGURED_NODE_ID; }

int saveNodeId(uint8_t nodeId) {
    if (nodeId < 0 || nodeId > 9) return -EINVAL;
    CONFIGURED_NODE_ID = nodeId;
    const int ret = settings_save_one("config/nid", &nodeId, sizeof(nodeId));
    if (ret != 0) {
        LOG_ERR("settings_save_one(config/nid) failed: %d", ret);
    }
    return ret;
}
#endif
} // namespace Settings

#ifdef CONFIG_SHELL

#ifdef CONFIG_SHELL_FREQUENCY
static int cmd_freq(const struct shell *sh, size_t argc, char **argv) {
    char *end;
    const float freq = strtof(argv[1], &end);
    if (*end != '\0' || freq < 902.0f || freq > 928.0f) {
        shell_error(sh, "Invalid frequency '%s' (902.0 - 928.0)", argv[1]);
        return -EINVAL;
    }
    const int ret = Settings::saveFrequency(static_cast<uint32_t>(freq * 1'000'000));

    if (ret == 0) {
        shell_print(sh, "Frequency saved: %f Mhz (reboot to apply)", static_cast<double>(freq));
    } else {
        shell_error(sh, "Save failed: %d", ret);
    }
    return ret;
}
#endif

#ifdef CONFIG_LICENSED_FREQUENCY

static int cmd_callsign(const struct shell *sh, size_t argc, char **argv) {
    const size_t len = strlen(argv[1]);
    if (len < 4) {
        shell_warn(
            sh,
            "Callsigns must be at least 4 characters. Assuming no callsign and suspending transmission upon reboot.");
        const int ret = Settings::saveCallsign("");
        if (ret != 0) {
            shell_error(sh, "Save failed: %d", ret);
        }
        return ret;
    } else if (len > (size_t) Settings::CALLSIGN_LEN) {
        shell_error(sh, "Callsign must be 1-%d characters", Settings::CALLSIGN_LEN);
        return -EINVAL;
    }
    char cs[Settings::CALLSIGN_LEN] = {};
    memcpy(cs, argv[1], len);
    const int ret = Settings::saveCallsign(cs);
    if (ret == 0) {
        shell_print(sh, "Callsign saved: %.*s (reboot to apply)", Settings::CALLSIGN_LEN, cs);
    } else {
        shell_error(sh, "Save failed: %d", ret);
    }
    return ret;
}

#endif

#ifdef CONFIG_SHELL_NODE_ID

static int cmd_node_id(const struct shell *sh, size_t argc, char **argv) {
    char *end;
    const unsigned long id = strtoul(argv[1], &end, 10);
    if (*end != '\0' || id < 0 || id > 9) {
        shell_error(sh, "Invalid node ID '%s' (expected 0-9)", argv[1]);
        return -EINVAL;
    }
    const int ret = Settings::saveNodeId(static_cast<uint8_t>(id));
    if (ret == 0) {
        shell_print(sh, "Node ID saved: %lu (reboot to apply)", id);
    } else {
        shell_error(sh, "Save failed: %d", ret);
    }
    return ret;
}
#endif

#if defined(CONFIG_SHELL_FREQUENCY) || defined(CONFIG_LICENSED_FREQUENCY) || defined(CONFIG_SHELL_NODE_ID)
SHELL_STATIC_SUBCMD_SET_CREATE(sub_config,
#ifdef CONFIG_SHELL_FREQUENCY
                               SHELL_CMD_ARG(freq, NULL, "Set LoRa frequency in Mhz (e.g. 903.123456)", cmd_freq, 2, 0),
#endif

#ifdef CONFIG_LICENSED_FREQUENCY
                               SHELL_CMD_ARG(callsign, NULL, "Set callsign, max 6 chars (e.g. W1ABC)", cmd_callsign, 2,
                                             0),
#endif

#ifdef CONFIG_SHELL_NODE_ID
                               SHELL_CMD_ARG(node_id, NULL, "Set node ID (0-9)", cmd_node_id, 2, 0),
#endif
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(config, &sub_config, "Configure settings", NULL);

#endif

#endif // CONFIG_SHELL

#endif