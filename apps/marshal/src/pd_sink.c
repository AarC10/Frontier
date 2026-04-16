/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb_c/usbc.h>

LOG_MODULE_REGISTER(marshal_pd, LOG_LEVEL_INF);

#define USBC_PORT0_NODE DT_ALIAS(usbc_port0)

#if DT_NODE_HAS_STATUS(USBC_PORT0_NODE, okay)

static struct marshal_pd_port_data {
    uint32_t snk_caps[DT_PROP_LEN(USBC_PORT0_NODE, sink_pdos)];
    int snk_cap_cnt;
    uint32_t src_caps[PDO_MAX_DATA_OBJECTS];
    int src_cap_cnt;
    atomic_t ps_ready;
} port0_data = {
    .snk_caps = DT_PROP(USBC_PORT0_NODE, sink_pdos),
    .snk_cap_cnt = DT_PROP_LEN(USBC_PORT0_NODE, sink_pdos),
    .src_caps = {0},
    .src_cap_cnt = 0,
    .ps_ready = 0,
};

static uint32_t build_rdo(void) {
    union pd_rdo rdo = {
        .raw_value = 0,
    };

    rdo.fixed.min_or_max_operating_current = PD_CONVERT_MA_TO_FIXED_PDO_CURRENT(500);
    rdo.fixed.operating_current = PD_CONVERT_MA_TO_FIXED_PDO_CURRENT(500);
    rdo.fixed.unchunked_ext_msg_supported = 0;
    rdo.fixed.no_usb_suspend = 1;
    rdo.fixed.usb_comm_capable = 1;
    rdo.fixed.cap_mismatch = 0;
    rdo.fixed.giveback = 0;
    rdo.fixed.object_pos = 1;

    return rdo.raw_value;
}

static int port0_policy_cb_get_snk_cap(const struct device *dev, uint32_t **pdos, int *num_pdos) {
    struct marshal_pd_port_data *dpm_data = usbc_get_dpm_data(dev);

    *pdos = dpm_data->snk_caps;
    *num_pdos = dpm_data->snk_cap_cnt;

    return 0;
}

static void port0_policy_cb_set_src_cap(const struct device *dev, const uint32_t *pdos, const int num_pdos) {
    struct marshal_pd_port_data *dpm_data = usbc_get_dpm_data(dev);
    int num = MIN(num_pdos, PDO_MAX_DATA_OBJECTS);

    for (int i = 0; i < num; ++i) {
        dpm_data->src_caps[i] = pdos[i];
    }

    dpm_data->src_cap_cnt = num;
}

static uint32_t port0_policy_cb_get_rdo(const struct device *dev) {
    ARG_UNUSED(dev);
    return build_rdo();
}

static bool port0_policy_check(const struct device *dev, const enum usbc_policy_check_t policy_check) {
    ARG_UNUSED(dev);

    switch (policy_check) {
        case CHECK_POWER_ROLE_SWAP:
            return false;
        case CHECK_DATA_ROLE_SWAP_TO_DFP:
            return false;
        case CHECK_DATA_ROLE_SWAP_TO_UFP:
            return true;
        case CHECK_SNK_AT_DEFAULT_LEVEL:
            return true;
        default:
            return false;
    }
}

static void port0_notify(const struct device *dev, const enum usbc_policy_notify_t policy_notify) {
    struct marshal_pd_port_data *dpm_data = usbc_get_dpm_data(dev);

    switch (policy_notify) {
        case POWER_CHANGE_0A0:
            LOG_INF("USB-C sink current: 0A");
            break;
        case POWER_CHANGE_DEF:
            LOG_INF("USB-C sink current: default");
            break;
        case POWER_CHANGE_1A5:
            LOG_INF("USB-C sink current: 1.5A");
            break;
        case POWER_CHANGE_3A0:
            LOG_INF("USB-C sink current: 3.0A");
            break;
        case TRANSITION_PS:
            atomic_set_bit(&dpm_data->ps_ready, 0);
            break;
        case PD_CONNECTED:
            LOG_INF("USB-C PD contract established");
            break;
        case PORT_PARTNER_NOT_RESPONSIVE:
            LOG_INF("USB-C partner is not PD capable");
            break;
        default:
            break;
    }
}

static int marshal_pd_sink_init(void) {
    const struct device *usbc_port0 = DEVICE_DT_GET(USBC_PORT0_NODE);

    if (!device_is_ready(usbc_port0)) {
        LOG_ERR("USB-C port device not ready");
        return -ENODEV;
    }

    port0_data.ps_ready = ATOMIC_INIT(0);

    usbc_set_policy_cb_check(usbc_port0, port0_policy_check);
    usbc_set_policy_cb_notify(usbc_port0, port0_notify);
    usbc_set_policy_cb_get_snk_cap(usbc_port0, port0_policy_cb_get_snk_cap);
    usbc_set_policy_cb_set_src_cap(usbc_port0, port0_policy_cb_set_src_cap);
    usbc_set_policy_cb_get_rdo(usbc_port0, port0_policy_cb_get_rdo);
    usbc_set_dpm_data(usbc_port0, &port0_data);
    usbc_start(usbc_port0);

    LOG_INF("USB-C PD sink started");
    return 0;
}

SYS_INIT(marshal_pd_sink_init, APPLICATION, 0);

#endif
