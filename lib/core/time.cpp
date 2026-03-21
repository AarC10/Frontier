#include "core/time.h"

#include <atomic>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(time);

static std::atomic<uint32_t> gps_seconds{0};

static void time_pps_callback(const device* dev, gpio_callback* cb, uint32_t pins) {
    gps_seconds.fetch_add(1, std::memory_order_relaxed);
}

int time_setup_pps(const gpio_dt_spec* pps) {
    static gpio_callback pps_cb;

    if (!pps || !pps->port) {
        LOG_ERR("Invalid PPS gpio_dt_spec");
        return -EINVAL;
    }

    if (!device_is_ready(pps->port)) {
        LOG_ERR("PPS GPIO device not ready");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(pps, GPIO_INPUT | GPIO_PULL_UP);
    if (ret) {
        LOG_ERR("Failed to configure PPS pin %d, rc=%d", pps->pin, ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(pps, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret) {
        LOG_ERR("Failed to configure interrupt on pin %d, rc=%d", pps->pin, ret);
        return ret;
    }

    gpio_init_callback(&pps_cb, time_pps_callback, BIT(pps->pin));
    ret = gpio_add_callback(pps->port, &pps_cb);
    if (ret) {
        LOG_ERR("Failed to add callback, rc=%d", ret);
        return ret;
    }

    LOG_INF("PPS callback added on pin %d", pps->pin);
    return 0;
}

uint32_t time_get_gps_seconds() { return gps_seconds.load(std::memory_order_relaxed); }
