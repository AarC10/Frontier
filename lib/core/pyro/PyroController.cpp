/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <core/pyro/PyroController.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pyro_controller, CONFIG_LOG_DEFAULT_LEVEL);

PyroController::PyroController(const gpio_dt_spec &chEn, const gpio_dt_spec &flt) : chEn(chEn), flt(flt) {}

int PyroController::init() {
    if (!gpio_is_ready_dt(&chEn)) {
        LOG_ERR("Pyro enable GPIO not ready");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&chEn, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        LOG_ERR("Failed to configure pyro enable pin: %d", ret);
        return ret;
    }

    if (hasFltPin()) {
        if (!gpio_is_ready_dt(&flt)) {
            LOG_ERR("Pyro fault GPIO not ready");
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&flt, GPIO_INPUT);
        if (ret != 0) {
            LOG_ERR("Failed to configure pyro fault pin: %d", ret);
            return ret;
        }
    }

    return 0;
}

int PyroController::arm() {
    if (isFault()) {
        LOG_ERR("Cannot arm: fault detected");
        return -EIO;
    }

    armed = true;
    LOG_INF("Pyro channel armed");
    return 0;
}

int PyroController::fire(uint32_t durationMs) {
    if (!armed) {
        LOG_ERR("Cannot fire: channel not armed");
        return -EPERM;
    }

    if (isFault()) {
        LOG_ERR("Cannot fire: fault detected");
        return -EIO;
    }

    LOG_INF("Firing pyro channel for %u ms", durationMs);
    int ret = gpio_pin_set_dt(&chEn, 1);
    if (ret != 0) {
        LOG_ERR("Failed to assert pyro enable: %d", ret);
        return ret;
    }

    k_msleep(durationMs);

    ret = gpio_pin_set_dt(&chEn, 0);
    if (ret != 0) {
        LOG_ERR("Failed to deassert pyro enable: %d", ret);
        return ret;
    }

    fired = true;
    LOG_INF("Pyro channel fire complete");
    return 0;
}

int PyroController::disarm() {
    armed = false;

    const int ret = gpio_pin_set_dt(&chEn, 0);
    if (ret != 0) {
        LOG_ERR("Failed to deassert pyro enable on disarm: %d", ret);
        return ret;
    }

    LOG_INF("Pyro channel disarmed");
    return 0;
}

bool PyroController::isFault() const {
    if (!hasFltPin()) {
        return false;
    }

    return gpio_pin_get_dt(&flt) > 0;
}

bool PyroController::isArmed() const { return armed; }

bool PyroController::hasFired() const { return fired; }

bool PyroController::hasFltPin() const { return flt.port != nullptr; }
