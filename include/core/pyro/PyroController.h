/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>
#include <zephyr/drivers/gpio.h>

/**
 * @brief Controls a single pyrotechnic output channel.
 *
 * Manages arm/fire/disarm sequencing for one pyro channel via a GPIO enable
 * pin and an optional fault (FLT) sense pin. A second optional ADC-backed
 * device can be supplied for igniter-line monitoring (ILM) continuity reads,
 * but the class operates safely without it.
 *
 * Typical usage:
 *   PyroController ch1(chEnSpec, fltSpec);
 *   ch1.init();
 *   ch1.arm();
 *   ch1.fire(50);   // assert enable for 50 ms
 *   ch1.disarm();
 */
class PyroController {
  public:
    /**
     * @brief Construct a pyro channel controller.
     *
     * @param chEn   GPIO spec for the channel enable output (active-high fires
     *               the squib).
     * @param flt    GPIO spec for the fault sense input (active-low fault
     *               signal). Pass a zeroed spec to skip fault sensing.
     */
    PyroController(const gpio_dt_spec &chEn, const gpio_dt_spec &flt);

    /**
     * @brief Configure GPIO pins.  Must be called before any other method.
     * @return 0 on success, negative errno on failure.
     */
    int init();

    /**
     * @brief Arm the channel (allow firing).
     * @return 0 on success, negative errno on failure.
     */
    int arm();

    /**
     * @brief Assert the channel enable pin for @p durationMs milliseconds,
     *        then deassert it.  The channel must be armed first.
     * @param durationMs Duration to hold the enable line active (milliseconds).
     * @return 0 on success, -EPERM if not armed, other negative errno on GPIO
     *         failure.
     */
    int fire(uint32_t durationMs);

    /**
     * @brief Disarm the channel and ensure the enable pin is deasserted.
     * @return 0 on success, negative errno on failure.
     */
    int disarm();

    /**
     * @brief Read the fault pin.
     * @return true if a fault is detected (FLT pin asserted), false otherwise
     *         or if no fault pin was provided.
     */
    bool isFault() const;

    /** @brief Returns true if the channel has been armed. */
    bool isArmed() const;

    /** @brief Returns true if the channel has been fired at least once. */
    bool hasFired() const;

  private:
    const gpio_dt_spec chEn;
    const gpio_dt_spec flt;

    bool armed{false};
    bool fired{false};

    bool hasFltPin() const;
};
