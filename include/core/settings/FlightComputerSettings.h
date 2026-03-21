/*
* Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>

namespace FlightComputerSettings {

enum class DeployMode : uint8_t {
    DUAL_DEPLOY = 0, // Drogue at apogee, main at altitude
    DROGUE_ONLY = 1, // Drogue at apogee, no main fire
    MAIN_ONLY = 2,   // Main at apogee (single deploy)
};

constexpr DeployMode DEFAULT_DEPLOY_MODE = DeployMode::DUAL_DEPLOY;
constexpr uint16_t DEFAULT_MAIN_DEPLOY_ALT_M = 152; // 500 ft AGL
constexpr uint16_t DEFAULT_ARMING_ALT_M = 30;       // 100 ft — launch detect threshold
constexpr uint16_t DEFAULT_APOGEE_DELAY_MS = 0;     // fire drogue immediately at apogee
constexpr uint16_t DEFAULT_MIN_BATTERY_MV = 3300;   // 3.3 V lockout
constexpr uint32_t DEFAULT_FLIGHT_COUNTER = 0;

int load();
DeployMode deployMode();
uint16_t mainDeployAltM();
uint16_t armingAltM();
uint16_t apogeeDelayMs();
uint16_t minBatteryMv();
uint32_t flightCounter();
int setDeployMode(DeployMode mode);
int setMainDeployAltM(uint16_t altMeters);
int setArmingAltM(uint16_t altMeters);
int setApogeeDelayMs(uint16_t delayMs);
int setMinBatteryMv(uint16_t mv);
uint32_t incrementFlightCounter();

} // namespace FlightComputerSettings