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
constexpr uint16_t DEFAULT_MAIN_DEPLOY_ALT_FT = 500; // ft AGL
constexpr uint16_t DEFAULT_ARMING_ALT_FT = 100;      // ft — launch detect threshold
constexpr uint16_t DEFAULT_APOGEE_DELAY_MS = 0;      // fire drogue immediately at apogee
constexpr uint16_t DEFAULT_MIN_BATTERY_MV = 3300;    // 3.3 V lockout
constexpr uint32_t DEFAULT_FLIGHT_COUNTER = 0;
constexpr float FT_TO_M = 0.3048f;

int load();
DeployMode deployMode();
uint16_t mainDeployAltFt();
uint16_t armingAltFt();
uint16_t apogeeDelayMs();
uint16_t minBatteryMv();
uint32_t flightCounter();
int setDeployMode(DeployMode mode);
int setMainDeployAltFt(uint16_t altFeet);
int setArmingAltFt(uint16_t altFeet);
int setApogeeDelayMs(uint16_t delayMs);
int setMinBatteryMv(uint16_t mv);
uint32_t incrementFlightCounter();
float mainDeployAltM();
float armingAltM();
} // namespace FlightComputerSettings