#include "core/flight/FlightStateMachine.h"

#include <cmath>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#include "zephyr/logging/log.h"

LOG_MODULE_REGISTER(FlightStateMachine);

FlightStateMachine::FlightStateMachine(Barometer &barometerRef, Imu &imuRef) : barometer(barometerRef), imu(imuRef) {
    const uint32_t now = k_uptime_get_32();
    stateEntryTimeMs = now;
    descentAltitudeAnchorTimeMs = now;
}

FlightState FlightStateMachine::update(const ImuSample &imuSample, const BaroSample &baroSample) {
    const uint32_t now = k_uptime_get_32();
    const float accelG = accelMagnitudeG(imuSample);
    const float currentPressureKPa = pressureKPa(baroSample);
    const float currentAltitudeM = pressureKPaToAltitudeM(currentPressureKPa);

    const bool pressureIncreasing = haveLastPressure && (currentPressureKPa > lastPressureKPa);

    switch (state) {
        case FlightState::PAD:
            if (accelG > padBoostAccelGs) {
                if (!above2gActive) {
                    above2gActive = true;
                    above2gStartMs = now;
                    LOG_INF("Above %.1f G detected, starting timer", padBoostAccelGs);
                    LOG_INF("Accel: %.2f G, Pressure: %.3f kPa, Altitude: %.2f m", accelG, currentPressureKPa, currentAltitudeM);
                }
                if (elapsedMs(now, above2gStartMs) >= padBoostSustainedMs) {
                    LOG_INF("Launch detected, transitioning to BOOST");
                    transitionTo(FlightState::BOOST, now);
                    boostEntryTimeMs = now;
                    above2gActive = false;
                }
            } else {
                above2gActive = false;
            }
            break;

        case FlightState::BOOST:
            if (accelG < boostCoastAccelGs) {
                transitionTo(FlightState::COAST, now);
            }
            break;

        case FlightState::COAST: {
            const bool inhibitApogee = elapsedMs(now, boostEntryTimeMs) < apogeeInhibitAfterBoostMs;
            const bool accelCondition = accelG < coastApogeeAccelGs;
            const bool dualSensorApogee = accelCondition && pressureIncreasing;
            if (!inhibitApogee && dualSensorApogee) {
                transitionTo(FlightState::APOGEE, now);
            }
            break;
        }

        case FlightState::APOGEE:
            transitionTo(FlightState::DESCENT, now);
            below01gActive = false;
            descentAltitudeAnchorTimeMs = now;
            descentAltitudeAnchorM = currentAltitudeM;
            break;

        case FlightState::DESCENT: {
            if (accelG < descentLandedAccelGs) {
                if (!below01gActive) {
                    below01gActive = true;
                    below01gStartMs = now;
                }
            } else {
                below01gActive = false;
            }

            if (elapsedMs(now, descentAltitudeAnchorTimeMs) >= descentLandedAltitudeWindowMs) {
                descentAltitudeAnchorTimeMs = now;
                descentAltitudeAnchorM = currentAltitudeM;
            }

            const float altitudeDeltaM = std::fabs(currentAltitudeM - descentAltitudeAnchorM);
            const bool accelStable = below01gActive && (elapsedMs(now, below01gStartMs) >= descentLandedAccelSustainMs);
            const bool altitudeStable = elapsedMs(now, descentAltitudeAnchorTimeMs) >= descentLandedAltitudeWindowMs &&
                                        altitudeDeltaM < descentLandedAltitudeDeltaM;

            if (accelStable && altitudeStable) {
                transitionTo(FlightState::LANDED, now);
            }
            break;
        }

        case FlightState::LANDED:
            break;
    }

    lastPressureKPa = currentPressureKPa;
    haveLastPressure = true;

    return state;
}

void FlightStateMachine::onStateChange(StateChangeCallback callback) { stateChangeCallback = std::move(callback); }

FlightState FlightStateMachine::currentState() const { return state; }

float FlightStateMachine::accelMagnitudeG(const ImuSample &sample) {
    const double ax = sensor_value_to_double(&sample.accelX);
    const double ay = sensor_value_to_double(&sample.accelY);
    const double az = sensor_value_to_double(&sample.accelZ);
    const double magnitudeMps2 = std::sqrt(ax * ax + ay * ay + az * az);
    sensor_value magnitude{};
    const int ret = sensor_value_from_double(&magnitude, magnitudeMps2);
    const double magnitudeG = (ret == 0) ? static_cast<double>(sensor_ms2_to_ug(&magnitude)) / 1000000.0 : 0.0;
    LOG_INF("Accel magnitude: %.2f G (ax=%.2f, ay=%.2f, az=%.2f m/s^2)", magnitudeG, ax, ay, az);
    return static_cast<float>(magnitudeG);
}

float FlightStateMachine::pressureKPaToAltitudeM(float pressureKPa) {
    return 44330.0f * (1.0f - std::pow(pressureKPa / 101.325f, 1.0f / 5.255f));
}

float FlightStateMachine::pressureKPa(const BaroSample &sample) {
    return sensor_value_to_float(&sample.pressure) / 1000.0f;
}

uint32_t FlightStateMachine::elapsedMs(uint32_t now, uint32_t then) { return now - then; }

void FlightStateMachine::transitionTo(FlightState next, uint32_t nowMs) {
    if (state != next) {
        const FlightState oldState = state;
        state = next;
        stateEntryTimeMs = nowMs;
        if (stateChangeCallback) {
            stateChangeCallback(oldState, next);
        }
    }
}
