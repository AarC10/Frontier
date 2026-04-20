#include "core/flight/FlightStateMachine.h"

#include <cmath>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#include "zephyr/logging/log.h"

LOG_MODULE_REGISTER(FlightStateMachine);

FlightStateMachine::FlightStateMachine(Barometer &barometerRef, Imu &imuRef) : barometer(barometerRef), imu(imuRef) {
    const uint32_t now = k_uptime_get_32();
    stateEntryTimeMs = now;
}

FlightState FlightStateMachine::update(const ImuSample &imuSample, const BaroSample &baroSample) {
    const uint32_t now = k_uptime_get_32();
    const float accelG = accelMagnitudeG(imuSample);
    const float currentPressureKPa = pressureKPa(baroSample);
    const float currentAltitudeM = pressureKPaToAltitudeM(currentPressureKPa);
    if (!haveFilteredPressure) {
        filteredPressureKPa = currentPressureKPa;
        haveFilteredPressure = true;
    } else {
        filteredPressureKPa += (currentPressureKPa - filteredPressureKPa) * apogeeBaroFilterAlpha;
    }
    const float filteredAltitudeM = pressureKPaToAltitudeM(filteredPressureKPa);
    const bool pressureIncreasing = haveLastPressure && (filteredPressureKPa > lastPressureKPa);

    if (!havePadAltitude) {
        padAltitudeM = currentAltitudeM;
        havePadAltitude = true;
    }

    const float altitudeAglM = currentAltitudeM - padAltitudeM;

    switch (state) {
        case FlightState::PAD:
            if (accelG > padBoostAccelGs) {
                if (!above2gActive) {
                    above2gActive = true;
                    above2gStartMs = now;
                    // LOG_INF("Above %.1f G detected, starting timer", padBoostAccelGs);
                    // LOG_INF("Accel: %.2f G, Pressure: %.3f kPa, Altitude: %.2f m", accelG, currentPressureKPa, currentAltitudeM);
                }
                if (elapsedMs(now, above2gStartMs) >= padBoostSustainedMs && altitudeAglM >= padBoostAltitudeGainM) {
                    LOG_INF("Launch detected, transitioning to BOOST");
                    transitionTo(FlightState::BOOST, now);
                    boostEntryTimeMs = now;
                    above2gActive = false;
                    maxAltitudeM = filteredAltitudeM;
                    descendingSampleCount = 0U;
                }
            } else {
                above2gActive = false;
                padAltitudeM = currentAltitudeM;
            }
            break;

        case FlightState::BOOST:
            if (filteredAltitudeM > maxAltitudeM) {
                maxAltitudeM = filteredAltitudeM;
            }
            if (accelG < boostCoastAccelGs) {
                transitionTo(FlightState::COAST, now);
            }
            break;

        case FlightState::COAST: {
            const bool inhibitApogee = elapsedMs(now, boostEntryTimeMs) < apogeeInhibitAfterBoostMs;
            const bool accelCondition = accelG < coastApogeeAccelGs;
            if (filteredAltitudeM > maxAltitudeM) {
                maxAltitudeM = filteredAltitudeM;
                descendingSampleCount = 0U;
            } else if ((maxAltitudeM - filteredAltitudeM) >= apogeeDescentDeltaM && pressureIncreasing) {
                if (descendingSampleCount < apogeeDescendingSamplesRequired) {
                    ++descendingSampleCount;
                }
            } else {
                descendingSampleCount = 0U;
            }

            const bool descentConfirmed = descendingSampleCount >= apogeeDescendingSamplesRequired;
            const bool dualSensorApogee = accelCondition && descentConfirmed;
            if (!inhibitApogee && dualSensorApogee) {
                transitionTo(FlightState::APOGEE, now);
            }
            break;
        }

        case FlightState::APOGEE:
            transitionTo(FlightState::DESCENT, now);
            below01gActive = false;
            resetDescentAltitudeHistory();
            break;

        case FlightState::DESCENT: {
            const bool nearRestAccel =
                std::fabs(accelG - descentLandedRestAccelGs) <= descentLandedAccelToleranceGs;
            float filteredAltitudeAglM = filteredAltitudeM - padAltitudeM;
            if (filteredAltitudeAglM < 0.0f) {
                filteredAltitudeAglM = 0.0f;
            }
            appendDescentAltitudeSample(now, filteredAltitudeAglM);

            if (nearRestAccel) {
                if (!below01gActive) {
                    below01gActive = true;
                    below01gStartMs = now;
                }
            } else {
                below01gActive = false;
            }

            const bool accelStable = below01gActive && (elapsedMs(now, below01gStartMs) >= descentLandedAccelSustainMs);
            const bool altitudeStable =
                descentAltitudeWindowFull(now) && (descentAltitudeRangeM() < descentLandedAltitudeDeltaM);

            if (accelStable && altitudeStable) {
                transitionTo(FlightState::LANDED, now);
            }
            break;
        }

        case FlightState::LANDED:
            break;
    }

    lastPressureKPa = filteredPressureKPa;
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
    // LOG_INF("Accel magnitude: %.2f G (ax=%.2f, ay=%.2f, az=%.2f m/s^2)", magnitudeG, ax, ay, az);
    return static_cast<float>(magnitudeG);
}

float FlightStateMachine::pressureKPaToAltitudeM(float pressureKPa) {
    return 44330.0f * (1.0f - std::pow(pressureKPa / 101.325f, 1.0f / 5.255f));
}

float FlightStateMachine::pressureKPa(const BaroSample &sample) {
    // MS56xx driver already reports SENSOR_CHAN_PRESS in kPa; no extra scaling.
    return sensor_value_to_float(&sample.pressure);
}

uint32_t FlightStateMachine::elapsedMs(uint32_t now, uint32_t then) { return now - then; }

void FlightStateMachine::resetDescentAltitudeHistory() {
    descentAltitudeHistoryHead = 0U;
    descentAltitudeHistoryCount = 0U;
}

void FlightStateMachine::appendDescentAltitudeSample(uint32_t nowMs, float altitudeM) {
    while (descentAltitudeHistoryCount > 0U &&
           elapsedMs(nowMs, descentAltitudeHistoryTimesMs[descentAltitudeHistoryHead]) > descentLandedAltitudeWindowMs) {
        descentAltitudeHistoryHead = (descentAltitudeHistoryHead + 1U) % descentLandedAltitudeHistoryCapacity;
        --descentAltitudeHistoryCount;
    }

    if (descentAltitudeHistoryCount == descentLandedAltitudeHistoryCapacity) {
        descentAltitudeHistoryHead = (descentAltitudeHistoryHead + 1U) % descentLandedAltitudeHistoryCapacity;
        --descentAltitudeHistoryCount;
    }

    const size_t tail =
        (descentAltitudeHistoryHead + descentAltitudeHistoryCount) % descentLandedAltitudeHistoryCapacity;
    descentAltitudeHistoryTimesMs[tail] = nowMs;
    descentAltitudeHistoryM[tail] = altitudeM;
    ++descentAltitudeHistoryCount;
}

bool FlightStateMachine::descentAltitudeWindowFull(uint32_t nowMs) const {
    if (descentAltitudeHistoryCount == 0U) {
        return false;
    }

    return elapsedMs(nowMs, descentAltitudeHistoryTimesMs[descentAltitudeHistoryHead]) >= descentLandedAltitudeWindowMs;
}

float FlightStateMachine::descentAltitudeRangeM() const {
    if (descentAltitudeHistoryCount == 0U) {
        return 0.0f;
    }

    float minAltitudeM = descentAltitudeHistoryM[descentAltitudeHistoryHead];
    float maxAltitudeM = minAltitudeM;

    for (size_t index = 1U; index < descentAltitudeHistoryCount; ++index) {
        const size_t sampleIndex = (descentAltitudeHistoryHead + index) % descentLandedAltitudeHistoryCapacity;
        const float altitudeM = descentAltitudeHistoryM[sampleIndex];
        if (altitudeM < minAltitudeM) {
            minAltitudeM = altitudeM;
        }
        if (altitudeM > maxAltitudeM) {
            maxAltitudeM = altitudeM;
        }
    }

    return maxAltitudeM - minAltitudeM;
}

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
