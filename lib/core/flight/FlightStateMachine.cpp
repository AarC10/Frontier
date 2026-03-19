#include "core/flight/FlightStateMachine.h"

#include <cmath>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

FlightStateMachine::FlightStateMachine(Barometer &barometerRef, Imu &imuRef)
    : barometer(barometerRef), imu(imuRef) {
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
            }
            if (elapsedMs(now, above2gStartMs) >= padBoostSustainedMillis) {
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
        const bool inhibitApogee = elapsedMs(now, boostEntryTimeMs) < apogeeInhibitAfterBoostMillis;
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

        if (elapsedMs(now, descentAltitudeAnchorTimeMs) >= descentLandedAltitudeWindowMillis) {
            descentAltitudeAnchorTimeMs = now;
            descentAltitudeAnchorM = currentAltitudeM;
        }

        const float altitudeDeltaM = std::fabs(currentAltitudeM - descentAltitudeAnchorM);
        const bool accelStable = below01gActive &&
                                 (elapsedMs(now, below01gStartMs) >= descentLandedAccelSustainMillis);
        const bool altitudeStable = elapsedMs(now, descentAltitudeAnchorTimeMs) >=
                                        descentLandedAltitudeWindowMillis &&
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

void FlightStateMachine::onStateChange(StateChangeCallback callback) {
    stateChangeCallback = std::move(callback);
}

FlightState FlightStateMachine::currentState() const {
    return state;
}

float FlightStateMachine::accelMagnitudeG(const ImuSample &sample) {
    const double ax = sensor_value_to_double(&sample.accelX);
    const double ay = sensor_value_to_double(&sample.accelY);
    const double az = sensor_value_to_double(&sample.accelZ);
    const double magnitude = std::sqrt(ax * ax + ay * ay + az * az);
    return static_cast<float>(magnitude);
}
