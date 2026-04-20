#pragma once

#include <core/sensors/Barometer.h>
#include <core/sensors/Imu.h>
#include <array>
#include <cstdint>
#include <functional>

enum class FlightState {
    PAD,
    BOOST,
    COAST,
    APOGEE,
    DESCENT,
    LANDED,
};

class FlightStateMachine {
  public:
    using StateChangeCallback = std::function<void(FlightState oldState, FlightState newState)>;

    FlightStateMachine(Barometer &barometer, Imu &imu);

    FlightState update(const ImuSample &imuSample, const BaroSample &baroSample);

    void onStateChange(StateChangeCallback callback);

    FlightState currentState() const;

  private:
    static constexpr float padBoostAccelGs = 2.0f;
    static constexpr uint32_t padBoostSustainedMs = 50U;
    static constexpr float padBoostAltitudeGainM = 0.25f;

    static constexpr float boostCoastAccelGs = 1.5f;

    static constexpr float coastApogeeAccelGs = 1.0f;
    static constexpr uint32_t apogeeInhibitAfterBoostMs = 1000U;
    static constexpr float apogeeBaroFilterAlpha = 0.1f;
    static constexpr float apogeeDescentDeltaM = 0.75f;
    static constexpr uint8_t apogeeDescendingSamplesRequired = 2U;

    static constexpr float descentLandedRestAccelGs = 1.0f;
    static constexpr float descentLandedAccelToleranceGs = 0.2f;
    static constexpr uint32_t descentLandedAccelSustainMs = 2000U;
    static constexpr uint32_t descentLandedAltitudeWindowMs = 4000U;
    static constexpr float descentLandedAltitudeDeltaM = 2.0f;
    static constexpr size_t descentLandedAltitudeHistoryCapacity = 512U;

    Barometer &barometer;
    Imu &imu;

    FlightState state{FlightState::PAD};
    StateChangeCallback stateChangeCallback{};

    uint32_t stateEntryTimeMs{0U};
    uint32_t boostEntryTimeMs{0U};

    uint32_t above2gStartMs{0U};
    bool above2gActive{false};

    uint32_t below01gStartMs{0U};
    bool below01gActive{false};

    float padAltitudeM{0.0f};
    bool havePadAltitude{false};
    float maxAltitudeM{0.0f};
    uint8_t descendingSampleCount{0U};

    float lastPressureKPa{0.0f};
    bool haveLastPressure{false};
    float filteredPressureKPa{0.0f};
    bool haveFilteredPressure{false};
    std::array<uint32_t, descentLandedAltitudeHistoryCapacity> descentAltitudeHistoryTimesMs{};
    std::array<float, descentLandedAltitudeHistoryCapacity> descentAltitudeHistoryM{};
    size_t descentAltitudeHistoryHead{0U};
    size_t descentAltitudeHistoryCount{0U};

    static float accelMagnitudeG(const ImuSample &sample);
    static float pressureKPaToAltitudeM(float pressureKPa);
    static float pressureKPa(const BaroSample &sample);
    static uint32_t elapsedMs(uint32_t now, uint32_t then);
    void resetDescentAltitudeHistory();
    void appendDescentAltitudeSample(uint32_t nowMs, float altitudeM);
    bool descentAltitudeWindowFull(uint32_t nowMs) const;
    float descentAltitudeRangeM() const;

    void transitionTo(FlightState next, uint32_t nowMs);
};
