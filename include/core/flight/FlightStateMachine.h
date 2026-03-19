#pragma once

#include <core/sensors/Barometer.h>
#include <core/sensors/Imu.h>

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

	static constexpr float boostCoastAccelGs = 1.5f;

	static constexpr float coastApogeeAccelGs = 1.0f;
	static constexpr uint32_t apogeeInhibitAfterBoostMs = 1000U;

	static constexpr float descentLandedAccelGs = 0.1f;
	static constexpr uint32_t descentLandedAccelSustainMs = 3000U;
	static constexpr uint32_t descentLandedAltitudeWindowMs = 5000U;
	static constexpr float descentLandedAltitudeDeltaM = 1.0f;

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

	uint32_t descentAltitudeAnchorTimeMs{0U};
	float descentAltitudeAnchorM{0.0f};

	float lastPressureKPa{0.0f};
	bool haveLastPressure{false};

	static float accelMagnitudeG(const ImuSample &sample);
	static float pressureKPaToAltitudeM(float pressureKPa);
	static float pressureKPa(const BaroSample &sample);
	static uint32_t elapsedMs(uint32_t now, uint32_t then);

	void transitionTo(FlightState next, uint32_t nowMs);
};
