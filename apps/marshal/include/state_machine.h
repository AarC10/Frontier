#pragma once

#include <stdint.h>
#include <zephyr/kernel.h>

#include "core/GnssReceiver.h"
#include "core/LoraTransceiver.h"

class StateMachine {
public:
#ifdef CONFIG_LICENSED_FREQUENCY
    explicit StateMachine(uint8_t nodeId, const float frequencyMHz = 903.0, const char callsign[6] = "");
#else
    explicit StateMachine(uint8_t nodeId, const float frequencyMHz = 903.0);
#endif
    void handleTxTimer();

    int run();

private:
    enum class State { Transmitter, Receiver };

    void enterTransmitter();
    void enterReceiver();
    void exitReceiver();
    void transitionTo(State target);
    int checkForTransition();

#ifdef CONFIG_LICENSED_FREQUENCY
    const char* callsign;
#endif
    LoraTransceiver lora;
    GnssReceiver gnssReceiver;
    k_timer txTimer{};
    uint8_t nodeId{};
    int lastPinSate{-1};
    State currentState{State::Transmitter};
};
