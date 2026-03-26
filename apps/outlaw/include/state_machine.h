#pragma once

#include "core/GnssReceiver.h"
#include "core/LoraTransceiver.h"

#include <stdint.h>
#include <zephyr/kernel.h>

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

    void doTx();

    friend void txWorkHandler(struct k_work *work);

#ifdef CONFIG_LICENSED_FREQUENCY
    const char *callsign;
#endif
    LoraTransceiver lora;
    GnssReceiver gnssReceiver;
    k_timer txTimer{};
    k_work txWork{};
    k_spinlock dataLock{};
    gnss_data pendingGnssData{};
    bool pendingHasFix{false};
    uint8_t nodeId{};
    int lastPinSate{-1};
    State currentState{State::Transmitter};
};
