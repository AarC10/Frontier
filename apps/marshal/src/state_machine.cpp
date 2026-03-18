#include "state_machine.h"

#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

static constexpr int TRANSMITTER_LOGIC_LEVEL = 0;
static constexpr int TRANSMITTER_LED_LEVEL = 0;

static constexpr int RECEIVER_LOGIC_LEVEL = 1;
static constexpr int RECEIVER_LED_LEVEL = 1;

LOG_MODULE_REGISTER(state_machine);

static const gpio_dt_spec dip0 = GPIO_DT_SPEC_GET(DT_ALIAS(dip0), gpios);
static const gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static void txTimerCallback(struct k_timer* timer) {
    if (auto* sm = static_cast<StateMachine*>(k_timer_user_data_get(timer))) {
        sm->handleTxTimer();
    }
}

#ifdef CONFIG_LICENSED_FREQUENCY
StateMachine::StateMachine(uint8_t nodeId, const float frequencyMHz, const char* callsign) :  callsign(callsign), lora(nodeId, frequencyMHz), nodeId(nodeId) {
    k_timer_init(&txTimer, txTimerCallback, nullptr);
    k_timer_user_data_set(&txTimer, this);
    lora.setCallsign(callsign);

#ifdef CONFIG_DEFAULT_RECEIVE_MODE
    currentState = State::Receiver;
    enterReceiver();
#else
    currentState = State::Transmitter;
    enterTransmitter();
#endif

    setGnssReciever(&gnssReceiver);
}

#else

StateMachine::StateMachine(const uint8_t nodeId, const float frequencyMhz) :  lora(nodeId, frequencyMhz), nodeId(nodeId) {
    k_timer_init(&txTimer, txTimerCallback, nullptr);
    k_timer_user_data_set(&txTimer, this);

#ifdef CONFIG_DEFAULT_RECEIVE_MODE
    currentState = State::Receiver;
    enterReceiver();
#else
    currentState = State::Transmitter;
    enterTransmitter();
#endif

    setGnssReciever(&gnssReceiver);
}

#endif


void StateMachine::handleTxTimer() {
    if (gnssReceiver.isFixAcquired()) {
        lora.txGnssPayload(gnssReceiver.getLatestData());
    } else {
        lora.txNoFixPayload();
    }
}

int StateMachine::run() {
    return checkForTransition();
}

void StateMachine::enterTransmitter() {
    LOG_INF("Entering transmitter state");
    lora.setTx();

    gpio_pin_set_dt(&led, TRANSMITTER_LED_LEVEL);
    k_timer_start(&txTimer, K_SECONDS(5), K_SECONDS(5));
}

void StateMachine::enterReceiver() {
    LOG_INF("Entering receiver state");
    lora.setRx();
    gpio_pin_set_dt(&led, RECEIVER_LED_LEVEL);
    k_timer_stop(&txTimer);
    lora.awaitRxPacket();
}

void StateMachine::exitReceiver() {
    lora.awaitCancel();
}

void StateMachine::transitionTo(State target) {
    if (target == currentState) {
        return;
    }

    if (currentState == State::Receiver) {
        exitReceiver();
    }

    currentState = target;

    if (currentState == State::Transmitter) {
        enterTransmitter();
    } else {
        enterReceiver();
    }
}

int StateMachine::checkForTransition() {
    const int current_pin_state = gpio_pin_get_dt(&dip0);
    LOG_DBG("Pin state: %d", current_pin_state);

    if (current_pin_state < 0) {
        LOG_DBG("dip0 gpio read error (%d); ignoring transition check", current_pin_state);
        return 0;
    }

    if (lastPinSate != current_pin_state) {
        if (current_pin_state == TRANSMITTER_LOGIC_LEVEL) {
            transitionTo(State::Transmitter);
        } else if (current_pin_state == RECEIVER_LOGIC_LEVEL) {
            transitionTo(State::Receiver);
        }
    }

    lastPinSate = current_pin_state;

    return 0;
}
