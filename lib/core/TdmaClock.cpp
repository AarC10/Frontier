#include <core/TdmaClock.h>

#include <zephyr/drivers/counter.h>
#include <zephyr/logging/log.h>

#include <utility>

LOG_MODULE_REGISTER(tdma_clock, LOG_LEVEL_INF);

namespace {
constexpr uint32_t frameLenMs = 1000;
constexpr uint32_t gpsDemoteMs = 5000;
constexpr uint32_t hunterStaleMs = 30000;
}

TdmaClock& TdmaClock::instance() {
    static TdmaClock clock;
    return clock;
}

void TdmaClock::init(const gpio_dt_spec* pps, const device* tim2Dev) {
    ppsSpec = pps;
    tim2 = tim2Dev;
    ppsConfigured = false;

    atomic_set(&currentSource, static_cast<atomic_val_t>(Source::FREERUN));
    atomic_set(&epochTicksValue, 0);
    atomic_set(&frameNumberValue, 0);
    atomic_set(&lastHunterUptimeMs, 0);

    k_timer_init(&freerunTimer, TdmaClock::freerunExpiry, nullptr);
    k_work_init_delayable(&demoteWork, TdmaClock::demoteHandler);

    if (ppsSpec != nullptr && device_is_ready(ppsSpec->port)) {
        if (gpio_pin_configure_dt(ppsSpec, GPIO_INPUT) == 0 &&
            gpio_pin_interrupt_configure_dt(ppsSpec, GPIO_INT_EDGE_RISING) == 0) {
            gpio_init_callback(&ppsCallback, TdmaClock::ppsIsr, BIT(ppsSpec->pin));
            if (gpio_add_callback(ppsSpec->port, &ppsCallback) == 0) {
                ppsConfigured = true;
            }
        }
    }

    if (tim2 == nullptr || !device_is_ready(tim2)) {
        LOG_WRN("TDMA timer device not ready, epoch ticks will remain 0");
    }

    startFreerun();
    scheduleDemote(K_MSEC(gpsDemoteMs));

    if (!ppsConfigured) {
        LOG_WRN("PPS not configured, starting in FREERUN");
    }
}

TdmaClock::Source TdmaClock::source() const {
    return static_cast<Source>(atomic_get(&currentSource));
}

uint32_t TdmaClock::epochTicks() const {
    return static_cast<uint32_t>(atomic_get(&epochTicksValue));
}

uint32_t TdmaClock::frameNumber() const {
    return static_cast<uint32_t>(atomic_get(&frameNumberValue));
}

void TdmaClock::onHunterBeacon(uint32_t beaconFrameNumber, uint32_t timestamp) {
    atomic_set(&frameNumberValue, static_cast<atomic_val_t>(beaconFrameNumber));
    atomic_set(&epochTicksValue, static_cast<atomic_val_t>(timestamp));
    atomic_set(&lastHunterUptimeMs, static_cast<atomic_val_t>(k_uptime_get_32()));

    if (source() != Source::GPS_PPS) {
        atomic_set(&currentSource, static_cast<atomic_val_t>(Source::HUNTER));
        stopFreerun();
        scheduleDemote(K_MSEC(hunterStaleMs));
    }
}

void TdmaClock::ppsIsr(const device* dev, gpio_callback* cb, uint32_t pins) {
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    TdmaClock& clock = TdmaClock::instance();

    atomic_set(&clock.epochTicksValue, static_cast<atomic_val_t>(clock.readTim2Ticks()));
    atomic_inc(&clock.frameNumberValue);
    atomic_set(&clock.currentSource, static_cast<atomic_val_t>(Source::GPS_PPS));

    clock.stopFreerun();
    (void)k_work_reschedule(&clock.demoteWork, K_MSEC(gpsDemoteMs));
}

void TdmaClock::freerunExpiry(k_timer* timer) {
    ARG_UNUSED(timer);

    TdmaClock& clock = TdmaClock::instance();
    if (clock.source() != Source::FREERUN) {
        return;
    }

    atomic_set(&clock.epochTicksValue, static_cast<atomic_val_t>(clock.readTim2Ticks()));
    atomic_inc(&clock.frameNumberValue);
}

void TdmaClock::demoteHandler(k_work* work) {
    ARG_UNUSED(work);

    TdmaClock& clock = TdmaClock::instance();
    const Source now = clock.source();

    if (now == Source::GPS_PPS) {
        const uint32_t nowMs = k_uptime_get_32();
        const uint32_t lastHunter = static_cast<uint32_t>(atomic_get(&clock.lastHunterUptimeMs));
        const uint32_t hunterAgeMs = nowMs - lastHunter;
        const bool hunterFresh = (lastHunter != 0U) && (hunterAgeMs < hunterStaleMs);

        if (hunterFresh) {
            atomic_set(&clock.currentSource, std::to_underlying(Source::HUNTER));
            clock.stopFreerun();
            clock.scheduleDemote(K_MSEC(hunterStaleMs - hunterAgeMs));
        } else {
            atomic_set(&clock.currentSource, std::to_underlying(Source::FREERUN));
            clock.startFreerun();
        }
        return;
    }

    if (now == Source::HUNTER) {
        atomic_set(&clock.currentSource, std::to_underlying(Source::FREERUN));
        clock.startFreerun();
    }
}

void TdmaClock::startFreerun() {
    k_timer_start(&freerunTimer, K_NO_WAIT, K_MSEC(frameLenMs));
}

void TdmaClock::stopFreerun() {
    k_timer_stop(&freerunTimer);
}

uint32_t TdmaClock::readTim2Ticks() const {
    if (tim2 == nullptr || !device_is_ready(tim2)) {
        return 0;
    }

    uint32_t ticks = 0;
    if (counter_get_value(tim2, &ticks) != 0) {
        return 0;
    }

    return ticks;
}

void TdmaClock::scheduleDemote(k_timeout_t delay) {
    (void)k_work_reschedule(&demoteWork, delay);
}

