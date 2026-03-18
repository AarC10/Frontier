#pragma once

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

class TdmaClock {
public:
    enum class Source : uint8_t {
        FREERUN = 0,
        HUNTER = 1,
        GPS_PPS = 2,
    };

    static TdmaClock& instance();

    void init(const gpio_dt_spec* pps, const device* tim2Dev);
    Source source() const;
    uint32_t epochTicks() const;
    uint32_t frameNumber() const;

    void onHunterBeacon(uint32_t frameNumber, uint32_t timestamp);

private:
    TdmaClock() = default;

    static void ppsIsr(const device* dev, gpio_callback* cb, uint32_t pins);
    static void freerunExpiry(k_timer* timer);
    static void demoteHandler(k_work* work);

    void startFreerun();
    void stopFreerun();
    uint32_t readTim2Ticks() const;
    void scheduleDemote(k_timeout_t delay);

    atomic_t currentSource;
    atomic_t epochTicksValue;
    atomic_t frameNumberValue;
    atomic_t lastHunterUptimeMs;

    k_timer freerunTimer;
    k_work_delayable demoteWork;
    gpio_callback ppsCallback;

    const gpio_dt_spec* ppsSpec;
    const device* tim2;
    bool ppsConfigured;
};

