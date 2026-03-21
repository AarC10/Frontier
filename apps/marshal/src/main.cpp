/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <core/Settings.h>
#include <core/flight/FlightStateMachine.h>
#include <core/flight_logger/FlightLogger.h>
#include <core/sensors/Barometer.h>
#include <core/sensors/Imu.h>
#include <core/sensors/VoltageMonitor.h>
#include <core/settings/FlightComputerSettings.h>
#include <utility>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/storage/flash_map.h>

LOG_MODULE_REGISTER(marshal, CONFIG_LOG_DEFAULT_LEVEL);

// GLOBALS
static Barometer barometer(DEVICE_DT_GET(DT_ALIAS(barometer)));
static Imu imu(DEVICE_DT_GET(DT_ALIAS(imu)));
static VoltageMonitor voltageMonitor(DEVICE_DT_GET(DT_ALIAS(vbat_sensor)), DEVICE_DT_GET(DT_ALIAS(vcc_sensor)));
static FlightLogger logger(PARTITION(raw_partition));
static bool armed = false;

static void imuDataReadyHandler(const device *dev, const sensor_trigger *trig) {
    ARG_UNUSED(dev);
    ARG_UNUSED(trig);

    const ImuSample sample = imu.sample();
    logger.logImu(sample);
}

#define BARO_STACK_SIZE 1024
#define BARO_PRIORITY   4
K_THREAD_STACK_DEFINE(baroStack, BARO_STACK_SIZE);
static k_thread baroThread;

static void baroThreadEntry(void *, void *, void *) {
    while (true) {
        const BaroSample sample = barometer.sample();
        logger.logBaro(sample);
        k_sleep(K_MSEC(40)); // 25 Hz
    }
}

// VOLTAGE MONITOR THREAD
#define VOLTAGE_STACK_SIZE 512
#define VOLTAGE_PRIORITY   10
K_THREAD_STACK_DEFINE(voltageStack, VOLTAGE_STACK_SIZE);
static k_thread voltageThread;

static void voltageThreadEntry(void *, void *, void *) {
    while (true) {
        voltageMonitor.sample();
        logger.logVoltage(static_cast<uint16_t>(voltageMonitor.vbatMv()), static_cast<uint16_t>(voltageMonitor.vccMv()),
                          0, 0 // TODO: read pyro ILM channels
        );

        k_sleep(K_MSEC(1000)); // 1 Hz
    }
}

static bool checkBatteryVoltage() {
    voltageMonitor.sample();
    const uint16_t vbat = static_cast<uint16_t>(voltageMonitor.vbatMv());
    const uint16_t threshold = FlightComputerSettings::minBatteryMv();

    if (vbat < threshold) {
        LOG_ERR("Battery voltage %u mV below lockout threshold %u mV — NOT ARMING", vbat, threshold);
        // TODO: signal error here
        return false;
    }

    LOG_INF("Battery voltage OK: %u mV (threshold %u mV)", vbat, threshold);
    return true;
}

int main() {
    int ret = FlightComputerSettings::load();
    if (ret != 0) {
        LOG_ERR("Failed to load settings: %d", ret);
    }

    ret = barometer.init();
    if (ret != 0) {
        LOG_ERR("Barometer init failed: %d", ret);
    }

    ret = imu.init(imuDataReadyHandler);
    if (ret != 0) {
        LOG_ERR("IMU init failed: %d", ret);
    }

    ret = voltageMonitor.init();
    if (ret != 0) {
        LOG_ERR("Voltage monitor init failed: %d", ret);
    }

    // Battery lockout and refuse to arm if voltage is too low
    armed = checkBatteryVoltage();

    // Init flight logger
    ret = logger.init();
    if (ret != 0) {
        LOG_ERR("Flight logger init failed: %d", ret);
        // TODO: set a fault LED / buzzer pattern here.
    }

    // Init FSM
    static FlightStateMachine fsm(barometer, imu);

    // TODO: feed FlightComputerSettings::armingAltM() and mainDeployAltM() into the FSM

    fsm.onStateChange([](FlightState oldState, FlightState newState) {
        logger.logStateChange(static_cast<uint8_t>(oldState), static_cast<uint8_t>(newState));

        if (newState == FlightState::BOOST && armed) {
            const uint32_t id = FlightComputerSettings::incrementFlightCounter();
            logger.startFlight(id, 100, 25);
            LOG_INF("Flight %u logging started", id);
        }

        if (newState == FlightState::LANDED) {
            logger.endFlight();
        }

        // TODO: PyroController logic
        // DUAL_DEPLOY:
        //   APOGEE: delay(apogeeDelayMs) → fire CH1 (drogue)
        //   DESCENT: altitude < mainDeployAltFt → fire CH2 (main)
        //
        // DROGUE_ONLY:
        //   APOGEE: delay(apogeeDelayMs) → fire CH1 (drogue)
        //
        // MAIN_ONLY:
        //   APOGEE: fire CH2 (main)
    });

    // Start background threads
    k_thread_create(&baroThread, baroStack, BARO_STACK_SIZE, baroThreadEntry, nullptr, nullptr, nullptr, BARO_PRIORITY,
                    0, K_NO_WAIT);

    k_thread_create(&voltageThread, voltageStack, VOLTAGE_STACK_SIZE, voltageThreadEntry, nullptr, nullptr, nullptr,
                    VOLTAGE_PRIORITY, 0, K_NO_WAIT);

    LOG_INF("READY: armed=%s mode=%u main_alt=%u ft log=%u/%u bytes", armed ? "YES" : "NO",
            static_cast<unsigned>(FlightComputerSettings::deployMode()), FlightComputerSettings::mainDeployAltFt(),
            logger.writeOffset(), logger.partitionSize());

    return 0;
}