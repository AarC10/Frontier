/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cmath>
#include <core/flight/FlightStateMachine.h>
#include <core/flight_logger/FlightExporter.h>
#include <core/flight_logger/FlightLogger.h>
#include <core/io/Buzzer.h>
#include <core/io/Led.h>
#include <core/pyro/PyroController.h>
#include <core/sensors/Barometer.h>
#include <core/sensors/Imu.h>
#include <core/sensors/VoltageMonitor.h>
#include <core/settings/FlightComputerSettings.h>
#include <cstdlib>
#include <marshal/device_runtime.h>
#include <marshal/usb_support.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>
#ifdef CONFIG_SHELL
#include <zephyr/shell/shell.h>
#endif

#ifdef CONFIG_BOARD_NATIVE_SIM
extern "C" void sim_pipe_reader_start(void);
#endif

LOG_MODULE_DECLARE(marshal);

namespace marshal {

static gpio_dt_spec statusLedSpec = GPIO_DT_SPEC_GET(DT_ALIAS(led), gpios);
static gpio_dt_spec buzzerSpec = GPIO_DT_SPEC_GET(DT_ALIAS(buzzer), gpios);
static Led statusLed(&statusLedSpec);
static Buzzer buzzer(&buzzerSpec);

#if !defined(CONFIG_BOARD_NATIVE_SIM)
static Barometer barometer(DEVICE_DT_GET(DT_ALIAS(barometer)));
static Imu imu(DEVICE_DT_GET(DT_ALIAS(imu)));
static VoltageMonitor voltageMonitor(DEVICE_DT_GET(DT_ALIAS(vbat_sensor)), DEVICE_DT_GET(DT_ALIAS(vcc_sensor)));
static gpio_dt_spec drogueEnSpec{DEVICE_DT_GET(DT_NODELABEL(gpiob)), 9, GPIO_ACTIVE_HIGH};
static gpio_dt_spec mainEnSpec{DEVICE_DT_GET(DT_NODELABEL(gpiob)), 7, GPIO_ACTIVE_HIGH};
static gpio_dt_spec drogueFltSpec{DEVICE_DT_GET(DT_NODELABEL(gpioc)), 13, GPIO_ACTIVE_LOW};
static gpio_dt_spec mainFltSpec{DEVICE_DT_GET(DT_NODELABEL(gpioa)), 4, GPIO_ACTIVE_LOW};
static PyroController droguePyro(drogueEnSpec, drogueFltSpec);
static PyroController mainPyro(mainEnSpec, mainFltSpec);
static bool armed = false;
static FlightLogger* flightLogger = nullptr;
static FlightExporter* flightExporter = nullptr;
static FlightStateMachine* flightStateMachine = nullptr;
static float padAltitudeM = 0.0f;
static float currentAltitudeM = 0.0f;
static bool apogeeSeen = false;
static uint32_t apogeeTimeMs = 0U;
static bool drogueDeployIssued = false;
static bool mainDeployIssued = false;

#define BARO_STACK_SIZE 1024
#define BARO_PRIORITY   4
K_THREAD_STACK_DEFINE(baroStack, BARO_STACK_SIZE);
static k_thread baroThread;

#define VOLTAGE_STACK_SIZE 512
#define VOLTAGE_PRIORITY   10
K_THREAD_STACK_DEFINE(voltageStack, VOLTAGE_STACK_SIZE);
static k_thread voltageThread;

static constexpr uint16_t kImuLoopRateHz = 100;
static constexpr uint16_t kBaroLoopRateHz = 100;
static constexpr uint32_t kPyroFireDurationMs = 50U;

static float pressureKPaToAltitudeM(float pressureKPa) {
    return 44330.0f * (1.0f - std::pow(pressureKPa / 101.325f, 1.0f / 5.255f));
}

static uint32_t sensorValueMilli(const sensor_value& value) {
    return static_cast<uint32_t>(value.val1) * 1000U + static_cast<uint32_t>(value.val2 / 1000);
}

static void logPyroAction(uint8_t channel, FlightLog::PyroAction action) {
    if (flightLogger != nullptr && flightLogger->isRecording()) {
        flightLogger->logPyroEvent(channel, action, 0);
    }
}

static void resetFlightActions() {
    apogeeSeen = false;
    apogeeTimeMs = 0U;
    drogueDeployIssued = false;
    mainDeployIssued = false;
}

static void requestPyroDeploy(PyroController& controller, uint8_t channel, const char* reason) {
    ARG_UNUSED(controller);

    LOG_WRN("Pyro channel %u deploy requested (%s, nominal pulse=%u ms), but fire() is intentionally stubbed", channel,
            reason, kPyroFireDurationMs);
    logPyroAction(channel, FlightLog::PyroAction::FIRE);

    // const int ret = controller.fire(kPyroFireDurationMs);
    // if (ret != 0) {
    //     LOG_ERR("Pyro channel %u fire failed: %d", channel, ret);
    // }
}

static void maybeRunDeployments() {
    if (!armed || !apogeeSeen) {
        return;
    }

    const auto mode = FlightComputerSettings::deployMode();
    const uint32_t now = k_uptime_get_32();
    const bool apogeeDelayElapsed = (now - apogeeTimeMs) >= FlightComputerSettings::apogeeDelayMs();

    switch (mode) {
        case FlightComputerSettings::DeployMode::DUAL_DEPLOY:
            if (!drogueDeployIssued && apogeeDelayElapsed) {
                requestPyroDeploy(droguePyro, 1, "apogee drogue");
                drogueDeployIssued = true;
            }

            if (!mainDeployIssued && currentAltitudeM <= FlightComputerSettings::mainDeployAltM()) {
                requestPyroDeploy(mainPyro, 2, "main deploy altitude");
                mainDeployIssued = true;
            }
            break;

        case FlightComputerSettings::DeployMode::DROGUE_ONLY:
            if (!drogueDeployIssued && apogeeDelayElapsed) {
                requestPyroDeploy(droguePyro, 1, "apogee drogue");
                drogueDeployIssued = true;
            }
            break;

        case FlightComputerSettings::DeployMode::MAIN_ONLY:
            if (!mainDeployIssued) {
                requestPyroDeploy(mainPyro, 2, "main-only apogee deploy");
                mainDeployIssued = true;
            }
            break;
    }
}

static void handleStateChange(FlightState oldState, FlightState newState) {
    if (newState == FlightState::BOOST && armed && flightLogger != nullptr && !flightLogger->isRecording()) {
        resetFlightActions();
        const uint32_t id = FlightComputerSettings::incrementFlightCounter();
        const int ret = flightLogger->startFlight(id, kImuLoopRateHz, kBaroLoopRateHz);
        if (ret != 0) {
            LOG_ERR("Failed to start flight logging for flight %u: %d", id, ret);
        } else {
            LOG_INF("Flight %u logging started", id);
        }
    }

    if (flightLogger != nullptr && flightLogger->isRecording()) {
        flightLogger->logStateChange(static_cast<uint8_t>(oldState), static_cast<uint8_t>(newState));
    }

    if (newState == FlightState::APOGEE) {
        apogeeSeen = true;
        apogeeTimeMs = k_uptime_get_32();
        LOG_INF("Apogee detected, deployment timer started");
    }

    if (newState == FlightState::LANDED) {
        const int drogueDisarmRet = droguePyro.disarm();
        if (drogueDisarmRet == 0) {
            logPyroAction(1, FlightLog::PyroAction::DISARM);
        }

        const int mainDisarmRet = mainPyro.disarm();
        if (mainDisarmRet == 0) {
            logPyroAction(2, FlightLog::PyroAction::DISARM);
        }

        if (flightLogger != nullptr && flightLogger->isRecording()) {
            flightLogger->endFlight();
        }
    }
}

static void baroThreadEntry(void*, void*, void*) {
    BaroSample latestBaroSample = barometer.sample();
    uint32_t loopCount = 0U;

    while (true) {
        const ImuSample imuSample = imu.sample();
        if ((loopCount % (kImuLoopRateHz / kBaroLoopRateHz)) == 0U) {
            latestBaroSample = barometer.sample();
        }

        const FlightState state = flightStateMachine->update(imuSample, latestBaroSample);

        if (flightLogger != nullptr && flightLogger->isRecording()) {
            flightLogger->logImu(imuSample);
            if ((loopCount % (kImuLoopRateHz / kBaroLoopRateHz)) == 0U) {
                flightLogger->logBaro(latestBaroSample);
            }
        }

        const uint32_t pressureMilliKpa = sensorValueMilli(latestBaroSample.pressure);
        const float absoluteAltitudeM = pressureKPaToAltitudeM(static_cast<float>(pressureMilliKpa) / 1000.0f);
        const uint32_t altitudeMm = static_cast<uint32_t>(absoluteAltitudeM * 1000.0f);
        const int32_t tempMilliC = latestBaroSample.temperature.val1 * 1000 + latestBaroSample.temperature.val2 / 1000;
        currentAltitudeM = absoluteAltitudeM - padAltitudeM;

        if (state == FlightState::PAD) {
            padAltitudeM = absoluteAltitudeM;
            currentAltitudeM = 0.0f;
        } else if (currentAltitudeM < 0.0f) {
            currentAltitudeM = 0.0f;
        }

        maybeRunDeployments();

        const int32_t aglMilliM = static_cast<int32_t>(currentAltitudeM * 1000.0f);
        // LOG_INF("Flight state=%u pressure=%u.%03u kPa altitude=%u.%03u m agl=%d.%03d m temp=%d.%03d C",
        //         static_cast<unsigned>(state), pressureMilliKpa / 1000, pressureMilliKpa % 1000, altitudeMm / 1000,
        //         altitudeMm % 1000, aglMilliM / 1000, std::abs(aglMilliM % 1000), tempMilliC / 1000,
        //         std::abs(tempMilliC % 1000));

        ++loopCount;
        k_sleep(K_MSEC(1000 / kImuLoopRateHz));
    }
}

static void voltageThreadEntry(void*, void*, void*) {
    while (true) {
        voltageMonitor.sample();
        const uint16_t vbatMv = static_cast<uint16_t>(voltageMonitor.vbatMv());
        const uint16_t vccMv = static_cast<uint16_t>(voltageMonitor.vccMv());

        if (flightLogger != nullptr && flightLogger->isRecording()) {
            flightLogger->logVoltage(vbatMv, vccMv, 0, 0);
        }

        LOG_INF("Voltage: VBAT=%u mV, VCC=%u mV", vbatMv, vccMv);

        k_sleep(K_MSEC(1000));
    }
}

static bool checkBatteryVoltage() {
    voltageMonitor.sample();
    const uint16_t vbat = static_cast<uint16_t>(voltageMonitor.vbatMv());
    const uint16_t threshold = FlightComputerSettings::minBatteryMv();

    if (vbat < threshold) {
        LOG_ERR("Battery voltage %u mV below lockout threshold %u mV - NOT ARMING", vbat, threshold);
        return false;
    }

    LOG_INF("Battery voltage OK: %u mV (threshold %u mV)", vbat, threshold);
    return true;
}

static int ensurePyrosArmed() {
    if (armed) {
        return 0;
    }

    if (!checkBatteryVoltage()) {
        return -EIO;
    }

    int ret = droguePyro.arm();
    if (ret != 0) {
        LOG_ERR("Drogue pyro arm failed: %d", ret);
        logPyroAction(1, FlightLog::PyroAction::FAULT);
        return ret;
    }
    logPyroAction(1, FlightLog::PyroAction::ARM);

    ret = mainPyro.arm();
    if (ret != 0) {
        LOG_ERR("Main pyro arm failed: %d", ret);
        logPyroAction(2, FlightLog::PyroAction::FAULT);
        return ret;
    }
    logPyroAction(2, FlightLog::PyroAction::ARM);

    armed = true;
    return 0;
}

#ifdef CONFIG_SHELL
static int cmdTestLog(const struct shell* sh, size_t argc, char** argv) {
    ARG_UNUSED(argc);

    if (flightLogger == nullptr || flightExporter == nullptr) {
        shell_error(sh, "Logger/exporter not initialized");
        return -ENODEV;
    }

    if (flightLogger->isRecording()) {
        shell_error(sh, "Flight logging already in progress");
        return -EBUSY;
    }

    char* end = nullptr;
    const unsigned long durationSeconds = strtoul(argv[1], &end, 10);
    if (argv[1][0] == '\0' || end == nullptr || *end != '\0' || durationSeconds == 0UL) {
        shell_error(sh, "Usage: test log <seconds>");
        return -EINVAL;
    }

    const uint32_t flightId = FlightComputerSettings::incrementFlightCounter();
    int ret = flightLogger->startFlight(flightId, kImuLoopRateHz, kBaroLoopRateHz);
    if (ret != 0) {
        shell_error(sh, "Failed to start logging: %d", ret);
        return ret;
    }

    shell_print(sh, "Recording flight %u for %lu second(s)...", flightId, durationSeconds);
    k_sleep(K_SECONDS(durationSeconds));
    flightLogger->endFlight();
    k_sleep(K_MSEC(100));

    ret = flightExporter->scanFlights();
    if (ret < 0) {
        shell_error(sh, "Recorded flight %u but scan failed: %d", flightId, ret);
        return ret;
    }

    ret = flightExporter->exportLatest();
    if (ret == 0) {
        shell_print(sh, "Flight %u recorded and exported to the FAT partition.", flightId);
        return 0;
    }

    shell_error(sh, "Flight %u recorded but export failed: %d", flightId, ret);
    return ret;
}

static int cmdTestDeploy(const struct shell* sh, size_t argc, char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (flightLogger == nullptr) {
        shell_error(sh, "Logger not initialized");
        return -ENODEV;
    }

    int ret = ensurePyrosArmed();
    if (ret != 0) {
        shell_error(sh, "Failed to arm pyros: %d", ret);
        return ret;
    }

    shell_print(sh, "Firing drogue pyro");
    ret = droguePyro.fire(kPyroFireDurationMs);
    if (ret != 0) {
        logPyroAction(1, FlightLog::PyroAction::FAULT);
        shell_error(sh, "Drogue pyro fire failed: %d", ret);
        return ret;
    }
    logPyroAction(1, FlightLog::PyroAction::FIRE);

    k_sleep(K_MSEC(100));

    shell_print(sh, "Firing main pyro");
    ret = mainPyro.fire(kPyroFireDurationMs);
    if (ret != 0) {
        logPyroAction(2, FlightLog::PyroAction::FAULT);
        shell_error(sh, "Main pyro fire failed: %d", ret);
        return ret;
    }
    logPyroAction(2, FlightLog::PyroAction::FIRE);

    shell_print(sh, "Pyro test complete");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
    subTest, SHELL_CMD_ARG(log, NULL, "Record live data for N seconds, then export latest flight", cmdTestLog, 2, 0),
    SHELL_CMD(deploy, NULL, "Arm and fire both pyro channels", cmdTestDeploy), SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(test, &subTest, "Marshal test helpers", NULL);
#endif
#endif

int initRuntime() {
    int ret = statusLed.init();
    if (ret != 0) {
        LOG_ERR("Status LED init failed: %d", ret);
    }

    ret = buzzer.init();
    if (ret != 0) {
        LOG_ERR("Buzzer init failed: %d", ret);
    }

#if !defined(CONFIG_BOARD_NATIVE_SIM)
    static const flash_area* rawFa = nullptr;
    static const flash_area* fatFa = nullptr;
    ret = flash_area_open(PARTITION_ID(raw_partition), &rawFa);
    if (ret != 0) {
        LOG_ERR("Failed to open raw partition: %d", ret);
    }

    ret = flash_area_open(PARTITION_ID(fat_partition), &fatFa);
    if (ret != 0) {
        LOG_ERR("Failed to open FAT partition: %d", ret);
    }

    static FlightLogger logger(rawFa);
    static FlightExporter exporter(rawFa, fatFa);
    flightLogger = &logger;
    flightExporter = &exporter;

    if (rawFa != nullptr) {
        ret = flightLogger->init();
        if (ret != 0) {
            LOG_ERR("Flight logger init failed: %d", ret);
        }
    }

    if (rawFa != nullptr && fatFa != nullptr) {
        ret = exporter.init();
        if (ret != 0) {
            LOG_ERR("Flight exporter init failed: %d", ret);
        } else {
            flightExporterShellRegister(&exporter);
        }
    }

    ret = initUsb();
    if (ret != 0) {
        LOG_ERR("USB init failed: %d", ret);
    }

    ret = barometer.init();
    if (ret != 0) {
        LOG_ERR("Barometer init failed: %d", ret);
    }

    ret = imu.init(nullptr);
    if (ret != 0) {
        LOG_ERR("IMU init failed: %d", ret);
    }

    ret = voltageMonitor.init();
    if (ret != 0) {
        LOG_ERR("Voltage monitor init failed: %d", ret);
    }

    armed = checkBatteryVoltage();

    ret = droguePyro.init();
    if (ret != 0) {
        LOG_ERR("Drogue pyro init failed: %d", ret);
    } else if (armed) {
        ret = droguePyro.arm();
        if (ret != 0) {
            LOG_ERR("Drogue pyro arm failed: %d", ret);
            logPyroAction(1, FlightLog::PyroAction::FAULT);
        } else {
            logPyroAction(1, FlightLog::PyroAction::ARM);
        }
    }

    ret = mainPyro.init();
    if (ret != 0) {
        LOG_ERR("Main pyro init failed: %d", ret);
    } else if (armed) {
        ret = mainPyro.arm();
        if (ret != 0) {
            LOG_ERR("Main pyro arm failed: %d", ret);
            logPyroAction(2, FlightLog::PyroAction::FAULT);
        } else {
            logPyroAction(2, FlightLog::PyroAction::ARM);
        }
    }

    static FlightStateMachine fsm(barometer, imu);
    flightStateMachine = &fsm;
    flightStateMachine->onStateChange(handleStateChange);
#else
    LOG_INF("native_sim mode: sensor, logger, and USB paths are disabled");
#endif

    return 0;
}

void startRuntimeThreads() {
#if !defined(CONFIG_BOARD_NATIVE_SIM)
    k_thread_create(&baroThread, baroStack, BARO_STACK_SIZE, baroThreadEntry, nullptr, nullptr, nullptr, BARO_PRIORITY,
                    0, K_NO_WAIT);

    k_thread_create(&voltageThread, voltageStack, VOLTAGE_STACK_SIZE, voltageThreadEntry, nullptr, nullptr, nullptr,
                    VOLTAGE_PRIORITY, 0, K_NO_WAIT);
#else
    sim_pipe_reader_start();
#endif
}

void runStatusLoop() {
    // buzzer.beep(3000);
    while (true) {
        statusLed.toggle();
        k_msleep(1000);
    }
}

} // namespace marshal
