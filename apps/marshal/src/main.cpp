/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <core/flight/FlightStateMachine.h>
#include <core/flight_logger/FlightExporter.h>
#include <core/flight_logger/FlightLogger.h>
#include <cmath>
#include <core/io/Buzzer.h>
#include <core/io/Led.h>
#include <core/pyro/PyroController.h>
#include <core/sensors/Barometer.h>
#include <core/sensors/Imu.h>
#include <core/sensors/VoltageMonitor.h>
#include <core/settings/FlightComputerSettings.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/storage/flash_map.h>
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
#include <zephyr/usb/usbd.h>
#endif
#if defined(CONFIG_USBD_MSC_CLASS)
#include <zephyr/usb/class/usbd_msc.h>
#endif

LOG_MODULE_REGISTER(marshal, LOG_LEVEL_INF);

// GLOBALS
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
static FlightStateMachine* flightStateMachine = nullptr;
static float padAltitudeM = 0.0f;
static float currentAltitudeM = 0.0f;
static bool apogeeSeen = false;
static uint32_t apogeeTimeMs = 0U;
static bool drogueDeployIssued = false;
static bool mainDeployIssued = false;
#endif

static gpio_dt_spec statusLedSpec = GPIO_DT_SPEC_GET(DT_ALIAS(led), gpios);
static gpio_dt_spec buzzerSpec = GPIO_DT_SPEC_GET(DT_ALIAS(buzzer), gpios);
static Led statusLed(&statusLedSpec);
static Buzzer buzzer(&buzzerSpec);

// USB
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
USBD_DEVICE_DEFINE(marshal_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), 0x0483, 0x5740);
USBD_DESC_LANG_DEFINE(marshal_lang);
USBD_DESC_MANUFACTURER_DEFINE(marshal_mfr, "Wild West Rocketry");
USBD_DESC_PRODUCT_DEFINE(marshal_product, "Marshal Flight Computer");
USBD_DESC_SERIAL_NUMBER_DEFINE(marshal_sn);
USBD_DESC_CONFIG_DEFINE(marshal_fs_cfg, "FS Config");
USBD_CONFIGURATION_DEFINE(marshal_fs_config, USB_SCD_SELF_POWERED, 125, &marshal_fs_cfg);
#endif
#if defined(CONFIG_USBD_MSC_CLASS)
USBD_DEFINE_MSC_LUN(marshal, "marshal", "WWR", "Marshal", "1.00");
#endif

#if !defined(CONFIG_BOARD_NATIVE_SIM)
#define BARO_STACK_SIZE 1024
#define BARO_PRIORITY   4
K_THREAD_STACK_DEFINE(baroStack, BARO_STACK_SIZE);
static k_thread baroThread;

static constexpr uint16_t kImuLoopRateHz = 100;
static constexpr uint16_t kBaroLoopRateHz = 100;
static constexpr uint32_t kPyroFireDurationMs = 50U;

float pressureKPaToAltitudeM(float pressureKPa) {
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
        LOG_INF("Flight state=%u pressure=%u.%03u kPa altitude=%u.%03u m agl=%d.%03d m temp=%d.%03d C",
                static_cast<unsigned>(state), pressureMilliKpa / 1000, pressureMilliKpa % 1000, altitudeMm / 1000,
                altitudeMm % 1000, aglMilliM / 1000, std::abs(aglMilliM % 1000), tempMilliC / 1000,
                std::abs(tempMilliC % 1000));

        ++loopCount;
        k_sleep(K_MSEC(1000 / kImuLoopRateHz));
    }
}

// VOLTAGE MONITOR THREAD
#define VOLTAGE_STACK_SIZE 512
#define VOLTAGE_PRIORITY   10
K_THREAD_STACK_DEFINE(voltageStack, VOLTAGE_STACK_SIZE);
static k_thread voltageThread;

static void voltageThreadEntry(void*, void*, void*) {
    while (true) {
        voltageMonitor.sample();
        const uint16_t vbatMv = static_cast<uint16_t>(voltageMonitor.vbatMv());
        const uint16_t vccMv = static_cast<uint16_t>(voltageMonitor.vccMv());

        if (flightLogger != nullptr && flightLogger->isRecording()) {
            flightLogger->logVoltage(vbatMv, vccMv, 0, 0);
        }

        LOG_INF("Voltage: VBAT=%u mV, VCC=%u mV", vbatMv, vccMv);

        k_sleep(K_MSEC(1000)); // 1 Hz
    }
}

static bool checkBatteryVoltage() {
    voltageMonitor.sample();
    const uint16_t vbat = static_cast<uint16_t>(voltageMonitor.vbatMv());
    const uint16_t threshold = FlightComputerSettings::minBatteryMv();

    if (vbat < threshold) {
        LOG_ERR("Battery voltage %u mV below lockout threshold %u mV - NOT ARMING", vbat, threshold);
        // TODO: signal error here
        return false;
    }

    LOG_INF("Battery voltage OK: %u mV (threshold %u mV)", vbat, threshold);
    return true;
}
#endif

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
static void setUsbCodeTriple(enum usbd_speed speed) {
    if (IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS) || IS_ENABLED(CONFIG_USBD_MSC_CLASS)) {
        usbd_device_set_code_triple(&marshal_usbd, speed, USB_BCC_MISCELLANEOUS, 0x02, 0x01);
        return;
    }

    usbd_device_set_code_triple(&marshal_usbd, speed, 0, 0, 0);
}

static int init_usb() {
    int ret = usbd_add_descriptor(&marshal_usbd, &marshal_lang);
    if (ret != 0) {
        return ret;
    }

    ret = usbd_add_descriptor(&marshal_usbd, &marshal_mfr);
    if (ret != 0) {
        return ret;
    }

    ret = usbd_add_descriptor(&marshal_usbd, &marshal_product);
    if (ret != 0) {
        return ret;
    }

    ret = usbd_add_descriptor(&marshal_usbd, &marshal_sn);
    if (ret != 0) {
        return ret;
    }

    ret = usbd_add_configuration(&marshal_usbd, USBD_SPEED_FS, &marshal_fs_config);
    if (ret != 0) {
        return ret;
    }

    ret = usbd_register_all_classes(&marshal_usbd, USBD_SPEED_FS, 1, NULL);
    if (ret != 0) {
        return ret;
    }

    setUsbCodeTriple(USBD_SPEED_FS);

    ret = usbd_init(&marshal_usbd);
    if (ret != 0) {
        return ret;
    }

    return usbd_enable(&marshal_usbd);
}
#endif

int main() {
    int ret = FlightComputerSettings::load();
    if (ret != 0) {
        LOG_ERR("Failed to load settings: %d", ret);
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

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
    ret = init_usb();
    if (ret != 0) {
        LOG_ERR("USB init failed: %d", ret);
    }
#endif

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

    // Battery lockout and refuse to arm if voltage is too low
    armed = checkBatteryVoltage();

    ret = statusLed.init();
    if (ret != 0) {
        LOG_ERR("Status LED init failed: %d", ret);
    }

    ret = buzzer.init();
    if (ret != 0) {
        LOG_ERR("Buzzer init failed: %d", ret);
    }

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

    // Init FSM
    static FlightStateMachine fsm(barometer, imu);
    flightStateMachine = &fsm;
    flightStateMachine->onStateChange(handleStateChange);

    // Start background threads
    k_thread_create(&baroThread, baroStack, BARO_STACK_SIZE, baroThreadEntry, nullptr, nullptr, nullptr, BARO_PRIORITY,
                    0, K_NO_WAIT);

    k_thread_create(&voltageThread, voltageStack, VOLTAGE_STACK_SIZE, voltageThreadEntry, nullptr, nullptr, nullptr,
                    VOLTAGE_PRIORITY, 0, K_NO_WAIT);
#else
    LOG_INF("native_sim mode: sensor, logger, and USB paths are disabled");
#endif

    while (true) {
        statusLed.toggle();
        k_msleep(1000);
    }

    // LOG_INF("READY: armed=%s mode=%u main_alt=%u ft log=%u/%u bytes", armed ? "YES" : "NO",
    //         static_cast<unsigned>(FlightComputerSettings::deployMode()), FlightComputerSettings::mainDeployAltFt(),
    //         logger.writeOffset(), logger.partitionSize());

    return 0;
}
