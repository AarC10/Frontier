/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <core/flight/FlightStateMachine.h>
#include <cmath>
#include <core/io/Buzzer.h>
#include <core/io/Led.h>
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
static bool armed = false;
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

// SENSOR READERS
#if !defined(CONFIG_BOARD_NATIVE_SIM)
static void imuDataReadyHandler(const device *dev, const sensor_trigger *trig) {
    ARG_UNUSED(dev);
    ARG_UNUSED(trig);

    const ImuSample sample = imu.sample();
    // logger.logImu(sample);
}
#endif

#if !defined(CONFIG_BOARD_NATIVE_SIM)
#define BARO_STACK_SIZE 1024
#define BARO_PRIORITY   4
K_THREAD_STACK_DEFINE(baroStack, BARO_STACK_SIZE);
static k_thread baroThread;
float pressureKPaToAltitudeM(float pressureKPa) {
    return 44330.0f * (1.0f - std::pow(pressureKPa / 101.325f, 1.0f / 5.255f));
}

static uint32_t sensorValueMilli(const sensor_value &value) {
    return static_cast<uint32_t>(value.val1) * 1000U + static_cast<uint32_t>(value.val2 / 1000);
}

static void baroThreadEntry(void *, void *, void *) {
    while (true) {
        const BaroSample sample = barometer.sample();

        const uint32_t pressureMilliKpa = sensorValueMilli(sample.pressure);
        const uint32_t altitudeMm = static_cast<uint32_t>(pressureKPaToAltitudeM(static_cast<float>(pressureMilliKpa) / 1000.0f) * 1000.0f);
        const int32_t tempMilliC = sample.temperature.val1 * 1000 + sample.temperature.val2 / 1000;

        LOG_INF("Barometer: pressure=%u.%03u kPa altitude=%u.%03u m temp=%d.%03d C", pressureMilliKpa / 1000,
                pressureMilliKpa % 1000, altitudeMm / 1000, altitudeMm % 1000, tempMilliC / 1000,
                std::abs(tempMilliC % 1000));
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
         LOG_INF("Voltage: VBAT=%u mV, VCC=%u mV", static_cast<uint16_t>(voltageMonitor.vbatMv()),
                 static_cast<uint16_t>(voltageMonitor.vccMv()));

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
    static const flash_area *rawFa = nullptr;
    static const flash_area *fatFa = nullptr;
    flash_area_open(PARTITION_ID(raw_partition), &rawFa);
    flash_area_open(PARTITION_ID(fat_partition), &fatFa);
    ARG_UNUSED(rawFa);
    ARG_UNUSED(fatFa);

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

    ret = statusLed.init();
    if (ret != 0) {
        LOG_ERR("Status LED init failed: %d", ret);
    }

    ret = buzzer.init();
    if (ret != 0) {
        LOG_ERR("Buzzer init failed: %d", ret);
    }

    // Init FSM
    static FlightStateMachine fsm(barometer, imu);
    ARG_UNUSED(fsm);

    // TODO: feed FlightComputerSettings::armingAltM() and mainDeployAltM() into the FSM

    // fsm.onStateChange([](FlightState oldState, FlightState newState) {
    //     logger.logStateChange(static_cast<uint8_t>(oldState), static_cast<uint8_t>(newState));
    //
    //     if (newState == FlightState::BOOST && armed) {
    //         const uint32_t id = FlightComputerSettings::incrementFlightCounter();
    //         logger.startFlight(id, 100, 25);
    //         LOG_INF("Flight %u logging started", id);
    //     }
    //
    //     if (newState == FlightState::LANDED) {
    //         logger.endFlight();
    //     }
    //
    //     // TODO: PyroController logic
    //     // DUAL_DEPLOY:
    //     //   APOGEE: delay(apogeeDelayMs) → fire CH1 (drogue)
    //     //   DESCENT: altitude < mainDeployAltFt → fire CH2 (main)
    //     //
    //     // DROGUE_ONLY:
    //     //   APOGEE: delay(apogeeDelayMs) → fire CH1 (drogue)
    //     //
    //     // MAIN_ONLY:
    //     //   APOGEE: fire CH2 (main)
    // });

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
