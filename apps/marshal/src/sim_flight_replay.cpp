/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <marshal/sim_flight_replay.h>

#include <cerrno>

#ifdef CONFIG_BOARD_NATIVE_SIM

#include <core/flight/FlightStateMachine.h>
#include <core/flight_logger/FlightExporter.h>
#include <core/flight_logger/FlightLogger.h>
#include <core/flight_logger/FlightLogRecords.h>
#include <core/sim/sim_sensor_state.h>
#include <core/sensors/Barometer.h>
#include <core/sensors/Imu.h>
#include <soc.h>
#include <cmdline.h>
#include <posix_board_if.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

LOG_MODULE_REGISTER(sim_flight_replay, LOG_LEVEL_INF);

namespace marshal::sim {

namespace {

constexpr uint32_t kFlightLogPageSize = 256U;
constexpr size_t kReplayThreadStackSize = 4096U;
constexpr int kReplayThreadPriority = 5;

struct ReplayEvent {
    enum class Kind : uint8_t {
        IMU,
        BARO,
    };

    Kind kind;
    uint32_t flightTimeMs;
    ImuSample imu{};
    BaroSample baro{};
};

struct ReplaySample {
    uint32_t flightTimeMs;
    ImuSample imu;
    BaroSample baro;
};

struct ReplayFlight {
    uint32_t flightId{0U};
    uint16_t imuRateHz{0U};
    uint16_t baroRateHz{0U};
    std::vector<ReplaySample> samples;
};

K_THREAD_STACK_DEFINE(replayThreadStack, kReplayThreadStackSize);
static k_thread replayThread;
static bool replayThreadStarted;
static const char* replayBinPath;
static uint32_t replayCutoffMs;
static ReplayFlight replayFlight;

static uint32_t decodeFlightTimeMs(uint16_t timestampRaw, uint16_t previousRaw, bool havePrevious, uint32_t& wrapCount) {
    if (havePrevious && timestampRaw < previousRaw) {
        ++wrapCount;
    }

    return static_cast<uint32_t>(timestampRaw) + (wrapCount * 0x10000U);
}

static bool readFileBytes(const char* path, std::vector<uint8_t>& bytes) {
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open()) {
        LOG_ERR("Failed to open replay file %s", path);
        return false;
    }

    input.seekg(0, std::ios::end);
    const std::streamsize size = input.tellg();
    if (size <= 0) {
        LOG_ERR("Replay file %s is empty", path);
        return false;
    }

    input.seekg(0, std::ios::beg);
    bytes.resize(static_cast<size_t>(size));
    input.read(reinterpret_cast<char*>(bytes.data()), size);
    if (!input.good() && !input.eof()) {
        LOG_ERR("Failed to read replay file %s", path);
        return false;
    }

    return true;
}

static bool toSensorValue(double value, sensor_value& out) {
    return sensor_value_from_double(&out, value) == 0;
}

static bool decodeImuRecord(const FlightLog::ImuRecord& record, ImuSample& out) {
    sensor_ug_to_ms2(static_cast<int32_t>(record.accelXMg) * 1000, &out.accelX);
    sensor_ug_to_ms2(static_cast<int32_t>(record.accelYMg) * 1000, &out.accelY);
    sensor_ug_to_ms2(static_cast<int32_t>(record.accelZMg) * 1000, &out.accelZ);

    constexpr double degToRad = 3.14159265358979323846 / 180.0;
    if (!toSensorValue((static_cast<double>(record.gyroXDdps) / 10.0) * degToRad, out.gyroX) ||
        !toSensorValue((static_cast<double>(record.gyroYDdps) / 10.0) * degToRad, out.gyroY) ||
        !toSensorValue((static_cast<double>(record.gyroZDdps) / 10.0) * degToRad, out.gyroZ)) {
        return false;
    }

    return true;
}

static bool decodeBaroRecord(const FlightLog::BaroRecord& record, BaroSample& out) {
    return toSensorValue(static_cast<double>(record.pressurePa) / 1000.0, out.pressure) &&
           toSensorValue(static_cast<double>(record.tempCDeg) / 100.0, out.temperature);
}

static const char* flightStateName(FlightState state) {
    switch (state) {
        case FlightState::PAD:
            return "PAD";
        case FlightState::BOOST:
            return "BOOST";
        case FlightState::COAST:
            return "COAST";
        case FlightState::APOGEE:
            return "APOGEE";
        case FlightState::DESCENT:
            return "DESCENT";
        case FlightState::LANDED:
            return "LANDED";
    }

    return "UNKNOWN";
}

static void publishReplaySample(const ReplaySample& sample) {
    k_mutex_lock(&g_sim_sensor_mutex, K_FOREVER);
    g_sim_sensor_state.accel_x = sample.imu.accelX;
    g_sim_sensor_state.accel_y = sample.imu.accelY;
    g_sim_sensor_state.accel_z = sample.imu.accelZ;
    g_sim_sensor_state.gyro_x = sample.imu.gyroX;
    g_sim_sensor_state.gyro_y = sample.imu.gyroY;
    g_sim_sensor_state.gyro_z = sample.imu.gyroZ;
    g_sim_sensor_state.pressure = sample.baro.pressure;
    g_sim_sensor_state.temperature = sample.baro.temperature;
    k_mutex_unlock(&g_sim_sensor_mutex);
}

static bool buildReplaySamples(const std::vector<ReplayEvent>& events, ReplayFlight& flight) {
    bool haveLatestBaro = false;
    BaroSample latestBaro{};

    for (size_t index = 0; index < events.size();) {
        const uint32_t groupTimeMs = events[index].flightTimeMs;
        bool haveGroupImu = false;
        bool haveGroupBaro = false;
        ImuSample groupImu{};
        BaroSample groupBaro{};

        while (index < events.size() && events[index].flightTimeMs == groupTimeMs) {
            if (events[index].kind == ReplayEvent::Kind::IMU) {
                groupImu = events[index].imu;
                haveGroupImu = true;
            } else {
                groupBaro = events[index].baro;
                haveGroupBaro = true;
            }
            ++index;
        }

        if (haveGroupBaro) {
            latestBaro = groupBaro;
            haveLatestBaro = true;
        }

        if (haveGroupImu && haveLatestBaro) {
            flight.samples.push_back(ReplaySample{
                .flightTimeMs = groupTimeMs,
                .imu = groupImu,
                .baro = haveGroupBaro ? groupBaro : latestBaro,
            });
        }
    }

    return !flight.samples.empty();
}

static bool loadReplayFlight(const char* path, ReplayFlight& flight) {
    std::vector<uint8_t> bytes;
    if (!readFileBytes(path, bytes)) {
        return false;
    }

    std::vector<ReplayEvent> events;
    size_t offset = 0U;
    bool foundFlightHeader = false;
    uint16_t previousSensorTimestampRaw = 0U;
    bool havePreviousSensorTimestamp = false;
    uint32_t sensorTimestampWrapCount = 0U;

    while (offset < bytes.size()) {
        const size_t pageOffset = offset % kFlightLogPageSize;

        if (bytes[offset] == 0xFFU) {
            offset += (pageOffset == 0U) ? kFlightLogPageSize : (kFlightLogPageSize - pageOffset);
            continue;
        }

        if (bytes[offset] != FlightLog::RECORD_MAGIC) {
            LOG_ERR("Replay file %s is not a valid flight log at offset 0x%zx", path, offset);
            return false;
        }

        if ((offset + sizeof(FlightLog::RecordHeader)) > bytes.size()) {
            LOG_ERR("Replay file %s has a truncated record header", path);
            return false;
        }

        const uint8_t rawType = bytes[offset + 1U];
        const size_t recordSize = FlightLog::recordSizeFromRaw(rawType);
        if (recordSize == 0U) {
            LOG_ERR("Replay file %s has unknown record type 0x%02x", path, rawType);
            return false;
        }

        if ((pageOffset + recordSize) > kFlightLogPageSize || (offset + recordSize) > bytes.size()) {
            LOG_ERR("Replay file %s has a record crossing a page boundary", path);
            return false;
        }

        if (rawType == std::to_underlying(FlightLog::RecordType::FLIGHT_HEADER)) {
            FlightLog::FlightHeaderRecord record{};
            std::memcpy(&record, bytes.data() + offset, sizeof(record));

            if (!foundFlightHeader) {
                if (record.formatVersion != FlightLog::FORMAT_VERSION) {
                    LOG_ERR("Replay file %s format version %u is unsupported", path, record.formatVersion);
                    return false;
                }

                flight.flightId = record.flightId;
                flight.imuRateHz = record.imuRateHz;
                flight.baroRateHz = record.baroRateHz;
                foundFlightHeader = true;
                previousSensorTimestampRaw = 0U;
                havePreviousSensorTimestamp = false;
                sensorTimestampWrapCount = 0U;
            } else {
                break;
            }

            offset += recordSize;
            continue;
        }

        if (!foundFlightHeader) {
            LOG_ERR("Replay file %s does not start with a flight header", path);
            return false;
        }

        switch (static_cast<FlightLog::RecordType>(rawType)) {
            case FlightLog::RecordType::IMU: {
                FlightLog::ImuRecord record{};
                std::memcpy(&record, bytes.data() + offset, sizeof(record));
                ImuSample sample{};
                if (!decodeImuRecord(record, sample)) {
                    LOG_ERR("Failed to decode IMU record from %s", path);
                    return false;
                }

                const uint32_t flightTimeMs = decodeFlightTimeMs(record.hdr.timestampMillis, previousSensorTimestampRaw,
                                                                 havePreviousSensorTimestamp, sensorTimestampWrapCount);
                previousSensorTimestampRaw = record.hdr.timestampMillis;
                havePreviousSensorTimestamp = true;

                events.push_back(ReplayEvent{
                    .kind = ReplayEvent::Kind::IMU,
                    .flightTimeMs = flightTimeMs,
                    .imu = sample,
                });
                break;
            }

            case FlightLog::RecordType::BARO: {
                FlightLog::BaroRecord record{};
                std::memcpy(&record, bytes.data() + offset, sizeof(record));
                BaroSample sample{};
                if (!decodeBaroRecord(record, sample)) {
                    LOG_ERR("Failed to decode BARO record from %s", path);
                    return false;
                }

                const uint32_t flightTimeMs = decodeFlightTimeMs(record.hdr.timestampMillis, previousSensorTimestampRaw,
                                                                 havePreviousSensorTimestamp, sensorTimestampWrapCount);
                previousSensorTimestampRaw = record.hdr.timestampMillis;
                havePreviousSensorTimestamp = true;

                events.push_back(ReplayEvent{
                    .kind = ReplayEvent::Kind::BARO,
                    .flightTimeMs = flightTimeMs,
                    .baro = sample,
                });
                break;
            }

            case FlightLog::RecordType::FLIGHT_END:
                offset += recordSize;
                return buildReplaySamples(events, flight);

            case FlightLog::RecordType::STATE_CHANGE:
            case FlightLog::RecordType::PYRO_EVENT:
            case FlightLog::RecordType::VOLTAGE:
            case FlightLog::RecordType::TIMESTAMP_SYNC:
                break;

            case FlightLog::RecordType::FLIGHT_HEADER:
                break;
        }

        offset += recordSize;
    }

    if (!foundFlightHeader) {
        LOG_ERR("Replay file %s did not contain a flight header", path);
        return false;
    }

    if (!buildReplaySamples(events, flight)) {
        LOG_ERR("Replay file %s did not contain paired IMU/BARO replay samples", path);
        return false;
    }

    return true;
}

static std::string buildReplayExportPath(const char* replayPath) {
    std::string outputPath = replayPath;
    const size_t slashPos = outputPath.find_last_of("/\\");
    const size_t extPos = outputPath.find_last_of('.');
    if (extPos != std::string::npos && (slashPos == std::string::npos || extPos > slashPos)) {
        outputPath.erase(extPos);
    }

    outputPath += "_REPLAY.BIN";
    return outputPath;
}

static int exportReplayFlightToHost(const flash_area* rawFa, uint32_t flightId, const char* replayPath) {
    if (rawFa == nullptr || replayPath == nullptr) {
        return -EINVAL;
    }

    FlightExporter exporter(rawFa, nullptr);
    const int scanRet = exporter.scanFlights();
    if (scanRet < 0) {
        LOG_ERR("Failed to scan replay flight for export: %d", scanRet);
        return scanRet;
    }

    const FlightExporter::FlightInfo* flightInfo = exporter.findFlight(flightId);
    if (flightInfo == nullptr) {
        LOG_ERR("Replay flight %u not found for export", flightId);
        return -ENOENT;
    }

    const std::string outputPath = buildReplayExportPath(replayPath);
    std::ofstream output(outputPath, std::ios::binary | std::ios::trunc);
    if (!output.is_open()) {
        LOG_ERR("Failed to open replay export path %s", outputPath.c_str());
        return -EIO;
    }

    constexpr size_t kExportChunkSize = 512U;
    uint8_t buffer[kExportChunkSize];
    uint32_t remaining = flightInfo->endOffset - flightInfo->startOffset;
    uint32_t offset = flightInfo->startOffset;

    while (remaining > 0U) {
        const size_t chunkSize = std::min<size_t>(remaining, sizeof(buffer));
        const int readRet = flash_area_read(rawFa, offset, buffer, chunkSize);
        if (readRet != 0) {
            LOG_ERR("Failed reading replay export chunk at 0x%08X: %d", offset, readRet);
            return readRet;
        }

        output.write(reinterpret_cast<const char*>(buffer), static_cast<std::streamsize>(chunkSize));
        if (!output.good()) {
            LOG_ERR("Failed writing replay export to %s", outputPath.c_str());
            return -EIO;
        }

        offset += static_cast<uint32_t>(chunkSize);
        remaining -= static_cast<uint32_t>(chunkSize);
    }

    output.close();
    if (!output) {
        LOG_ERR("Failed to finalize replay export %s", outputPath.c_str());
        return -EIO;
    }

    LOG_INF("Replay flight exported to %s", outputPath.c_str());
    return 0;
}

static void replayThreadEntry(void*, void*, void*) {
    Barometer dummyBarometer(nullptr);
    Imu dummyImu(nullptr);
    FlightStateMachine stateMachine(dummyBarometer, dummyImu);
    const flash_area* rawFa = nullptr;
    uint32_t replayTimeMs = 0U;
    bool landedDetected = false;

    int ret = flash_area_open(PARTITION_ID(raw_partition), &rawFa);
    if (ret != 0) {
        LOG_ERR("Failed to open raw partition for replay logging: %d", ret);
        LOG_PANIC();
        posix_exit(1);
    }

    FlightLogger flightLogger(rawFa);
    ret = flightLogger.init();
    if (ret != 0) {
        LOG_ERR("Failed to init replay flight logger: %d", ret);
        flash_area_close(rawFa);
        LOG_PANIC();
        posix_exit(1);
    }

    ret = flightLogger.eraseAll();
    if (ret != 0) {
        LOG_ERR("Failed to erase replay log partition: %d", ret);
        flash_area_close(rawFa);
        LOG_PANIC();
        posix_exit(1);
    }

    ret = flightLogger.startFlight(replayFlight.flightId, replayFlight.imuRateHz, replayFlight.baroRateHz);
    if (ret != 0) {
        LOG_ERR("Failed to start replay flight log: %d", ret);
        flash_area_close(rawFa);
        LOG_PANIC();
        posix_exit(1);
    }

    stateMachine.onStateChange([&flightLogger, &replayTimeMs, &landedDetected](FlightState oldState, FlightState newState) {
        LOG_INF("Replay detected %s -> %s at %u ms", flightStateName(oldState), flightStateName(newState),
                replayTimeMs);
        flightLogger.logStateChange(std::to_underlying(oldState), std::to_underlying(newState));
        if (newState == FlightState::LANDED) {
            landedDetected = true;
        }
    });

    LOG_INF("Starting flight replay %u from %s (%zu samples, imu=%u Hz, baro=%u Hz)", replayFlight.flightId,
            replayBinPath, replayFlight.samples.size(), replayFlight.imuRateHz, replayFlight.baroRateHz);

    uint32_t previousFlightTimeMs = 0U;
    bool firstSample = true;
    for (const ReplaySample& sample : replayFlight.samples) {
        if (sample.flightTimeMs > replayCutoffMs) {
            LOG_INF("Replay cutoff reached at %u ms", replayCutoffMs);
            break;
        }

        if (!firstSample) {
            const uint32_t deltaMs = sample.flightTimeMs - previousFlightTimeMs;
            if (deltaMs > 0U) {
                k_msleep(static_cast<int32_t>(deltaMs));
            }
        }

        replayTimeMs = sample.flightTimeMs;
        publishReplaySample(sample);
        flightLogger.logImu(sample.imu);
        flightLogger.logBaro(sample.baro);
        (void) stateMachine.update(sample.imu, sample.baro);
        if (landedDetected) {
            break;
        }

        previousFlightTimeMs = sample.flightTimeMs;
        firstSample = false;
    }

    flightLogger.endFlight();
    ret = exportReplayFlightToHost(rawFa, replayFlight.flightId, replayBinPath);
    if (ret != 0) {
        LOG_ERR("Replay export failed: %d", ret);
    }
    flash_area_close(rawFa);

    LOG_INF("Replay finished at %u ms with final state %s", replayTimeMs, flightStateName(stateMachine.currentState()));
    LOG_PANIC();
    posix_exit(0);
}

static void registerReplayOption(void) {
    static args_struct_t replayOptions[] = {
        {
            .option = const_cast<char*>("flight-bin"),
            .name = const_cast<char*>("path.bin"),
            .type = 's',
            .dest = &replayBinPath,
            .descript = const_cast<char*>(
                "Replay the first flight found in a Frontier flight .bin log and ignore recorded state changes"),
        },
        {
            .option = const_cast<char*>("flight-cutoff-ms"),
            .name = const_cast<char*>("ms"),
            .type = 'u',
            .dest = &replayCutoffMs,
            .descript = const_cast<char*>(
                "Stop the replay after this many milliseconds if LANDED has not been detected"),
        },
        ARG_TABLE_ENDMARKER,
    };

    native_add_command_line_opts(replayOptions);
}

NATIVE_TASK(registerReplayOption, PRE_BOOT_1, 10);

} // namespace

bool flightReplayRequested() { return replayBinPath != nullptr && replayBinPath[0] != '\0'; }

int startFlightReplay() {
    if (!flightReplayRequested()) {
        return -ENOENT;
    }

    if (replayThreadStarted) {
        return 0;
    }

    replayFlight = ReplayFlight{};
    if (!loadReplayFlight(replayBinPath, replayFlight)) {
        return -EINVAL;
    }

    if (replayCutoffMs == UINT32_MAX) {
        replayCutoffMs = replayFlight.samples.back().flightTimeMs;
    }

    replayThreadStarted = true;
    k_tid_t tid = k_thread_create(&replayThread, replayThreadStack, K_THREAD_STACK_SIZEOF(replayThreadStack),
                                  replayThreadEntry, nullptr, nullptr, nullptr, kReplayThreadPriority, 0, K_NO_WAIT);
    k_thread_name_set(tid, "flight_replay");
    return 0;
}

} // namespace marshal::sim

#else

namespace marshal::sim {

bool flightReplayRequested() { return false; }

int startFlightReplay() { return -ENOTSUP; }

} // namespace marshal::sim

#endif
