/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "core/flight_logger/FlightLogger.h"

#include "core/flight_logger/FlightLogRecords.h"

#include <cstring>
#include <utility>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(FlightLogger, CONFIG_LOG_DEFAULT_LEVEL);

static inline float svToFloat(const sensor_value &sv) {
    return static_cast<float>(sv.val1) + static_cast<float>(sv.val2) / 1000000.0f;
}

static inline int16_t accelToMilliG(const sensor_value &sv) {
    const float mg = svToFloat(sv) * (1000.0f / 9.80665f);
    if (mg > 32767.0f) return INT16_MAX;
    if (mg < -32767.0f) return INT16_MIN;
    return static_cast<int16_t>(mg);
}

static inline int16_t gyroToDeciDps(const sensor_value &sv) {
    const float ddps = svToFloat(sv) * (10.0f * 180.0f / 3.14159265f);
    if (ddps > 32767.0f) return INT16_MAX;
    if (ddps < -32767.0f) return INT16_MIN;
    return static_cast<int16_t>(ddps);
}

static inline uint32_t pressureToPa(const sensor_value &sv) {
    const float pa = svToFloat(sv) * 1000.0f;
    if (pa < 0.0f) return 0;
    return static_cast<uint32_t>(pa);
}

static inline int16_t tempToCentiDeg(const sensor_value &sv) {
    const float cdeg = svToFloat(sv) * 100.0f;
    if (cdeg > 32767.0f) return INT16_MAX;
    if (cdeg < -32767.0f) return INT16_MIN;
    return static_cast<int16_t>(cdeg);
}

FlightLogger::FlightLogger(const flash_area *partition) : flashArea(partition), writerStack{}, writerThread() {
    k_msgq_init(&msgq, msgqBuffer, FlightLog::MAX_RECORD_SIZE, MSGQ_DEPTH);
    memset(pageBuffer, 0xFF, PAGE_SIZE);
}

int FlightLogger::init() {
    if (flashArea == nullptr) {
        LOG_ERR("Null partition pointer");
        return -EINVAL;
    }

    partSize = flashArea->fa_size;
    LOG_INF("Raw log partition: %u bytes (%u KB)", partSize, partSize / 1024);

    int ret = scanForWritePointer();
    if (ret != 0) {
        LOG_ERR("Write pointer scan failed: %d", ret);
        flashArea = nullptr;
        return ret;
    }

    LOG_INF("Write pointer recovered at offset 0x%08X (%u bytes used, %u free)", writePtr, writePtr,
            partSize - writePtr);

    return 0;
}

int FlightLogger::startFlight(uint32_t flightId, uint16_t imuRateHz, uint16_t baroRateHz) {
    if (flashArea == nullptr) {
        LOG_ERR("Logger not initialized");
        return -EINVAL;
    }

    if (writePtr >= partSize) {
        LOG_ERR("Raw partition full");
        return -ENOSPC;
    }

    flightEpochMs = k_uptime_get_32();
    flightRecordCount = 0;
    lastSyncMs = flightEpochMs;
    dropped.store(0, std::memory_order_relaxed);

    // Purge stale data from a previous flight
    k_msgq_purge(&msgq);
    pageOffset = 0;
    memset(pageBuffer, 0xFF, PAGE_SIZE);

    // Write FlightHeader directly into page buffer
    FlightLog::FlightHeaderRecord hdr{};
    hdr.hdr.magic = FlightLog::RECORD_MAGIC;
    hdr.hdr.type = std::to_underlying(FlightLog::RecordType::FLIGHT_HEADER);
    hdr.hdr.timestampMillis = 0;
    hdr.flightId = flightId;
    hdr.epochUptimeMillis = flightEpochMs;
    hdr.imuRateHz = imuRateHz;
    hdr.baroRateHz = baroRateHz;
    hdr.formatVersion = FlightLog::FORMAT_VERSION;
    memset(hdr.reserved, 0xFF, sizeof(hdr.reserved));

    appendToPage(&hdr, sizeof(hdr));
    flightRecordCount++;

    int ret = flushPage();
    if (ret != 0) {
        LOG_ERR("Failed to flush flight header: %d", ret);
        return ret;
    }

    recording.store(true, std::memory_order_release);

    // Start writer thread if not already running
    if (writerTid == nullptr) {
        writerTid = k_thread_create(&writerThread, writerStack, WRITER_STACK_SIZE, writerEntry, this, nullptr, nullptr,
                                    WRITER_PRIORITY, 0, K_NO_WAIT);
        k_thread_name_set(writerTid, "log_writer");
    }

    LOG_INF("Flight %u started (epoch=%u ms, writePtr=0x%08X)", flightId, flightEpochMs, writePtr);

    return 0;
}

void FlightLogger::endFlight() {
    if (!recording.load(std::memory_order_acquire)) {
        return;
    }

    // Submit FlightEnd through the normal queue so it serializes
    // with any in-flight records still in the queue
    FlightLog::FlightEndRecord rec{};
    rec.hdr.magic = FlightLog::RECORD_MAGIC;
    rec.hdr.type = std::to_underlying(FlightLog::RecordType::FLIGHT_END);
    rec.hdr.timestampMillis = flightTimestampMs();
    rec.totalRecords = flightRecordCount + 1; // +1 for this record

    submitRecord(&rec, sizeof(rec));

    recording.store(false, std::memory_order_release);

    // Give the writer thread time to drain and flush
    k_sleep(K_MSEC(50));

    LOG_INF("Flight ended (%u records, writePtr=0x%08X, %u dropped)", flightRecordCount, writePtr,
            dropped.load(std::memory_order_relaxed));
}

void FlightLogger::logImu(const ImuSample &sample) {
    if (!recording.load(std::memory_order_relaxed)) return;

    FlightLog::ImuRecord rec{};
    rec.hdr.magic = FlightLog::RECORD_MAGIC;
    rec.hdr.type = std::to_underlying(FlightLog::RecordType::IMU);
    rec.hdr.timestampMillis = flightTimestampMs();
    rec.accelXMg = accelToMilliG(sample.accelX);
    rec.accelYMg = accelToMilliG(sample.accelY);
    rec.accelZMg = accelToMilliG(sample.accelZ);
    rec.gyroXDdps = gyroToDeciDps(sample.gyroX);
    rec.gyroYDdps = gyroToDeciDps(sample.gyroY);
    rec.gyroZDdps = gyroToDeciDps(sample.gyroZ);

    submitRecord(&rec, sizeof(rec));
}

void FlightLogger::logBaro(const BaroSample &sample) {
    if (!recording.load(std::memory_order_relaxed)) return;

    FlightLog::BaroRecord rec{};
    rec.hdr.magic = FlightLog::RECORD_MAGIC;
    rec.hdr.type = std::to_underlying(FlightLog::RecordType::BARO);
    rec.hdr.timestampMillis = flightTimestampMs();
    rec.pressurePa = pressureToPa(sample.pressure);
    rec.tempCDeg = tempToCentiDeg(sample.temperature);
    rec.reserved = 0xFFFF;

    submitRecord(&rec, sizeof(rec));
}

void FlightLogger::logStateChange(uint8_t prevState, uint8_t newState) {
    FlightLog::StateChangeRecord rec{};
    rec.hdr.magic = FlightLog::RECORD_MAGIC;
    rec.hdr.type = std::to_underlying(FlightLog::RecordType::STATE_CHANGE);
    rec.hdr.timestampMillis = flightTimestampMs();
    rec.prevState = prevState;
    rec.newState = newState;
    rec.reserved = 0xFFFF;

    submitRecord(&rec, sizeof(rec));
}

void FlightLogger::logPyroEvent(uint8_t channel, FlightLog::PyroAction action, uint16_t ilmRaw) {
    FlightLog::PyroEventRecord rec{};
    rec.hdr.magic = FlightLog::RECORD_MAGIC;
    rec.hdr.type = std::to_underlying(FlightLog::RecordType::PYRO_EVENT);
    rec.hdr.timestampMillis = flightTimestampMs();
    rec.channel = channel;
    rec.action = std::to_underlying(action);
    rec.ilmRaw = ilmRaw;

    submitRecord(&rec, sizeof(rec));
}

void FlightLogger::logVoltage(uint16_t vbatRaw, uint16_t vccRaw, uint16_t pyro1Raw, uint16_t pyro2Raw) {
    if (!recording.load(std::memory_order_relaxed)) return;

    FlightLog::VoltageRecord rec{};
    rec.hdr.magic = FlightLog::RECORD_MAGIC;
    rec.hdr.type = std::to_underlying(FlightLog::RecordType::VOLTAGE);
    rec.hdr.timestampMillis = flightTimestampMs();
    rec.vbatRaw = vbatRaw;
    rec.vccRaw = vccRaw;
    rec.pyro1Ilmraw = pyro1Raw;
    rec.pyro2IlmRaw = pyro2Raw;

    submitRecord(&rec, sizeof(rec));
}

int FlightLogger::eraseAll() {
    if (recording.load(std::memory_order_acquire)) {
        LOG_ERR("Cannot erase while recording");
        return -EBUSY;
    }

    if (flashArea == nullptr) {
        return -EINVAL;
    }

    LOG_INF("Erasing raw partition (%u bytes)...", partSize);

    int ret = flash_area_erase(flashArea, 0, partSize);
    if (ret != 0) {
        LOG_ERR("Erase failed: %d", ret);
        return ret;
    }

    writePtr = 0;
    pageOffset = 0;
    memset(pageBuffer, 0xFF, PAGE_SIZE);

    LOG_INF("Erase complete");
    return 0;
}

void FlightLogger::writerEntry(void *arg, void *, void *) {
    auto *self = static_cast<FlightLogger *>(arg);
    self->writerLoop();
}

void FlightLogger::writerLoop() {
    uint8_t recBuf[FlightLog::MAX_RECORD_SIZE];

    while (true) {
        // Block until a record is available or timeout for sync emission
        int ret = k_msgq_get(&msgq, recBuf, K_MSEC(100));

        if (ret == 0) {
            const auto *hdr = reinterpret_cast<const FlightLog::RecordHeader *>(recBuf);
            const size_t recSize = FlightLog::recordSizeFromRaw(hdr->type);

            if (recSize == 0 || recSize > FlightLog::MAX_RECORD_SIZE) {
                LOG_ERR("Bad record type 0x%02X in queue, discarding", hdr->type);
                continue;
            }

            appendToPage(recBuf, recSize);

            // Flight crit events should be flushed immediately to survive potential power loss
            const auto type = static_cast<FlightLog::RecordType>(hdr->type);
            if (type == FlightLog::RecordType::STATE_CHANGE || type == FlightLog::RecordType::PYRO_EVENT ||
                type == FlightLog::RecordType::FLIGHT_END) {
                flushPage();
            }
        }

        // Emit timestamp sync if interval has elapsed
        if (recording.load(std::memory_order_relaxed)) {
            emitTimestampSyncIfNeeded();
        }

        // If flight ended and queue drained, flush the partial page
        if (!recording.load(std::memory_order_acquire) && k_msgq_num_used_get(&msgq) == 0 && pageOffset > 0) {
            flushPage();
        }
    }
}

void FlightLogger::emitTimestampSyncIfNeeded() {
    const uint32_t now = k_uptime_get_32();
    if ((now - lastSyncMs) < SYNC_INTERVAL_MS) {
        return;
    }

    FlightLog::TimestampSyncRecord rec{};
    rec.hdr.magic = FlightLog::RECORD_MAGIC;
    rec.hdr.type = static_cast<uint8_t>(FlightLog::RecordType::TIMESTAMP_SYNC);
    rec.hdr.timestampMillis = flightTimestampMs();
    rec.epochUptimeMs = now;

    appendToPage(&rec, sizeof(rec));
    flightRecordCount++;
    lastSyncMs = now;
}

uint16_t FlightLogger::flightTimestampMs() const {
    const uint32_t now = k_uptime_get_32();
    return static_cast<uint16_t>((now - flightEpochMs) & 0xFFFF);
}

int FlightLogger::writePage() {
    if (pageOffset == 0) {
        return 0;
    }

    if (writePtr + PAGE_SIZE > partSize) {
        LOG_ERR("Raw partition full at offset 0x%08X", writePtr);
        return -ENOSPC;
    }

    // Bytes [pageOffset .. PAGE_SIZE-1] are already 0xFF, so trailing
    // portion is a no-op on flash
    int ret = flash_area_write(flashArea, writePtr, pageBuffer, PAGE_SIZE);
    if (ret != 0) {
        LOG_ERR("Flash write failed at 0x%08X: %d", writePtr, ret);
        return ret;
    }

    writePtr += PAGE_SIZE;
    pageOffset = 0;
    memset(pageBuffer, 0xFF, PAGE_SIZE);

    return 0;
}

int FlightLogger::flushPage() { return writePage(); }

void FlightLogger::appendToPage(const void *record, size_t len) {
    // If this record would overflow the page, write the current page first.
    // This guarantees we never split a record across two pages.
    if (pageOffset + len > PAGE_SIZE) {
        writePage();
    }

    if (len > PAGE_SIZE) {
        LOG_ERR("Record size %zu exceeds page size %zu", len, PAGE_SIZE);
        return;
    }

    memcpy(&pageBuffer[pageOffset], record, len);
    pageOffset += len;

    if (pageOffset >= PAGE_SIZE) {
        writePage();
    }
}

void FlightLogger::submitRecord(const void *record, size_t len) {
    uint8_t buff[FlightLog::MAX_RECORD_SIZE] = {};
    memcpy(buff, record, len);

    int ret = k_msgq_put(&msgq, buff, K_NO_WAIT);
    if (ret != 0) {
        dropped.fetch_add(1, std::memory_order_relaxed);
    } else {
        flightRecordCount++;
    }
}

int FlightLogger::scanForWritePointer() {
    // Jump by each record's fixed size. Stop at 0xFF or corruption.
    uint32_t offset = 0;
    uint32_t lastGoodOffset = 0;
    uint8_t probe;

    while (offset < partSize) {
        int ret = flash_area_read(flashArea, offset, &probe, 1);
        if (ret != 0) {
            LOG_ERR("Flash read error at offset 0x%08X: %d", offset, ret);
            return ret;
        }

        // Erased so this is our write pointer
        if (probe == 0xFF) {
            break;
        }

        // Not magic, not erased so corruption or torn write
        if (probe != FlightLog::RECORD_MAGIC) {
            LOG_WRN("Corruption at 0x%08X (0x%02X), rewinding to 0x%08X", offset, probe, lastGoodOffset);
            offset = lastGoodOffset;
            break;
        }

        // Read type byte to determine record size
        uint8_t typeByte;
        ret = flash_area_read(flashArea, offset + 1, &typeByte, 1);
        if (ret != 0) {
            LOG_ERR("Flash read error at offset 0x%08X: %d", offset + 1, ret);
            return ret;
        }

        const size_t recSize = FlightLog::recordSizeFromRaw(typeByte);
        if (recSize == 0) {
            LOG_WRN("Unknown type 0x%02X at 0x%08X, rewinding to 0x%08X", typeByte, offset, lastGoodOffset);
            offset = lastGoodOffset;
            break;
        }

        if (offset + recSize > partSize) {
            LOG_WRN("Record at 0x%08X overflows partition, rewinding to 0x%08X", offset, lastGoodOffset);
            offset = lastGoodOffset;
            break;
        }

        lastGoodOffset = offset;
        offset += recSize;
    }

    writePtr = offset;
    return 0;
}
