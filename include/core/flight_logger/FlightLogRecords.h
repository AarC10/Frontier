/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace FlightLog {

static constexpr uint8_t RECORD_MAGIC = 0xA5; // 0xA5 seems to be the standard
static constexpr uint8_t FORMAT_VERSION = 1;

enum class RecordType : uint8_t {
    FLIGHT_HEADER = 0x01,
    IMU = 0x02,
    BARO = 0x03,
    STATE_CHANGE = 0x04,
    PYRO_EVENT = 0x05,
    VOLTAGE = 0x06,
    TIMESTAMP_SYNC = 0x07,
    FLIGHT_END = 0x08,
};

enum class PyroAction : uint8_t {
    ARM = 0x01,
    FIRE = 0x02,
    FAULT = 0x03,
    DISARM = 0x04,
};

struct __attribute__((packed)) RecordHeader {
    uint8_t magic;
    uint8_t type;
    uint16_t timestampMillis;
};

static_assert(sizeof(RecordHeader) == 4, "RecordHeader must be 4 bytes");

struct __attribute__((packed)) FlightHeaderRecord {
    RecordHeader hdr;
    uint32_t flightId;
    uint32_t epochUptimeMillis;
    uint16_t imuRateHz;
    uint16_t baroRateHz;
    uint8_t formatVersion;
    uint8_t reserved[15]; // Force struct to be 32 bytes to pack evenly into 256 byte page. 8 records per page
};

static_assert(sizeof(FlightHeaderRecord) == 32, "FlightHeaderRecord must be 32 bytes");

struct __attribute__((packed)) ImuRecord {
    RecordHeader hdr;
    int16_t accelXMg;
    int16_t accelYMg;
    int16_t accelZMg;
    int16_t gyroXDdps;
    int16_t gyroYDdps;
    int16_t gyroZDdps;
};

static_assert(sizeof(ImuRecord) == 16, "ImuRecord must be 16 bytes");

struct __attribute__((packed)) BaroRecord {
    RecordHeader hdr;
    uint32_t pressurePa;
    int16_t tempCDeg;
    uint16_t reserved;
};

static_assert(sizeof(BaroRecord) == 12, "BaroRecord must be 12 bytes");

struct __attribute__((packed)) StateChangeRecord {
    RecordHeader hdr;
    uint8_t prevState;
    uint8_t newState;
    uint16_t reserved;
};

static_assert(sizeof(StateChangeRecord) == 8, "StateChangeRecord must be 8 bytes");

struct __attribute__((packed)) PyroEventRecord {
    RecordHeader hdr;
    uint8_t channel;
    uint8_t action;
    uint16_t ilmRaw;
};

static_assert(sizeof(PyroEventRecord) == 8, "PyroEventRecord must be 8 bytes");

struct __attribute__((packed)) VoltageRecord {
    RecordHeader hdr;
    uint16_t vbatRaw;
    uint16_t vccRaw;
    uint16_t pyro1Ilmraw;
    uint16_t pyro2IlmRaw;
};

static_assert(sizeof(VoltageRecord) == 12, "VoltageRecord must be 12 bytes");

struct __attribute__((packed)) TimestampSyncRecord {
    RecordHeader hdr;
    uint32_t epochUptimeMs;
};

static_assert(sizeof(TimestampSyncRecord) == 8, "TimestampSyncRecord must be 8 bytes");

struct __attribute__((packed)) FlightEndRecord {
    RecordHeader hdr;
    uint32_t totalRecords;
};

static_assert(sizeof(FlightEndRecord) == 8, "FlightEndRecord must be 8 bytes");

static constexpr size_t MAX_RECORD_SIZE = sizeof(FlightHeaderRecord);

/**
 * Returns the fixed byte size of a record given its type.
 * Returns 0 for unknown types — the scanner treats this as corruption.
 */
inline constexpr size_t recordSize(RecordType type) {
    switch (type) {
        case RecordType::FLIGHT_HEADER:
            return sizeof(FlightHeaderRecord);
        case RecordType::IMU:
            return sizeof(ImuRecord);
        case RecordType::BARO:
            return sizeof(BaroRecord);
        case RecordType::STATE_CHANGE:
            return sizeof(StateChangeRecord);
        case RecordType::PYRO_EVENT:
            return sizeof(PyroEventRecord);
        case RecordType::VOLTAGE:
            return sizeof(VoltageRecord);
        case RecordType::TIMESTAMP_SYNC:
            return sizeof(TimestampSyncRecord);
        case RecordType::FLIGHT_END:
            return sizeof(FlightEndRecord);
    }
    return 0;
}

/**
 * Returns the fixed byte size from a raw type byte.
 * Convenience wrapper for use during flash scanning.
 */
inline constexpr size_t recordSizeFromRaw(uint8_t rawType) { return recordSize(static_cast<RecordType>(rawType)); }

} // namespace FlightLog