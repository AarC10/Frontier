/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <core/flight/FlightLogRecords.h>
#include <core/sensors/Barometer.h>
#include <core/sensors/Imu.h>

#include <atomic>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>

// Callers will submit records via k_msgq_put
// Writer thread batch writes 256 byte pages
// Flight ciritical events will force an immediate flush
// Write pointer walks from the start of raw partition
class FlightLogger {
public:
	explicit FlightLogger(uint8_t partitionId);

	FlightLogger(const FlightLogger &) = delete;
	FlightLogger &operator=(const FlightLogger &) = delete;

	/**
	 * Init flash access and recover write ptr
	 * Should be called once from thread ctx before logging
	 *
	 * @return 0 on success
	 */
	int init();

	// ----- Flight Lifecyle -----

	/**
	 * Writes FlightHeader record and starts a writer thread if not running
	 * @param flightId The ID of the flight
	 * @param imuRateHz IMU data rate
	 * @param baroRateHz Barometer data rate
	 * @return 0 on succes
	 */
	int startFlight(uint32_t flightId, uint16_t imuRateHz, uint16_t baroRateHz);

	/**
	 * Writes FlightEnd record and flushes
	 */
	void endFlight();

	// ----- Record submission -----
	// Should be ISR safe and non-blocking

	void logImu(const ImuSample &sample);
	void logBaro(const BaroSample &sample);
	void logStateChange(uint8_t prevState, uint8_t newState);
	void logPyroEvent(uint8_t channel, FlightLog::PyroAction action, uint16_t ilmRaw);
	void logVoltage(uint16_t vbatRaw, uint16_t vccRaw, uint16_t pyro1Raw, uint16_t pyro2Raw);

	// Manage partitions

	// Erase the entire raw partition. Blocking. Only call in PAD or LANDED.

	/**
	 * Erase flash chip ifwe are in pad state
	 * @return 0 on success, -EBUSY if currently recording, or flash error code
	 */
	int eraseAll();

	uint32_t writeOffset() const { return writePtr; }

	uint32_t partitionSize() const { return partSize; }

	bool isRecording() const { return recording.load(std::memory_order_relaxed); }

	/**
	 * Get number of records dropped due to full msgq. Non zero means writer thread can't keep up
	 * @return Records dropped count
	 */
	uint32_t droppedCount() const { return dropped.load(std::memory_order_relaxed); }

private:
	// Flash access
	uint8_t partId;
	const flash_area *fa{nullptr};
	uint32_t partSize{0};
	uint32_t writePtr{0};

	// FLight state
	std::atomic<bool> recording{false};
	uint32_t flightEpochMs{0};
	uint32_t flightRecordCount{0};
	uint32_t lastSyncMs{0};

	// Page buffer
	static constexpr size_t PAGE_SIZE = 256;
	uint8_t pageBuffer[PAGE_SIZE];
	size_t pageOffset{0};

	// Message Queue
	// Depth: 100 Hz IMU + 25 Hz baro = 125 rec/s. 3 ms flash stall queues <1 record. 64 entries gives healthy margin.
	static constexpr size_t MSGQ_DEPTH = 64;
	char msgqBuffer[MSGQ_DEPTH * FlightLog::MAX_RECORD_SIZE];
	k_msgq msgq;

	// Stats
	std::atomic<uint32_t> dropped{0};

	// Write thread
	static constexpr size_t WRITER_STACK_SIZE = 2048;
	static constexpr int WRITER_PRIORITY = 8;
	K_KERNEL_STACK_MEMBER(writerStack, WRITER_STACK_SIZE);
	k_thread writerThread;
	k_tid_t writerTid{nullptr};

	static void writerEntry(void *arg, void *, void *);
	void writerLoop();

	// Timestamp Syncer
	static constexpr uint32_t SYNC_INTERVAL_MS = 30000;
	void emitTimestampSyncIfNeeded();

	uint16_t flightTimestampMs() const;
	int writePage();
	int flushPage();
	void appendToPage(const void *record, size_t len);
	void submitRecord(const void *record, size_t len);
	int scanForWritePointer();

};