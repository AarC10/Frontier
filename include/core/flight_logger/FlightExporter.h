/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <core/flight_logger/FlightLogger.h>

#include <cstdint>

#include <zephyr/storage/flash_map.h>

class FlightExporter {
public:
	struct FlightInfo {
		uint32_t flightId;
		uint32_t startOffset;
		uint32_t endOffset;
		uint32_t epochUptimeMs;
		uint32_t recordCount;
		bool hasFlightEnd;
	};

	static constexpr size_t MAX_FLIGHTS = 16;

	FlightExporter(const flash_area *rawPartition, const flash_area *fatPartition);

	FlightExporter(const FlightExporter &) = delete;
	FlightExporter &operator=(const FlightExporter &) = delete;

	int init();

	int scanFlights();

	int exportFlight(uint32_t flightId);

	int exportAll();

	int exportLatest();

	int clearExports();

	int format();

	size_t flightCount() const { return numFlights; }
	const FlightInfo *flights() const { return flightList; }
	const FlightInfo *findFlight(uint32_t flightId) const;

	bool isMounted() const { return mounted; }

	static bool fileExists(const char *path);

	static void buildFilename(char *buf, size_t bufLen, uint32_t flightId);

private:
	const flash_area *rawFa;
	const flash_area *fatFa;

	FlightInfo flightList[MAX_FLIGHTS];
	size_t numFlights{0};
	bool mounted{false};

	int mountOrFormat();

	int formatAndMount();

	int copyFlightToFile(const FlightInfo &flight, const char *filename);
};

void flightExporterShellRegister(FlightExporter *exporter);