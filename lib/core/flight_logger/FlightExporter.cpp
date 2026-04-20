/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "core/flight_logger/FlightExporter.h"

#include <cstdio>
#include <cstring>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>


#ifdef CONFIG_SHELL
#include <zephyr/shell/shell.h>
#endif

LOG_MODULE_REGISTER(FlightExporter, LOG_LEVEL_INF);

static constexpr size_t COPY_BUF_SIZE = 512;
static const char *const FAT_MOUNT = "/marshal:";

namespace {

constexpr uint32_t kFlightLogPageSize = 256U;
constexpr uint32_t kFilesystemReserveBytes = 128U * 1024U;

uint32_t flightSizeBytes(const FlightExporter::FlightInfo &flight) {
	return flight.endOffset - flight.startOffset;
}

uint32_t chunkCountForSize(uint32_t totalSize, uint32_t chunkSize) {
	if (chunkSize == 0U) {
		return 0U;
	}

	return (totalSize + chunkSize - 1U) / chunkSize;
}

bool scanPageForRecords(const flash_area *flashArea, uint32_t pageBase, uint32_t partSize, uint32_t &consumedBytes) {
	consumedBytes = 0U;

	while (consumedBytes < kFlightLogPageSize && (pageBase + consumedBytes) < partSize) {
		uint8_t probe[2] = {0xFF, 0xFF};
		const int ret = flash_area_read(flashArea, pageBase + consumedBytes, probe, sizeof(probe));
		if (ret != 0) {
			LOG_ERR("Flash read error at 0x%08X: %d", pageBase + consumedBytes, ret);
			return false;
		}

		if (probe[0] == 0xFF) {
			return true;
		}

		if (probe[0] != FlightLog::RECORD_MAGIC) {
			LOG_WRN("Corruption at 0x%08X, stopping scan", pageBase + consumedBytes);
			return false;
		}

		const size_t recSize = FlightLog::recordSizeFromRaw(probe[1]);
		if (recSize == 0U) {
			LOG_WRN("Unknown type 0x%02X at 0x%08X, stopping scan", probe[1], pageBase + consumedBytes);
			return false;
		}

		if ((consumedBytes + recSize) > kFlightLogPageSize || (pageBase + consumedBytes + recSize) > partSize) {
			LOG_WRN("Record at 0x%08X overflows page or partition, stopping scan", pageBase + consumedBytes);
			return false;
		}

		consumedBytes += static_cast<uint32_t>(recSize);
	}

	return true;
}

} // namespace

static uint8_t __aligned(4) fatFsData[1024];
static struct fs_mount_t fatMnt = {
	.type = FS_FATFS,
	.mnt_point = FAT_MOUNT,
	.fs_data = &fatFsData,
};

FlightExporter::FlightExporter(const flash_area *rawPartition,
			       const flash_area *fatPartition)
	: rawFa(rawPartition), fatFa(fatPartition) {}

int FlightExporter::init() {
	return mountOrFormat();
}

int FlightExporter::mountOrFormat() {
	int ret = fs_mount(&fatMnt);
	if (ret == 0) {
		mounted = true;
		LOG_INF("FAT partition mounted at %s", FAT_MOUNT);
		return 0;
	}

	LOG_WRN("FAT mount failed (%d), formatting...", ret);
	return formatAndMount();
}

int FlightExporter::formatAndMount() {
	if (fatFa == nullptr) {
		return -EINVAL;
	}

	// Unmount first if somehow mounted
	if (mounted) {
		fs_unmount(&fatMnt);
		mounted = false;
	}

	LOG_INF("Erasing FAT partition (%u bytes)", fatFa->fa_size);
	int ret = flash_area_erase(fatFa, 0, fatFa->fa_size);
	if (ret != 0) {
		LOG_ERR("FAT erase failed: %d", ret);
		return ret;
	}

	ret = fs_mkfs(FS_FATFS, reinterpret_cast<uintptr_t>("marshal"), nullptr, 0);
	if (ret != 0) {
		LOG_ERR("fs_mkfs failed: %d", ret);
		return ret;
	}

	ret = fs_mount(&fatMnt);
	if (ret != 0) {
		LOG_ERR("FAT mount after format failed: %d", ret);
		return ret;
	}
	mounted = true;
	LOG_INF("FAT partition formatted and mounted at %s", FAT_MOUNT);
	return 0;
}

int FlightExporter::format() {
	return formatAndMount();
}

int FlightExporter::scanFlights() {
	if (rawFa == nullptr) {
		return -EINVAL;
	}

	numFlights = 0;
	uint32_t pageBase = 0;
	const uint32_t partSize = rawFa->fa_size;
	int currentFlight = -1;

	while (pageBase < partSize) {
		uint8_t firstByte = 0xFF;
		int ret = flash_area_read(rawFa, pageBase, &firstByte, 1);
		if (ret != 0) {
			LOG_ERR("Flash read error at 0x%08X: %d", pageBase, ret);
			return ret;
		}

		if (firstByte == 0xFF) {
			break;
		}

		uint32_t pageOffset = 0U;
		while (pageOffset < kFlightLogPageSize && (pageBase + pageOffset) < partSize) {
			uint8_t probe[2] = {0xFF, 0xFF};
			ret = flash_area_read(rawFa, pageBase + pageOffset, probe, sizeof(probe));
			if (ret != 0) {
				LOG_ERR("Flash read error at 0x%08X: %d", pageBase + pageOffset, ret);
				return ret;
			}

			if (probe[0] == 0xFF) {
				break;
			}

			if (probe[0] != FlightLog::RECORD_MAGIC) {
				LOG_WRN("Corruption at 0x%08X, stopping scan", pageBase + pageOffset);
				goto scan_done;
			}

			const uint8_t typeByte = probe[1];
			const size_t recSize = FlightLog::recordSizeFromRaw(typeByte);
			if (recSize == 0) {
				LOG_WRN("Unknown type 0x%02X at 0x%08X, stopping scan", typeByte, pageBase + pageOffset);
				goto scan_done;
			}

			if ((pageOffset + recSize) > kFlightLogPageSize || (pageBase + pageOffset + recSize) > partSize) {
				LOG_WRN("Record at 0x%08X overflows page or partition, stopping scan", pageBase + pageOffset);
				goto scan_done;
			}

			const auto type = static_cast<FlightLog::RecordType>(typeByte);

			if (type == FlightLog::RecordType::FLIGHT_HEADER) {
				FlightLog::FlightHeaderRecord hdr{};
				ret = flash_area_read(rawFa, pageBase + pageOffset, &hdr, sizeof(hdr));
				if (ret != 0) {
					LOG_ERR("Failed to read flight header at 0x%08X", pageBase + pageOffset);
					goto scan_done;
				}

				if (currentFlight >= 0) {
					flightList[currentFlight].endOffset = pageBase + pageOffset;
				}

				if (numFlights >= MAX_FLIGHTS) {
					LOG_WRN("Max flights (%zu) reached, stopping scan", MAX_FLIGHTS);
					goto scan_done;
				}

				currentFlight = static_cast<int>(numFlights);
				FlightInfo &info = flightList[numFlights];
				info.flightId = hdr.flightId;
				info.startOffset = pageBase + pageOffset;
				info.endOffset = 0;
				info.epochUptimeMs = hdr.epochUptimeMillis;
				info.recordCount = 0;
				info.hasFlightEnd = false;
				numFlights++;
			}

			if (type == FlightLog::RecordType::FLIGHT_END && currentFlight >= 0) {
				flightList[currentFlight].hasFlightEnd = true;
			}

			if (currentFlight >= 0) {
				flightList[currentFlight].recordCount++;
			}

			pageOffset += static_cast<uint32_t>(recSize);
		}

		uint32_t consumedBytes = 0U;
		if (!scanPageForRecords(rawFa, pageBase, partSize, consumedBytes)) {
			break;
		}

		pageBase += kFlightLogPageSize;
	}

scan_done:
	if (currentFlight >= 0 && flightList[currentFlight].endOffset == 0) {
		flightList[currentFlight].endOffset = pageBase;
	}

	LOG_INF("Scan complete: %zu flight(s) found", numFlights);
	return static_cast<int>(numFlights);
}

const FlightExporter::FlightInfo *FlightExporter::findFlight(uint32_t flightId) const {
	for (size_t i = 0; i < numFlights; i++) {
		if (flightList[i].flightId == flightId) {
			return &flightList[i];
		}
	}
	return nullptr;
}

void FlightExporter::buildFilename(char *buf, size_t bufLen, uint32_t flightId) {
	// FAT 8.3 limit: 8 base + 3 ext. "FLT_NNN" = 7 chars, fits cleanly.
	snprintf(buf, bufLen, "%s/FLT_%03u.BIN", FAT_MOUNT, flightId);
}

void FlightExporter::buildChunkFilename(char *buf, size_t bufLen, uint32_t flightId, uint32_t chunkIndex) {
	// FAT 8.3 limit: "FNNN_NN" = 7 chars before the extension.
	snprintf(buf, bufLen, "%s/F%03u_%02u.BIN", FAT_MOUNT, flightId, chunkIndex);
}

bool FlightExporter::fileExists(const char *path) {
	struct fs_dirent entry;
	return fs_stat(path, &entry) == 0;
}

uint32_t FlightExporter::maxSingleFileExportSize() const {
	if (fatFa == nullptr || fatFa->fa_size <= kFilesystemReserveBytes) {
		return 0U;
	}

	return fatFa->fa_size - kFilesystemReserveBytes;
}

uint32_t FlightExporter::recommendedChunkSize() const {
	const uint32_t maxExportSize = maxSingleFileExportSize();
	if (maxExportSize == 0U) {
		return 0U;
	}

	return (DEFAULT_CHUNK_SIZE_BYTES < maxExportSize) ? DEFAULT_CHUNK_SIZE_BYTES : maxExportSize;
}

int FlightExporter::copyRangeToFile(uint32_t offset, uint32_t size, const char *filename) {
	if (size == 0U) {
		return -EINVAL;
	}

	struct fs_file_t file;
	fs_file_t_init(&file);

	int ret = fs_open(&file, filename, FS_O_CREATE | FS_O_WRITE);
	if (ret != 0) {
		LOG_ERR("Failed to create %s: %d", filename, ret);
		return ret;
	}

	uint8_t buf[COPY_BUF_SIZE];
	uint32_t remaining = size;
	uint32_t cursor = offset;

	while (remaining > 0) {
		const size_t chunk = (remaining > COPY_BUF_SIZE)
			? COPY_BUF_SIZE : remaining;

		ret = flash_area_read(rawFa, cursor, buf, chunk);
		if (ret != 0) {
			LOG_ERR("Flash read failed at 0x%08X: %d", cursor, ret);
			fs_close(&file);
			return ret;
		}

		ssize_t written = fs_write(&file, buf, chunk);
		if (written < 0) {
			LOG_ERR("File write failed: %zd", written);
			fs_close(&file);
			return static_cast<int>(written);
		}

		cursor += chunk;
		remaining -= chunk;
	}

	fs_close(&file);
	return 0;
}

int FlightExporter::copyFlightToFile(const FlightInfo &flight,
				     const char *filename) {
	const uint32_t size = flightSizeBytes(flight);
	const int ret = copyRangeToFile(flight.startOffset, size, filename);
	if (ret != 0) {
		return ret;
	}

	LOG_INF("Exported flight %u: %u bytes -> %s",
		flight.flightId, size, filename);
	return 0;
}

int FlightExporter::exportFlight(uint32_t flightId) {
	if (!mounted) {
		LOG_ERR("FAT not mounted");
		return -ENODEV;
	}

	const FlightInfo *info = findFlight(flightId);
	if (info == nullptr) {
		LOG_ERR("Flight %u not found", flightId);
		return -ENOENT;
	}

	const uint32_t size = flightSizeBytes(*info);
	const uint32_t maxExportSize = maxSingleFileExportSize();
	if (size > maxExportSize) {
		LOG_WRN("Flight %u (%u bytes) exceeds single-file export limit (%u bytes)",
			flightId, size, maxExportSize);
		return -EFBIG;
	}

	char filename[32];
	buildFilename(filename, sizeof(filename), flightId);

	if (fileExists(filename)) {
		LOG_INF("Flight %u already exported (%s), skipping", flightId, filename);
		return -EEXIST;
	}

	return copyFlightToFile(*info, filename);
}

int FlightExporter::exportFlightChunk(uint32_t flightId, uint32_t chunkIndex) {
	if (!mounted) {
		LOG_ERR("FAT not mounted");
		return -ENODEV;
	}

	if (chunkIndex == 0U) {
		return -EINVAL;
	}

	const FlightInfo *info = findFlight(flightId);
	if (info == nullptr) {
		LOG_ERR("Flight %u not found", flightId);
		return -ENOENT;
	}

	const uint32_t chunkSize = recommendedChunkSize();
	if (chunkSize == 0U) {
		return -ENOSPC;
	}

	const uint32_t size = flightSizeBytes(*info);
	const uint32_t chunkCount = chunkCountForSize(size, chunkSize);
	if (chunkIndex > chunkCount) {
		return -ERANGE;
	}

	const uint32_t chunkOffset = (chunkIndex - 1U) * chunkSize;
	const uint32_t remaining = size - chunkOffset;
	const uint32_t bytesToWrite = (remaining > chunkSize) ? chunkSize : remaining;

	char filename[32];
	buildChunkFilename(filename, sizeof(filename), flightId, chunkIndex);

	if (fileExists(filename)) {
		LOG_INF("Flight %u chunk %u already exported (%s), skipping",
			flightId, chunkIndex, filename);
		return -EEXIST;
	}

	const int ret = copyRangeToFile(info->startOffset + chunkOffset, bytesToWrite, filename);
	if (ret != 0) {
		return ret;
	}

	LOG_INF("Exported flight %u chunk %u/%u: %u bytes -> %s",
		flightId, chunkIndex, chunkCount, bytesToWrite, filename);
	return 0;
}

int FlightExporter::exportAll() {
	if (!mounted) {
		LOG_ERR("FAT not mounted");
		return -ENODEV;
	}

	if (numFlights == 0) {
		LOG_WRN("No flights to export");
		return 0;
	}

	int exported = 0;
	for (size_t i = 0; i < numFlights; i++) {
		const uint32_t size = flightSizeBytes(flightList[i]);
		if (size > maxSingleFileExportSize()) {
			LOG_WRN("Skipping flight %u: %u bytes exceeds single-file export limit",
				flightList[i].flightId, size);
			continue;
		}

		char filename[32];
		buildFilename(filename, sizeof(filename), flightList[i].flightId);

		if (fileExists(filename)) {
			LOG_INF("Flight %u already exported, skipping",
				flightList[i].flightId);
			continue;
		}

		int ret = copyFlightToFile(flightList[i], filename);
		if (ret != 0) {
			LOG_ERR("Failed to export flight %u: %d",
				flightList[i].flightId, ret);
			continue;
		}
		exported++;
	}

	LOG_INF("Exported %d new flight(s) (%zu total on partition)",
		exported, numFlights);
	return exported;
}

int FlightExporter::exportLatest() {
	int ret = scanFlights();
	if (ret < 0) {
		return ret;
	}

	if (numFlights == 0) {
		return -ENOENT;
	}

	// The last flight in the list is the most recent
	const FlightInfo &latest = flightList[numFlights - 1];
	return exportFlight(latest.flightId);
}

int FlightExporter::clearExports() {
	if (!mounted) {
		LOG_ERR("FAT not mounted");
		return -ENODEV;
	}

	struct fs_dir_t dir;
	fs_dir_t_init(&dir);

	int ret = fs_opendir(&dir, FAT_MOUNT);
	if (ret != 0) {
		LOG_ERR("Failed to open %s: %d", FAT_MOUNT, ret);
		return ret;
	}

	int deleted = 0;
	struct fs_dirent entry;

	while (fs_readdir(&dir, &entry) == 0 && entry.name[0] != '\0') {
		// Only delete .BIN files
		const size_t len = strlen(entry.name);
		if (len < 4) continue;

		const char *ext = &entry.name[len - 4];
		if (strcmp(ext, ".BIN") != 0 && strcmp(ext, ".bin") != 0) {
			continue;
		}

		char path[48];
		snprintf(path, sizeof(path), "%s/%s", FAT_MOUNT, entry.name);

		ret = fs_unlink(path);
		if (ret == 0) {
			deleted++;
		} else {
			LOG_WRN("Failed to delete %s: %d", path, ret);
		}
	}

	fs_closedir(&dir);
	LOG_INF("Deleted %d file(s)", deleted);
	return 0;
}

#ifdef CONFIG_SHELL

static FlightExporter *shellExporter = nullptr;
static FlightLogger *shellLogger = nullptr;

void flightExporterShellRegister(FlightExporter *exporter) {
	shellExporter = exporter;
}

void flightLoggerShellRegister(FlightLogger *logger) {
	shellLogger = logger;
}

static int cmdList(const struct shell *sh, size_t argc, char **argv) {
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (shellExporter == nullptr) {
		shell_error(sh, "Exporter not initialized");
		return -EINVAL;
	}

	int ret = shellExporter->scanFlights();
	if (ret < 0) {
		shell_error(sh, "Scan failed: %d", ret);
		return ret;
	}

	if (shellExporter->flightCount() == 0) {
		shell_print(sh, "No flights recorded");
		return 0;
	}

	shell_print(sh, "  ID   Offset       Size     Records  Complete  Exported");
	shell_print(sh, " ---  ----------  ---------  -------  --------  --------");

	for (size_t i = 0; i < shellExporter->flightCount(); i++) {
		const auto &f = shellExporter->flights()[i];
		const uint32_t size = f.endOffset - f.startOffset;

		char filename[32];
		FlightExporter::buildFilename(filename, sizeof(filename), f.flightId);
		const bool exported = FlightExporter::fileExists(filename);

		shell_print(sh, " %3u  0x%08X  %7u B  %7u  %-8s  %s",
			    f.flightId, f.startOffset, size,
			    f.recordCount,
			    f.hasFlightEnd ? "yes" : "no",
			    exported ? "yes" : "no");
	}

	return 0;
}

static int cmdParts(const struct shell *sh, size_t argc, char **argv) {
	ARG_UNUSED(argc);

	if (shellExporter == nullptr) {
		shell_error(sh, "Exporter not initialized");
		return -EINVAL;
	}

	int ret = shellExporter->scanFlights();
	if (ret < 0) {
		shell_error(sh, "Scan failed: %d", ret);
		return ret;
	}

	char *end;
	const unsigned long id = strtoul(argv[1], &end, 10);
	if (*end != '\0') {
		shell_error(sh, "Invalid flight ID '%s'", argv[1]);
		return -EINVAL;
	}

	const FlightExporter::FlightInfo *flight = shellExporter->findFlight(static_cast<uint32_t>(id));
	if (flight == nullptr) {
		shell_error(sh, "Flight %lu not found", id);
		return -ENOENT;
	}

	const uint32_t size = flightSizeBytes(*flight);
	const uint32_t singleFileLimit = shellExporter->maxSingleFileExportSize();
	const uint32_t chunkSize = shellExporter->recommendedChunkSize();
	const uint32_t chunkCount = chunkCountForSize(size, chunkSize);

	shell_print(sh, "Flight %lu size: %u bytes", id, size);
	shell_print(sh, "Single-file export limit: %u bytes", singleFileLimit);
	if (size <= singleFileLimit) {
		shell_print(sh, "Fits as a single file: FLT_%03lu.BIN", id);
		return 0;
	}

	shell_print(sh, "Requires %u chunk(s) of up to %u bytes", chunkCount, chunkSize);
	shell_print(sh, "Use: flight export %lu <part>   where <part> is 1-%u", id, chunkCount);
	shell_print(sh, "Copy each chunk off the USB drive before exporting the next one.");
	return 0;
}

static int cmdExport(const struct shell *sh, size_t argc, char **argv) {
	if (shellExporter == nullptr) {
		shell_error(sh, "Exporter not initialized");
		return -EINVAL;
	}

	int ret = shellExporter->scanFlights();
	if (ret < 0) {
		shell_error(sh, "Scan failed: %d", ret);
		return ret;
	}

	char *end;
	const unsigned long id = strtoul(argv[1], &end, 10);
	if (*end != '\0') {
		shell_error(sh, "Invalid flight ID '%s'", argv[1]);
		return -EINVAL;
	}

	const uint32_t flightId = static_cast<uint32_t>(id);
	const bool exportChunk = (argc >= 3);
	uint32_t chunkIndex = 0U;
	if (exportChunk) {
		const unsigned long part = strtoul(argv[2], &end, 10);
		if (*end != '\0' || part == 0UL) {
			shell_error(sh, "Invalid part index '%s'", argv[2]);
			return -EINVAL;
		}
		chunkIndex = static_cast<uint32_t>(part);
	}

	if (exportChunk) {
		shell_print(sh, "Exporting flight %lu chunk %u", id, chunkIndex);
		ret = shellExporter->exportFlightChunk(flightId, chunkIndex);
	} else {
		shell_print(sh, "Exporting flight %lu", id);
		ret = shellExporter->exportFlight(flightId);
	}

	if (ret == 0) {
		if (exportChunk) {
			char filename[32];
			FlightExporter::buildChunkFilename(filename, sizeof(filename), flightId, chunkIndex);
			shell_print(sh, "Done. %s ready on USB drive.", filename + strlen(FAT_MOUNT) + 1);
			shell_print(sh, "Copy it off the USB drive before exporting the next chunk.");
		} else {
			shell_print(sh, "Done. FLT_%03lu.BIN ready on USB drive.", id);
		}
	} else if (ret == -EEXIST) {
		if (exportChunk) {
			shell_print(sh, "Flight %lu chunk %u already exported.", id, chunkIndex);
		} else {
			shell_print(sh, "Flight %lu already exported.", id);
		}
	} else if (ret == -ERANGE && exportChunk) {
		const FlightExporter::FlightInfo *flight = shellExporter->findFlight(flightId);
		const uint32_t chunkCount = (flight == nullptr)
			? 0U
			: chunkCountForSize(flightSizeBytes(*flight), shellExporter->recommendedChunkSize());
		shell_error(sh, "Part index out of range. Valid parts: 1-%u", chunkCount);
	} else if (ret == -EFBIG && !exportChunk) {
		const FlightExporter::FlightInfo *flight = shellExporter->findFlight(flightId);
		const uint32_t chunkCount = (flight == nullptr)
			? 0U
			: chunkCountForSize(flightSizeBytes(*flight), shellExporter->recommendedChunkSize());
		shell_error(sh, "Flight %lu is too large for a single file on this partition.", id);
		shell_print(sh, "Use: flight parts %lu", id);
		shell_print(sh, "Or export a chunk directly with: flight export %lu <part>   (1-%u)", id, chunkCount);
	} else {
		if (ret == -ENOSPC && exportChunk) {
			shell_error(sh, "Export failed: FAT partition is full. Copy existing files off and run 'flight clear'.");
		} else {
			shell_error(sh, "Export failed: %d", ret);
		}
	}
	return ret;
}

static int cmdExportAll(const struct shell *sh, size_t argc, char **argv) {
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (shellExporter == nullptr) {
		shell_error(sh, "Exporter not initialized");
		return -EINVAL;
	}

	int ret = shellExporter->scanFlights();
	if (ret < 0) {
		shell_error(sh, "Scan failed: %d", ret);
		return ret;
	}

	shell_print(sh, "Exporting %zu flight(s)", shellExporter->flightCount());
	ret = shellExporter->exportAll();

	if (ret >= 0) {
		shell_print(sh, "Done. %d new file(s) written.", ret);
	} else {
		shell_error(sh, "Export failed: %d", ret);
	}
	return (ret >= 0) ? 0 : ret;
}

static int cmdClear(const struct shell *sh, size_t argc, char **argv) {
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (shellExporter == nullptr) {
		shell_error(sh, "Exporter not initialized");
		return -EINVAL;
	}

	int ret = shellExporter->clearExports();
	if (ret == 0) {
		shell_print(sh, "All exported files deleted.");
	} else {
		shell_error(sh, "Clear failed: %d", ret);
	}
	return ret;
}

static int cmdFormat(const struct shell *sh, size_t argc, char **argv) {
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (shellExporter == nullptr) {
		shell_error(sh, "Exporter not initialized");
		return -EINVAL;
	}

	shell_print(sh, "Formatting FAT partition");
	int ret = shellExporter->format();
	if (ret == 0) {
		shell_print(sh, "Format complete.");
	} else {
		shell_error(sh, "Format failed: %d", ret);
	}
	return ret;
}

static int cmdErase(const struct shell *sh, size_t argc, char **argv) {
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (shellLogger == nullptr) {
		shell_error(sh, "Logger not initialized");
		return -EINVAL;
	}

	shell_print(sh, "Erasing raw log partition...");
	int ret = shellLogger->eraseAll();
	if (ret == 0) {
		shell_print(sh, "Raw partition erased.");
	} else if (ret == -EBUSY) {
		shell_error(sh, "Cannot erase while recording.");
	} else {
		shell_error(sh, "Erase failed: %d", ret);
	}
	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(subFlight,
	SHELL_CMD(list, NULL, "List recorded flights", cmdList),
	SHELL_CMD_ARG(export, NULL,
		"Export a whole flight or a numbered chunk (e.g. flight export 3 or flight export 3 1)", cmdExport, 2, 1),
	SHELL_CMD_ARG(parts, NULL,
		"Show chunk info for an oversized flight (e.g. flight parts 3)", cmdParts, 2, 0),
	SHELL_CMD(export_all, NULL,
		"Export all flights (skips existing)", cmdExportAll),
	SHELL_CMD(clear, NULL,
		"Delete all exported .BIN files", cmdClear),
	SHELL_CMD(format, NULL,
		"Erase and reformat the FAT partition", cmdFormat),
	SHELL_CMD(erase, NULL,
		"Erase the raw log partition (destroys all recorded flights)", cmdErase),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(flight, &subFlight, "Flight data management", NULL);

#endif // CONFIG_SHELL
