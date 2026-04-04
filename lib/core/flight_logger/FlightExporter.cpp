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
	uint32_t offset = 0;
	const uint32_t partSize = rawFa->fa_size;
	int currentFlight = -1;

	while (offset < partSize) {
		uint8_t probe[2];
		int ret = flash_area_read(rawFa, offset, probe, 2);
		if (ret != 0) {
			LOG_ERR("Flash read error at 0x%08X: %d", offset, ret);
			return ret;
		}

		if (probe[0] == 0xFF) {
			break;
		}

		if (probe[0] != FlightLog::RECORD_MAGIC) {
			LOG_WRN("Corruption at 0x%08X, stopping scan", offset);
			break;
		}

		const uint8_t typeByte = probe[1];
		const size_t recSize = FlightLog::recordSizeFromRaw(typeByte);
		if (recSize == 0) {
			LOG_WRN("Unknown type 0x%02X at 0x%08X, stopping scan",
				typeByte, offset);
			break;
		}

		if (offset + recSize > partSize) {
			break;
		}

		const auto type = static_cast<FlightLog::RecordType>(typeByte);

		if (type == FlightLog::RecordType::FLIGHT_HEADER) {
			FlightLog::FlightHeaderRecord hdr{};
			ret = flash_area_read(rawFa, offset, &hdr, sizeof(hdr));
			if (ret != 0) {
				LOG_ERR("Failed to read flight header at 0x%08X", offset);
				break;
			}

			if (currentFlight >= 0) {
				flightList[currentFlight].endOffset = offset;
			}

			if (numFlights >= MAX_FLIGHTS) {
				LOG_WRN("Max flights (%zu) reached, stopping scan",
					MAX_FLIGHTS);
				break;
			}

			currentFlight = static_cast<int>(numFlights);
			FlightInfo &info = flightList[numFlights];
			info.flightId = hdr.flightId;
			info.startOffset = offset;
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

		offset += recSize;
	}

	if (currentFlight >= 0 && flightList[currentFlight].endOffset == 0) {
		flightList[currentFlight].endOffset = offset;
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
	snprintf(buf, bufLen, "%s/FLIGHT_%03u.BIN", FAT_MOUNT, flightId);
}

bool FlightExporter::fileExists(const char *path) {
	struct fs_dirent entry;
	return fs_stat(path, &entry) == 0;
}

int FlightExporter::copyFlightToFile(const FlightInfo &flight,
				     const char *filename) {
	const uint32_t size = flight.endOffset - flight.startOffset;

	struct fs_file_t file;
	fs_file_t_init(&file);

	int ret = fs_open(&file, filename, FS_O_CREATE | FS_O_WRITE);
	if (ret != 0) {
		LOG_ERR("Failed to create %s: %d", filename, ret);
		return ret;
	}

	uint8_t buf[COPY_BUF_SIZE];
	uint32_t remaining = size;
	uint32_t offset = flight.startOffset;

	while (remaining > 0) {
		const size_t chunk = (remaining > COPY_BUF_SIZE)
			? COPY_BUF_SIZE : remaining;

		ret = flash_area_read(rawFa, offset, buf, chunk);
		if (ret != 0) {
			LOG_ERR("Flash read failed at 0x%08X: %d", offset, ret);
			fs_close(&file);
			return ret;
		}

		ssize_t written = fs_write(&file, buf, chunk);
		if (written < 0) {
			LOG_ERR("File write failed: %zd", written);
			fs_close(&file);
			return static_cast<int>(written);
		}

		offset += chunk;
		remaining -= chunk;
	}

	fs_close(&file);
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

	char filename[32];
	buildFilename(filename, sizeof(filename), flightId);

	if (fileExists(filename)) {
		LOG_INF("Flight %u already exported (%s), skipping", flightId, filename);
		return -EEXIST;
	}

	return copyFlightToFile(*info, filename);
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

void flightExporterShellRegister(FlightExporter *exporter) {
	shellExporter = exporter;
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

	shell_print(sh, "Exporting flight %lu", id);
	ret = shellExporter->exportFlight(static_cast<uint32_t>(id));

	if (ret == 0) {
		shell_print(sh, "Done. FLIGHT_%03lu.BIN ready on USB drive.", id);
	} else if (ret == -EEXIST) {
		shell_print(sh, "Flight %lu already exported.", id);
	} else {
		shell_error(sh, "Export failed: %d", ret);
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

SHELL_STATIC_SUBCMD_SET_CREATE(subFlight,
	SHELL_CMD(list, NULL, "List recorded flights", cmdList),
	SHELL_CMD_ARG(export, NULL,
		"Export a flight by ID (e.g. flight export 3)", cmdExport, 2, 0),
	SHELL_CMD(export_all, NULL,
		"Export all flights (skips existing)", cmdExportAll),
	SHELL_CMD(clear, NULL,
		"Delete all exported .BIN files", cmdClear),
	SHELL_CMD(format, NULL,
		"Erase and reformat the FAT partition", cmdFormat),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(flight, &subFlight, "Flight data management", NULL);

#endif // CONFIG_SHELL
