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

#ifdef CONFIG_FAT_FILESYSTEM_ELM
#include <ff.h>
#endif

#ifdef CONFIG_SHELL
#include <zephyr/shell/shell.h>
#endif

LOG_MODULE_REGISTER(FlightExporter);

static constexpr size_t COPY_BUF_SIZE = 512;
static const char *const FAT_MOUNT = "/fat";

#ifdef CONFIG_FAT_FILESYSTEM_ELM
static FATFS fatFs;
static struct fs_mount_t fatMnt = {
	.type = FS_FATFS,
	.mnt_point = FAT_MOUNT,
	.fs_data = &fatFs,
};
#endif


FlightExporter::FlightExporter(const flash_area *rawPartition,
			       const flash_area *fatPartition)
	: rawFa(rawPartition), fatFa(fatPartition) {}

int FlightExporter::init() {
	return mountOrFormat();
}

int FlightExporter::mountOrFormat() {

}

int FlightExporter::formatAndMount() {

}

int FlightExporter::format() {
	return formatAndMount();
}

int FlightExporter::scanFlights() {

}

const FlightExporter::FlightInfo *FlightExporter::findFlight(uint32_t flightId) const {
	
}

void FlightExporter::buildFilename(char *buf, size_t bufLen, uint32_t flightId) {
	snprintf(buf, bufLen, "%s/FLIGHT_%03u.BIN", FAT_MOUNT, flightId);
}

bool FlightExporter::fileExists(const char *path) {
	struct fs_dirent entry;
	return fs_stat(path, &entry) == 0;
}

// ──────────────────── Raw -> file copy ──────────────────────────────

int FlightExporter::copyFlightToFile(const FlightInfo &flight,
				     const char *filename) {

}

int FlightExporter::exportFlight(uint32_t flightId) {

}

int FlightExporter::exportAll() {
}


int FlightExporter::clearExports() {

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

	shell_print(sh, "Exporting flight %lu...", id);
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

	shell_print(sh, "Exporting %zu flight(s)...", shellExporter->flightCount());
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

	shell_print(sh, "Formatting FAT partition...");
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