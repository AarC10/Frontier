#!/usr/bin/env python3

"""Parse Frontier flight log binaries into CSV."""

from __future__ import annotations

import argparse
import csv
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO


PAGE_SIZE = 256
RECORD_MAGIC = 0xA5
FORMAT_VERSION = 1


STATE_NAMES = {
    0: "PAD",
    1: "BOOST",
    2: "COAST",
    3: "APOGEE",
    4: "DESCENT",
    5: "LANDED",
}

PYRO_ACTION_NAMES = {
    0x01: "ARM",
    0x02: "FIRE",
    0x03: "FAULT",
    0x04: "DISARM",
}


CSV_FIELDS = [
    "flight_index",
    "offset",
    "page_index",
    "page_offset",
    "record_type",
    "record_size",
    "timestamp_raw_ms",
    "flight_time_ms",
    "epoch_uptime_ms",
    "flight_id",
    "imu_rate_hz",
    "baro_rate_hz",
    "format_version",
    "accel_x_mg",
    "accel_y_mg",
    "accel_z_mg",
    "accel_x_g",
    "accel_y_g",
    "accel_z_g",
    "gyro_x_ddps",
    "gyro_y_ddps",
    "gyro_z_ddps",
    "gyro_x_dps",
    "gyro_y_dps",
    "gyro_z_dps",
    "pressure_pa",
    "pressure_kpa",
    "temp_cdeg",
    "temp_c",
    "prev_state",
    "prev_state_name",
    "new_state",
    "new_state_name",
    "pyro_channel",
    "pyro_action",
    "pyro_action_name",
    "ilm_raw",
    "vbat_raw",
    "vcc_raw",
    "pyro1_ilm_raw",
    "pyro2_ilm_raw",
    "timestamp_sync_epoch_uptime_ms",
    "total_records",
]


@dataclass(frozen=True)
class RecordDef:
    type_name: str
    struct_fmt: struct.Struct

    @property
    def size(self) -> int:
        return self.struct_fmt.size


RECORD_DEFS = {
    0x01: RecordDef("FLIGHT_HEADER", struct.Struct("<BBHIIHHB15s")),
    0x02: RecordDef("IMU", struct.Struct("<BBHhhhhhh")),
    0x03: RecordDef("BARO", struct.Struct("<BBHIhH")),
    0x04: RecordDef("STATE_CHANGE", struct.Struct("<BBHBBH")),
    0x05: RecordDef("PYRO_EVENT", struct.Struct("<BBHBBH")),
    0x06: RecordDef("VOLTAGE", struct.Struct("<BBHHHHH")),
    0x07: RecordDef("TIMESTAMP_SYNC", struct.Struct("<BBHI")),
    0x08: RecordDef("FLIGHT_END", struct.Struct("<BBHI")),
}


class ParseError(RuntimeError):
    """Raised when the binary log is malformed."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Parse Frontier flight log binaries into CSV."
    )
    parser.add_argument("bin_path", type=Path, help="Path to the input .BIN file")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Output CSV path. Defaults to the input path with a .csv suffix.",
    )
    return parser.parse_args()


def decode_flight_time(
    timestamp_raw: int, prev_raw: int | None, wrap_count: int
) -> tuple[int, int]:
    if prev_raw is not None and timestamp_raw < prev_raw:
        wrap_count += 1
    return timestamp_raw + wrap_count * 0x10000, wrap_count


def parse_records(data: bytes) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    offset = 0
    prev_timestamp_raw: int | None = None
    timestamp_wrap_count = 0
    flight_epoch_uptime_ms: int | None = None
    flight_index = -1

    while offset < len(data):
        page_offset = offset % PAGE_SIZE

        if data[offset] == 0xFF:
            offset += PAGE_SIZE - page_offset if page_offset != 0 else PAGE_SIZE
            continue

        if data[offset] != RECORD_MAGIC:
            raise ParseError(
                f"unexpected byte 0x{data[offset]:02X} at offset 0x{offset:08X}"
            )

        if offset + 2 > len(data):
            raise ParseError(f"truncated record header at offset 0x{offset:08X}")

        raw_type = data[offset + 1]
        record_def = RECORD_DEFS.get(raw_type)
        if record_def is None:
            raise ParseError(
                f"unknown record type 0x{raw_type:02X} at offset 0x{offset:08X}"
            )

        if page_offset + record_def.size > PAGE_SIZE:
            raise ParseError(
                f"record crosses page boundary at offset 0x{offset:08X}"
            )

        record_end = offset + record_def.size
        if record_end > len(data):
            raise ParseError(f"truncated {record_def.type_name} at offset 0x{offset:08X}")

        unpacked = record_def.struct_fmt.unpack(data[offset:record_end])
        _, _, timestamp_raw, *fields = unpacked

        if raw_type == 0x01:
            flight_index += 1
            prev_timestamp_raw = None
            timestamp_wrap_count = 0

        flight_time_ms, timestamp_wrap_count = decode_flight_time(
            timestamp_raw, prev_timestamp_raw, timestamp_wrap_count
        )
        prev_timestamp_raw = timestamp_raw

        row: dict[str, object] = {
            "flight_index": flight_index if flight_index >= 0 else "",
            "offset": offset,
            "page_index": offset // PAGE_SIZE,
            "page_offset": page_offset,
            "record_type": record_def.type_name,
            "record_size": record_def.size,
            "timestamp_raw_ms": timestamp_raw,
            "flight_time_ms": flight_time_ms,
            "epoch_uptime_ms": (
                flight_epoch_uptime_ms + flight_time_ms
                if flight_epoch_uptime_ms is not None
                else ""
            ),
        }

        if raw_type == 0x01:
            (
                flight_id,
                flight_epoch_uptime_ms,
                imu_rate_hz,
                baro_rate_hz,
                format_version,
                _reserved,
            ) = fields
            if format_version != FORMAT_VERSION:
                raise ParseError(
                    f"unsupported format version {format_version} at offset 0x{offset:08X}"
                )
            row.update(
                {
                    "flight_id": flight_id,
                    "imu_rate_hz": imu_rate_hz,
                    "baro_rate_hz": baro_rate_hz,
                    "format_version": format_version,
                    "epoch_uptime_ms": flight_epoch_uptime_ms,
                }
            )
        elif raw_type == 0x02:
            accel_x_mg, accel_y_mg, accel_z_mg, gyro_x_ddps, gyro_y_ddps, gyro_z_ddps = fields
            row.update(
                {
                    "accel_x_mg": accel_x_mg,
                    "accel_y_mg": accel_y_mg,
                    "accel_z_mg": accel_z_mg,
                    "accel_x_g": accel_x_mg / 1000.0,
                    "accel_y_g": accel_y_mg / 1000.0,
                    "accel_z_g": accel_z_mg / 1000.0,
                    "gyro_x_ddps": gyro_x_ddps,
                    "gyro_y_ddps": gyro_y_ddps,
                    "gyro_z_ddps": gyro_z_ddps,
                    "gyro_x_dps": gyro_x_ddps / 10.0,
                    "gyro_y_dps": gyro_y_ddps / 10.0,
                    "gyro_z_dps": gyro_z_ddps / 10.0,
                }
            )
        elif raw_type == 0x03:
            pressure_pa, temp_cdeg, _reserved = fields
            row.update(
                {
                    "pressure_pa": pressure_pa,
                    "pressure_kpa": pressure_pa / 1000.0,
                    "temp_cdeg": temp_cdeg,
                    "temp_c": temp_cdeg / 100.0,
                }
            )
        elif raw_type == 0x04:
            prev_state, new_state, _reserved = fields
            row.update(
                {
                    "prev_state": prev_state,
                    "prev_state_name": STATE_NAMES.get(prev_state, "UNKNOWN"),
                    "new_state": new_state,
                    "new_state_name": STATE_NAMES.get(new_state, "UNKNOWN"),
                }
            )
        elif raw_type == 0x05:
            channel, action, ilm_raw = fields
            row.update(
                {
                    "pyro_channel": channel,
                    "pyro_action": action,
                    "pyro_action_name": PYRO_ACTION_NAMES.get(action, "UNKNOWN"),
                    "ilm_raw": ilm_raw,
                }
            )
        elif raw_type == 0x06:
            vbat_raw, vcc_raw, pyro1_ilm_raw, pyro2_ilm_raw = fields
            row.update(
                {
                    "vbat_raw": vbat_raw,
                    "vcc_raw": vcc_raw,
                    "pyro1_ilm_raw": pyro1_ilm_raw,
                    "pyro2_ilm_raw": pyro2_ilm_raw,
                }
            )
        elif raw_type == 0x07:
            (timestamp_sync_epoch_uptime_ms,) = fields
            row.update(
                {
                    "timestamp_sync_epoch_uptime_ms": timestamp_sync_epoch_uptime_ms,
                    "epoch_uptime_ms": timestamp_sync_epoch_uptime_ms,
                }
            )
        elif raw_type == 0x08:
            (total_records,) = fields
            row.update({"total_records": total_records})

        rows.append(row)
        offset = record_end

    return rows


def open_output(path: Path | None, bin_path: Path) -> tuple[BinaryIO, str]:
    if path is None:
        path = bin_path.with_suffix(".csv")

    if str(path) == "-":
        return sys.stdout, "stdout"

    handle = path.open("w", newline="", encoding="utf-8")
    return handle, str(path)


def write_csv(rows: list[dict[str, object]], output_path: Path | None, bin_path: Path) -> str:
    handle, destination = open_output(output_path, bin_path)
    close_handle = handle is not sys.stdout

    try:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    finally:
        if close_handle:
            handle.close()

    return destination


def main() -> int:
    args = parse_args()

    if not args.bin_path.is_file():
        print(f"error: input file not found: {args.bin_path}", file=sys.stderr)
        return 1

    try:
        rows = parse_records(args.bin_path.read_bytes())
        destination = write_csv(rows, args.output, args.bin_path)
    except ParseError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except OSError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(f"wrote {len(rows)} records to {destination}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
