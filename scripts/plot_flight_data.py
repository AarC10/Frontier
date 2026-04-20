#!/usr/bin/env python3

"""Plot Frontier flight log data from a .BIN or parsed .csv file."""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import tempfile
from pathlib import Path

if "MPLCONFIGDIR" not in os.environ:
    os.environ["MPLCONFIGDIR"] = str(Path(tempfile.gettempdir()) / "frontier-mpl-cache")

import matplotlib.pyplot as plt

from parse_flight_bin import ParseError, parse_records


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot Frontier flight log data from a .BIN or parsed .csv file."
    )
    parser.add_argument("input_path", type=Path, help="Input .BIN or .csv file")
    parser.add_argument(
        "--flight-index",
        type=int,
        default=0,
        help="Flight index to plot when the input contains multiple flights (default: 0)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Save the plot to a file instead of opening an interactive window.",
    )
    parser.add_argument(
        "--title",
        help="Optional custom plot title.",
    )
    return parser.parse_args()


def load_rows(path: Path) -> list[dict[str, object]]:
    suffix = path.suffix.lower()
    if suffix == ".bin":
        return parse_records(path.read_bytes())
    if suffix == ".csv":
        with path.open("r", newline="", encoding="utf-8") as handle:
            return list(csv.DictReader(handle))
    raise ValueError(f"unsupported input type: {path.suffix}")


def as_float(row: dict[str, object], key: str) -> float | None:
    value = row.get(key, "")
    if value in ("", None):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def as_int(row: dict[str, object], key: str) -> int | None:
    value = row.get(key, "")
    if value in ("", None):
        return None
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return None


def pressure_kpa_to_altitude_m(pressure_kpa: float) -> float:
    return 44330.0 * (1.0 - math.pow(pressure_kpa / 101.325, 1.0 / 5.255))


def select_flight(rows: list[dict[str, object]], flight_index: int) -> list[dict[str, object]]:
    matching_rows: list[dict[str, object]] = []
    for row in rows:
        row_flight_index = as_int(row, "flight_index")
        if row_flight_index == flight_index:
            matching_rows.append(row)
    return matching_rows


def plot_series_or_note(ax: plt.Axes, has_series: bool, note: str) -> None:
    if not has_series:
        ax.text(
            0.5,
            0.5,
            note,
            transform=ax.transAxes,
            ha="center",
            va="center",
            fontsize=10,
            color="0.4",
        )


def add_event_markers(
    axes: list[plt.Axes],
    flight_rows: list[dict[str, object]],
) -> None:
    state_labeled = False
    pyro_labeled = False

    for row in flight_rows:
        time_s = as_float(row, "flight_time_ms")
        if time_s is None:
            continue
        time_s /= 1000.0

        record_type = row.get("record_type")
        if record_type == "STATE_CHANGE":
            label = row.get("new_state_name", "state")
            for ax in axes:
                ax.axvline(
                    time_s,
                    color="0.55",
                    linestyle="--",
                    linewidth=0.9,
                    alpha=0.8,
                    label="State change" if not state_labeled else None,
                )
            axes[0].annotate(
                str(label),
                xy=(time_s, 1.0),
                xycoords=("data", "axes fraction"),
                xytext=(2, -16),
                textcoords="offset points",
                rotation=90,
                va="top",
                fontsize=8,
                color="0.35",
            )
            state_labeled = True
        elif record_type == "PYRO_EVENT":
            action_name = row.get("pyro_action_name", "PYRO")
            channel = row.get("pyro_channel", "")
            for ax in axes:
                ax.axvline(
                    time_s,
                    color="#d55e00",
                    linestyle=":",
                    linewidth=1.0,
                    alpha=0.85,
                    label="Pyro event" if not pyro_labeled else None,
                )
            axes[3].annotate(
                f"CH{channel} {action_name}",
                xy=(time_s, 1.0),
                xycoords=("data", "axes fraction"),
                xytext=(2, -16),
                textcoords="offset points",
                rotation=90,
                va="top",
                fontsize=8,
                color="#8c3c00",
            )
            pyro_labeled = True


def main() -> int:
    args = parse_args()

    if not args.input_path.is_file():
        print(f"error: input file not found: {args.input_path}", file=sys.stderr)
        return 1

    try:
        rows = load_rows(args.input_path)
    except (OSError, ValueError, ParseError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    if not rows:
        print("error: input file produced no rows", file=sys.stderr)
        return 1

    flight_rows = select_flight(rows, args.flight_index)
    if not flight_rows:
        print(
            f"error: no rows found for flight index {args.flight_index}",
            file=sys.stderr,
        )
        return 1

    flight_header = next(
        (row for row in flight_rows if row.get("record_type") == "FLIGHT_HEADER"),
        {},
    )
    flight_id = as_int(flight_header, "flight_id")

    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True, constrained_layout=True)
    title = args.title or f"Flight {flight_id if flight_id is not None else args.flight_index}"
    fig.suptitle(title)

    baro_times_s: list[float] = []
    baro_altitudes_m: list[float] = []
    baseline_altitude_m: float | None = None

    imu_times_s: list[float] = []
    accel_x_g: list[float] = []
    accel_y_g: list[float] = []
    accel_z_g: list[float] = []
    accel_mag_g: list[float] = []

    gyro_times_s: list[float] = []
    gyro_x_dps: list[float] = []
    gyro_y_dps: list[float] = []
    gyro_z_dps: list[float] = []

    voltage_times_s: list[float] = []
    vbat_raw: list[float] = []
    vcc_raw: list[float] = []
    pyro1_raw: list[float] = []
    pyro2_raw: list[float] = []

    for row in flight_rows:
        time_ms = as_float(row, "flight_time_ms")
        if time_ms is None:
            continue
        time_s = time_ms / 1000.0
        record_type = row.get("record_type")

        if record_type == "BARO":
            pressure_kpa = as_float(row, "pressure_kpa")
            if pressure_kpa is None:
                continue
            altitude_m = pressure_kpa_to_altitude_m(pressure_kpa)
            if baseline_altitude_m is None:
                baseline_altitude_m = altitude_m
            baro_times_s.append(time_s)
            baro_altitudes_m.append(altitude_m - baseline_altitude_m)
        elif record_type == "IMU":
            x_g = as_float(row, "accel_x_g")
            y_g = as_float(row, "accel_y_g")
            z_g = as_float(row, "accel_z_g")
            gx = as_float(row, "gyro_x_dps")
            gy = as_float(row, "gyro_y_dps")
            gz = as_float(row, "gyro_z_dps")
            if None in (x_g, y_g, z_g, gx, gy, gz):
                continue
            imu_times_s.append(time_s)
            accel_x_g.append(x_g)
            accel_y_g.append(y_g)
            accel_z_g.append(z_g)
            accel_mag_g.append(math.sqrt(x_g * x_g + y_g * y_g + z_g * z_g))
            gyro_times_s.append(time_s)
            gyro_x_dps.append(gx)
            gyro_y_dps.append(gy)
            gyro_z_dps.append(gz)
        elif record_type == "VOLTAGE":
            battery = as_float(row, "vbat_raw")
            rail = as_float(row, "vcc_raw")
            pyro1 = as_float(row, "pyro1_ilm_raw")
            pyro2 = as_float(row, "pyro2_ilm_raw")
            if None in (battery, rail, pyro1, pyro2):
                continue
            voltage_times_s.append(time_s)
            vbat_raw.append(battery)
            vcc_raw.append(rail)
            pyro1_raw.append(pyro1)
            pyro2_raw.append(pyro2)

    axes[0].plot(baro_times_s, baro_altitudes_m, color="#0072b2", linewidth=1.8, label="Relative altitude")
    axes[0].set_ylabel("Altitude (m)")
    axes[0].grid(True, alpha=0.25)
    plot_series_or_note(axes[0], bool(baro_times_s), "No BARO records")

    axes[1].plot(imu_times_s, accel_x_g, color="#d55e00", linewidth=1.0, label="Accel X")
    axes[1].plot(imu_times_s, accel_y_g, color="#009e73", linewidth=1.0, label="Accel Y")
    axes[1].plot(imu_times_s, accel_z_g, color="#0072b2", linewidth=1.0, label="Accel Z")
    axes[1].plot(imu_times_s, accel_mag_g, color="0.2", linewidth=1.4, label="Accel |mag|")
    axes[1].set_ylabel("Accel (g)")
    axes[1].grid(True, alpha=0.25)
    plot_series_or_note(axes[1], bool(imu_times_s), "No IMU records")

    axes[2].plot(gyro_times_s, gyro_x_dps, color="#d55e00", linewidth=1.0, label="Gyro X")
    axes[2].plot(gyro_times_s, gyro_y_dps, color="#009e73", linewidth=1.0, label="Gyro Y")
    axes[2].plot(gyro_times_s, gyro_z_dps, color="#0072b2", linewidth=1.0, label="Gyro Z")
    axes[2].set_ylabel("Gyro (dps)")
    axes[2].grid(True, alpha=0.25)
    plot_series_or_note(axes[2], bool(gyro_times_s), "No IMU records")

    axes[3].plot(voltage_times_s, vbat_raw, color="#cc79a7", linewidth=1.2, label="VBAT raw")
    axes[3].plot(voltage_times_s, vcc_raw, color="#56b4e9", linewidth=1.2, label="VCC raw")
    axes[3].plot(voltage_times_s, pyro1_raw, color="#e69f00", linewidth=1.0, label="Pyro1 ILM raw")
    axes[3].plot(voltage_times_s, pyro2_raw, color="#f0e442", linewidth=1.0, label="Pyro2 ILM raw")
    axes[3].set_ylabel("ADC raw")
    axes[3].set_xlabel("Flight time (s)")
    axes[3].grid(True, alpha=0.25)
    plot_series_or_note(axes[3], bool(voltage_times_s), "No VOLTAGE records")

    add_event_markers(list(axes), flight_rows)

    for ax in axes:
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(loc="upper right", fontsize=8, ncol=min(4, len(handles)))

    if args.output is not None:
        fig.savefig(args.output, dpi=150)
        print(f"wrote plot to {args.output}")
    else:
        plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
