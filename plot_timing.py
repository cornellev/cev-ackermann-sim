#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_rows(path: Path):
    with path.open(newline="") as timing_file:
        rows = list(csv.DictReader(timing_file))
    if not rows:
        raise ValueError(f"No timing rows found in {path}")
    return rows


def moving_average(values, window):
    averages = []
    for index in range(len(values)):
        start = max(0, index - window + 1)
        sample = values[start : index + 1]
        averages.append(sum(sample) / len(sample))
    return averages


def make_plot(input_path: Path, output_path: Path, moving_window: int):
    rows = read_rows(input_path)
    search_index = list(range(1, len(rows) + 1))
    durations = [float(row["astar_ms"]) for row in rows]
    path_lengths = [int(row["path_steps"]) for row in rows]
    successful = [row["found"].lower() == "true" for row in rows]
    success_times = [duration if found else None for duration, found in zip(durations, successful)]
    failure_times = [duration if not found else None for duration, found in zip(durations, successful)]

    figure, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    figure.suptitle("A* Timing and Search Results")

    timing_axis = axes[0]
    timing_axis.plot(search_index, durations, color="#2563eb", linewidth=1, alpha=0.45, label="A* time")
    timing_axis.plot(
        search_index,
        moving_average(durations, moving_window),
        color="#dc2626",
        linewidth=2,
        label=f"{moving_window}-sample mean",
    )
    timing_axis.scatter(search_index, success_times, color="#16a34a", s=12, label="success")
    timing_axis.scatter(search_index, failure_times, color="#dc2626", s=28, marker="x", label="failure")
    timing_axis.set_ylabel("A* time (ms)")
    timing_axis.grid(True, alpha=0.25)
    timing_axis.legend(loc="upper right")

    path_axis = axes[1]
    path_axis.plot(search_index, path_lengths, color="#7c3aed", linewidth=1.5)
    path_axis.set_xlabel("Search number")
    path_axis.set_ylabel("Path steps")
    path_axis.grid(True, alpha=0.25)

    figure.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=160)
    print(f"Wrote {output_path} from {len(rows)} searches")


def main():
    parser = argparse.ArgumentParser(description="Plot astar_timing.csv")
    parser.add_argument("input", nargs="?", type=Path, default=Path("astar_timing.csv"))
    parser.add_argument("-o", "--output", type=Path, default=Path("astar_timing.png"))
    parser.add_argument("--moving-window", type=int, default=25)
    args = parser.parse_args()
    if args.moving_window < 1:
        parser.error("--moving-window must be at least 1")
    make_plot(args.input, args.output, args.moving_window)


if __name__ == "__main__":
    main()