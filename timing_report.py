#!/usr/bin/env python3

import csv
import statistics
import sys


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "astar_timing.csv"
    with open(path, newline="") as timing_file:
        rows = list(csv.DictReader(timing_file))
    if not rows:
        print("No timing rows found.")
        return
    durations = [float(row["astar_ms"]) for row in rows]
    successes = sum(row["found"] == "true" for row in rows)
    print("metric              value")
    print(f"searches            {len(rows)}")
    print(f"successful          {successes}")
    print(f"failed              {len(rows) - successes}")
    print(f"mean_astar_ms       {statistics.mean(durations):.3f}")
    print(f"median_astar_ms     {statistics.median(durations):.3f}")
    print(f"p95_astar_ms        {statistics.quantiles(durations, n=20, method='inclusive')[18]:.3f}")
    print(f"max_astar_ms        {max(durations):.3f}")


if __name__ == "__main__":
    main()