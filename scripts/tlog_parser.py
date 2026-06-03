"""
tlog_parser.py — Parse MAVLink telemetry logs and export to CSV.

Usage:
  python scripts/tlog_parser.py logs/mav.tlog
  python scripts/tlog_parser.py logs/mav.tlog --csv output.csv
  python scripts/tlog_parser.py logs/mav.tlog --summary
"""

import argparse
import csv
import sys
from pathlib import Path
from collections import defaultdict


def parse_tlog(path: str) -> list[dict]:
    from pymavlink import mavutil

    mlog = mavutil.mavlink_connection(path)
    rows = []

    while True:
        msg = mlog.recv_match(blocking=False)
        if msg is None:
            break
        row = {"type": msg.get_type(), "timestamp": getattr(msg, "_timestamp", 0)}
        for field in msg.__dict__:
            if not field.startswith("_"):
                row[field] = getattr(msg, field)
        rows.append(row)

    return rows


def print_summary(rows: list[dict]):
    types = defaultdict(int)
    for r in rows:
        types[r["type"]] += 1

    print(f"Total messages: {len(rows)}")
    print(f"Message types:  {len(types)}")
    print()
    print(f"{'Type':<25} {'Count':>8}")
    print("-" * 35)
    for t, c in sorted(types.items(), key=lambda x: -x[1]):
        print(f"{t:<25} {c:>8}")


def export_csv(rows: list[dict], output: str):
    if not rows:
        print("No data to export")
        return

    keys = list(rows[0].keys())
    with open(output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)

    print(f"Exported {len(rows)} rows to {output}")


def main():
    parser = argparse.ArgumentParser(description="Parse MAVLink .tlog files")
    parser.add_argument("path", help="Path to .tlog file")
    parser.add_argument("--csv", help="Export to CSV file")
    parser.add_argument(
        "--summary", action="store_true", default=True, help="Print summary"
    )
    args = parser.parse_args()

    if not Path(args.path).exists():
        print(f"File not found: {args.path}")
        sys.exit(1)

    rows = parse_tlog(args.path)

    if args.summary:
        print_summary(rows)

    if args.csv:
        export_csv(rows, args.csv)


if __name__ == "__main__":
    main()
