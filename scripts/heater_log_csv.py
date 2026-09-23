#!/usr/bin/env python3
"""Extract heater PID telemetry from an `idf.py monitor` capture into CSV.

Capture:
    idf.py -p /dev/cu.usbmodem5ABA0676261 monitor | tee heat.log
Convert:
    scripts/heater_log_csv.py heat.log > heat.csv

Each `heat ph=...` line becomes one row (boot_ms plus every key=value
field). `heat phase` and `heat shot:` lines are echoed to stderr so the
per-shot summaries are visible alongside the conversion.
"""

import csv
import re
import sys

LINE = re.compile(r"I \((\d+)\) \S+: heat ph=(.*)$")
FIELD = re.compile(r"(\w+)=(\S+)")
EVENT = re.compile(r"heat (phase|shot:)")
ANSI = re.compile(r"\x1b\[[0-9;]*m")


def main() -> int:
    src = open(sys.argv[1], errors="replace") if len(sys.argv) > 1 else sys.stdin
    writer = None
    for raw in src:
        line = ANSI.sub("", raw).rstrip()
        m = LINE.search(line)
        if m:
            row = {"boot_ms": m.group(1), "ph": m.group(2).split()[0]}
            row.update(FIELD.findall(m.group(2)))
            if writer is None:
                writer = csv.DictWriter(sys.stdout, fieldnames=list(row))
                writer.writeheader()
            writer.writerow({k: row.get(k, "") for k in writer.fieldnames})
        elif EVENT.search(line):
            print(line, file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
