"""Copies stdin to stdout as "[<name>] <line>" and appends each line to
/var/lib/intrinsic/logs/<name>.log, rotated at 10 MB x 3 (phase_3.md, decision 28).
File lines start with an ISO-8601 time; one is added if the line has none.

Usage: <command> 2>&1 | python3 logpipe.py <name>
"""
import datetime
import logging
import logging.handlers
import os
import re
import sys

LOG_DIR = "/var/lib/intrinsic/logs"
MAX_BYTES = 10 * 1024 * 1024
BACKUPS = 3
ISO_TIME = re.compile(r"\d{4}-\d{2}-\d{2}[T ]\d{2}:\d{2}:\d{2}")


def with_time(line: str) -> str:
  if ISO_TIME.match(line):
    return line
  return f"{datetime.datetime.now().isoformat(sep=' ', timespec='milliseconds')} {line}"


def main() -> None:
  name = sys.argv[1]
  os.makedirs(LOG_DIR, exist_ok=True)
  handler = logging.handlers.RotatingFileHandler(
      os.path.join(LOG_DIR, f"{name}.log"), maxBytes=MAX_BYTES, backupCount=BACKUPS)
  handler.setFormatter(logging.Formatter("%(message)s"))
  log = logging.getLogger(name)
  log.addHandler(handler)
  log.setLevel(logging.INFO)
  log.propagate = False
  sys.stdin.reconfigure(errors="replace")
  for line in sys.stdin:
    line = line.rstrip("\n")
    sys.stdout.write(f"[{name}] {line}\n")
    sys.stdout.flush()
    log.info(with_time(line))


if __name__ == "__main__":
  main()
