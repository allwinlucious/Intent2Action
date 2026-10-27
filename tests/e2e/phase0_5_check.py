"""Phase 0.5 check: ROS 2 between Isaac (Fast DDS, Windows) and the container.

Runs sim/ros_probe.py in Isaac Sim's Python and tests/e2e/ros_probe_peer.py in
the intrinsic-core:dev image, once with the peer on CycloneDDS and once on
Fast DDS, then checks the Hyper-V firewall:
  .venv/bin/python tests/e2e/phase0_5_check.py
Needs no running container; the peer runs as container `probe`.
"""

import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import threading
import time

import numpy as np

_ROOT = Path("/home/mew/untitled-project")
ISAAC_WORK_DIR = Path("/mnt/c/isaac-sim-work")
RESULT_PATH = ISAAC_WORK_DIR / "probe_result.json"
FASTDDS_XML = _ROOT / "container/dds/fastdds.xml"
ISAAC_SCRIPT = _ROOT / "sim/ros_probe.py"
LOG_DIR = Path("/tmp")
RUNS = (("rmw_cyclonedds_cpp", "cyclone"), ("rmw_fastrtps_cpp", "fastdds"))
FIRST_PING_TIMEOUT_S = 180
NO_7400_DELAY_S = 10
ISAAC_TIMEOUT_S = 300
PEER_TIMEOUT_S = 60
MIN_RECEIVED_RATIO = 0.99
SERVICE_CALLS = 10
DISCOVERY_PORTS = (7400, 7401)
WSL_FIREWALL_ID = "{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}"
PEER_COMMAND = (
    "docker run --rm --name probe --network host"
    " -e RMW_IMPLEMENTATION={rmw}"
    " -e CYCLONEDDS_URI=file:///opt/app/container/dds/cyclonedds.xml"
    " -e FASTRTPS_DEFAULT_PROFILES_FILE=/opt/app/container/dds/fastdds.xml"
    " -v /home/mew/untitled-project/tests/e2e:/probe:ro"
    " --entrypoint bash intrinsic-core:dev"
    " -c 'source /opt/ros/humble/setup.bash && python3 /probe/ros_probe_peer.py'"
)


class _Process:
  """A subprocess whose merged output is collected line by line by a thread."""

  def __init__(self, args, label: str, echo, stdin=subprocess.DEVNULL):
    self.lines = []
    self.process = subprocess.Popen(
        args,
        stdin=stdin,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        errors="replace",
    )
    self._label = label
    self._echo = echo
    self._thread = threading.Thread(target=self._read, daemon=True)
    self._thread.start()

  def _read(self) -> None:
    for line in self.process.stdout:
      line = line.rstrip()
      self.lines.append(line)
      if self._echo(line):
        print(f"  {self._label}: {line}", flush=True)

  def wait_for_line(self, prefix: str, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
      if any(line.startswith(prefix) for line in self.lines):
        return True
      time.sleep(0.1)
    return False

  def wait(self, timeout_s: float) -> bool:
    try:
      self.process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
      return False
    self._thread.join(timeout=5)
    return True


def _docker(command: str) -> list[str]:
  return ["sg", "docker", "-c", command]


def _bound_ports() -> list[str]:
  """Local UDP addresses on 7400/7401, on Windows and in WSL."""
  found = []
  windows = subprocess.run(
      ["netstat.exe", "-ano", "-p", "udp"],
      capture_output=True, text=True, errors="replace", check=True,
  ).stdout
  for line in windows.splitlines():
    fields = line.split()
    if len(fields) >= 2 and fields[0] == "UDP":
      found += [f"windows {fields[1]}" for port in DISCOVERY_PORTS
                if fields[1].endswith(f":{port}")]
  wsl = subprocess.run(
      ["ss", "-uln"], capture_output=True, text=True, check=True
  ).stdout
  for line in wsl.splitlines()[1:]:
    fields = line.split()
    if len(fields) >= 4:
      found += [f"wsl {fields[3]}" for port in DISCOVERY_PORTS
                if fields[3].endswith(f":{port}")]
  return found


def _run_probe(rmw: str, label: str) -> list[tuple[str, bool, bool, str]]:
  """One run; returns (name, required, passed, detail) per check."""
  shutil.copy(FASTDDS_XML, ISAAC_WORK_DIR / "fastdds.xml")
  RESULT_PATH.unlink(missing_ok=True)
  subprocess.run(_docker("docker rm -f probe"), capture_output=True)

  print(f"run {label} ({rmw})", flush=True)
  peer = _Process(
      _docker(PEER_COMMAND.format(rmw=rmw)), "peer", echo=lambda line: True
  )
  with open(ISAAC_SCRIPT) as script:
    isaac = _Process(
        ["cmd.exe", "/c", r"cd /d C:\isaac-sim && python.bat -"],
        "isaac",
        echo=lambda line: line.startswith(("PING_START", "PROBE_DONE", "NO_MATCH")),
        stdin=script,
    )

  results = []
  discovered = peer.wait_for_line("FIRST_PING", FIRST_PING_TIMEOUT_S)
  if discovered:
    time.sleep(NO_7400_DELAY_S)
    bound = _bound_ports()
    results.append(
        (f"no_7400_{label}", True, not bound,
         ", ".join(bound) if bound else "no socket on 7400/7401")
    )
  else:
    results.append((f"no_7400_{label}", True, False, "not run: no FIRST_PING"))

  if not isaac.wait(ISAAC_TIMEOUT_S):
    isaac.process.terminate()
    isaac.wait(10)
    print(f"  isaac: terminated after {ISAAC_TIMEOUT_S} s", flush=True)
  if not peer.wait(PEER_TIMEOUT_S):
    subprocess.run(_docker("docker rm -f probe"), capture_output=True)
    peer.wait(10)
    print(f"  peer: removed after {PEER_TIMEOUT_S} s", flush=True)
  (LOG_DIR / f"phase0_5_isaac_{label}.log").write_text("\n".join(isaac.lines))
  (LOG_DIR / f"phase0_5_peer_{label}.log").write_text("\n".join(peer.lines))

  service_required = label == "fastdds"
  if not discovered:
    results += [
        (f"topic_{label}", True, False, "no discovery"),
        (f"service_{label}", service_required, False, "no discovery"),
        (f"latency_{label}", False, False, "no discovery"),
    ]
    return results

  if RESULT_PATH.exists():
    result = json.loads(RESULT_PATH.read_text())
    sent, received, rtt_ms = result["sent"], result["received"], result["rtt_ms"]
    ratio = received / sent if sent else 0.0
    results.append(
        (f"topic_{label}", True, ratio >= MIN_RECEIVED_RATIO,
         f"{received}/{sent} ({ratio:.2%})")
    )
    if rtt_ms:
      p50, p99 = np.percentile(rtt_ms, [50, 99])
      results.append(
          (f"latency_{label}", False, True,
           f"p50 {p50:.3f} p99 {p99:.3f} max {max(rtt_ms):.3f} ms")
      )
    else:
      results.append((f"latency_{label}", False, False, "no round trips"))
  else:
    results += [
        (f"topic_{label}", True, False, f"{RESULT_PATH} missing"),
        (f"latency_{label}", False, False, f"{RESULT_PATH} missing"),
    ]

  ok = 0
  for line in peer.lines:
    if match := re.match(r"SERVICE (\d+)/\d+", line):
      ok = int(match.group(1))
  results.append(
      (f"service_{label}", service_required, ok == SERVICE_CALLS,
       f"{ok}/{SERVICE_CALLS}")
  )
  return results


def _check_firewall() -> tuple[str, bool, bool, str]:
  output = subprocess.run(
      ["powershell.exe", "-NoProfile", "-Command",
       "Get-NetFirewallHyperVVMSetting -PolicyStore ActiveStore"
       f" -Name '{WSL_FIREWALL_ID}'"],
      capture_output=True, text=True, errors="replace",
  ).stdout
  match = re.search(r"DefaultInboundAction\s*:\s*(\S+)", output)
  action = match.group(1) if match else "not found"
  return ("firewall", True, action == "Block", f"DefaultInboundAction {action}")


def main() -> int:
  start = time.monotonic()
  results = []
  for rmw, label in RUNS:
    results += _run_probe(rmw, label)
  results.append(_check_firewall())
  for name, required, passed, detail in results:
    note = "" if required else " (recorded)"
    print(f"{'PASS' if passed else 'FAIL'} {name}{note}: {detail}", flush=True)
  print(f"elapsed {time.monotonic() - start:.1f}")
  return 0 if all(passed for _, required, passed, _ in results if required) else 1


if __name__ == "__main__":
  sys.exit(main())
