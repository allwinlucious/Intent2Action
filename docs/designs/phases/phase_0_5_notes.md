# Phase 0.5 notes: results

Plan: [phase_0_5.md](phase_0_5.md). Run on 2026-10-27 after rebuilding
`intrinsic-core:dev` with `ros-humble-rmw-cyclonedds-cpp` and
`container/dds/`.

## Result

Every required check passed on the first run. Fallbacks (a) explicit
ports and (b) LAN address were not needed and not applied.

## Full script output

`.venv/bin/python tests/e2e/phase0_5_check.py` (exit code 0):

```
run cyclone (rmw_cyclonedds_cpp)
  isaac: PING_START
  peer: FIRST_PING
  isaac: PROBE_DONE sent 5000 received 5000
run fastdds (rmw_fastrtps_cpp)
  isaac: PING_START
  peer: FIRST_PING
  peer: SERVICE 10/10
  isaac: PROBE_DONE sent 5000 received 5000
PASS no_7400_cyclone: no socket on 7400/7401
PASS topic_cyclone: 5000/5000 (100.00%)
PASS latency_cyclone (recorded): p50 0.539 p99 0.957 max 3.892 ms
FAIL service_cyclone (recorded): 0/10
PASS no_7400_fastdds: no socket on 7400/7401
PASS topic_fastdds: 5000/5000 (100.00%)
PASS latency_fastdds (recorded): p50 0.575 p99 0.999 max 3.563 ms
PASS service_fastdds: 10/10
PASS firewall: DefaultInboundAction Block
elapsed 144.5
```

## Configuration that worked

The plan's base configuration, unchanged: unicast discovery to 127.0.0.1
on both sides, multicast off.

*   `container/dds/fastdds.xml`: Inv §3.3's profile with
    `maxInitialPeersRange` 20 (UDPv4 only, `interfaceWhiteList`
    127.0.0.1, builtin transports off, metatraffic unicast and initial
    peer 127.0.0.1 without ports). Used by Isaac (copied to
    `C:\isaac-sim-work\fastdds.xml`) and by the Fast DDS peer.
*   `container/dds/cyclonedds.xml`: Inv §4.2's configuration with
    `MaxAutoParticipantIndex` 19 (`NetworkInterface address="127.0.0.1"
    multicast="false"`, `AllowMulticast false`, `ParticipantIndex auto`,
    `Peer Address="127.0.0.1"` without a port).
*   Fast DDS 2.6.10 on Windows accepted the profile, and neither side
    bound UDP 7400/7401 (Inv §7's open item on port 7400 is answered: not
    bound with this profile).

## Latency (round trip, Isaac → container → Isaac, 5000 samples at 250 Hz)

| Run (container peer) | p50 | p99 | max |
|---|---|---|---|
| CycloneDDS | 0.539 ms | 0.957 ms | 3.892 ms |
| Fast DDS | 0.575 ms | 0.999 ms | 3.563 ms |

QoS reliable / keep-last 1 / volatile on both topics; no ping was lost in
either run.

## `service_cyclone`: incompatibility confirmed

`FAIL service_cyclone (recorded): 0/10`. Isaac's log
(`/tmp/phase0_5_isaac_cyclone.log`, not kept in the repository) shows one
Fast DDS error per call attempt, about every 5 s (the peer's per-call
timeout), 7 in total:

```
[RTPS_READER_HISTORY Error] Change payload size of '24' bytes is larger than the history payload size of '11' bytes and cannot be resized. -> Function eprosima::fastrtps::rtps::ReaderHistory::can_change_be_added_nts
```

The Cyclone request carries 16 bytes more than Fast DDS expects for an
empty `Trigger` request, matching the request header rmw_cyclonedds
writes into the payload (Inv §5.3). Isaac's server never receives the
request, so a CycloneDDS client cannot call an Isaac (Fast DDS) service.
This confirms the design's "DDS per process" split (Isaac-facing service
calls on Fast DDS).

## Situation the plan does not cover

When calls time out, the peer cannot finish 10 calls before it exits.
The peer exits 10 s after the last ping (about 30 s after the first),
while 10 timed-out calls take 50 s. In the CycloneDDS run the peer made 7
attempts, all failed, and exited before printing `SERVICE n/10`. The check
then used the plan's rule "missing line = 0/10". The recorded result is
still accurate here because every attempt failed. If a future run mixes
successes with timeouts, though, the count would be under-reported. Not
changed, since the plan fixes both the exit rule and the timeout.

## State left behind

The probe container is removed (`docker run --rm`; `docker ps -a` lists
only the pre-existing, stopped `intrinsic` container). No Isaac process is
left running on Windows. `C:\isaac-sim-work\` now also holds
`fastdds.xml` and `probe_result.json` from the last run.

## Resolution (lead)

*   Peer now exits only after all ten service calls finished (success or
    5 s timeout) and 10 s without pings, so `SERVICE n/10` is always
    printed.
*   Result accepted: Q1 answered — CycloneDDS ↔ Fast DDS topics work at
    250 Hz with 100% delivery, round trip p50 0.54 ms, p99 0.96 ms, max
    3.9 ms. Cross-vendor services fail as predicted (Cyclone request is
    16 bytes longer), confirming the design's "DDS per process" split.
*   Control rate: p99 ≈ 1 ms leaves room for 250 Hz lockstep at about
    real-time; phase 2 sets the rate.
