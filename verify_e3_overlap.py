#!/usr/bin/env python3
"""Corroborate E3 concurrent ROS1/ROS2 navigation with Core/bridge evidence.

Run on the Ubuntu host AFTER create3_physical_e2e.py --parallel-goals-only
or --execute --validate-parallel-goals. A successful client run alone cannot
prove that both ROS action servers were active at the same time.

  python3 verify_e3_overlap.py --client-run /path/to/client-run \
      --core-evidence /path/to/core_bridge_events.jsonl

This verifier makes no ROS/IS-Wire calls and cannot move a robot. It saves
e3_overlap_verification.json in the corresponding client evidence directory.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys


class VerificationError(ValueError):
    pass


def jsonl(path: Path) -> list[dict]:
    records = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                item = json.loads(line)
            except json.JSONDecodeError as exc:
                raise VerificationError(f"{path}:{line_number} invalid JSON") from exc
            if not isinstance(item, dict):
                raise VerificationError(f"{path}:{line_number} is not an object")
            records.append(item)
    return records


def unique_event(events: list[dict], request_id: str, component: str,
                 event: str, robot_id: str) -> dict:
    matches = [r for r in events if r.get("request_id") == request_id
               and r.get("component") == component and r.get("event") == event
               and r.get("robot_id") == robot_id]
    if len(matches) != 1:
        raise VerificationError(f"expected one {component}.{event} for {robot_id} "
                                f"request_id={request_id}; found {len(matches)}")
    return matches[0]


def verify(client_run: Path, core_evidence: Path) -> dict:
    summary = json.loads((client_run / "summary.json").read_text(encoding="utf-8"))
    cycles = summary.get("parallel_distinct_cycles")
    if cycles is not None:
        if (not isinstance(cycles, list) or len(cycles) != 5
                or summary.get("parallel_completed_loops") != 5):
            raise VerificationError("E3 must contain five fully completed cycles")
        pairs = []
        for index, cycle in enumerate(cycles, 1):
            if cycle.get("loop") != index:
                raise VerificationError("E3 cycles are missing or out of order")
            for phase in ("target", "return"):
                pair = cycle.get(phase)
                if not isinstance(pair, dict):
                    raise VerificationError(f"E3 loop {index} has no {phase} pair")
                pairs.append((index, phase, pair))
    elif isinstance(summary.get("parallel_distinct_goals"), dict):
        # Historical E3 runs dispatched one pair. Keep them verifiable without
        # treating them as five-cycle runs.
        pairs = [(1, "target", summary["parallel_distinct_goals"])]
    else:
        raise VerificationError("client summary does not contain an E3 run")
    client_records = jsonl(client_run / "wire_requests.jsonl")
    server_records = jsonl(core_evidence)
    verified_pairs = []
    seen_ids = set()
    last_terminal = {}
    for index, phase, runs in pairs:
        proof = {}
        if not isinstance(runs.get("client_rpc_overlap_ms"), (int, float)) or (
                runs["client_rpc_overlap_ms"] <= 0):
            raise VerificationError(f"E3 loop {index} {phase} client RPCs did not overlap")
        for version, component, status in (("ros1", "ros1_adapter", 3),
                                           ("ros2", "ros2_adapter", 4)):
            item = runs.get(version)
            if not isinstance(item, dict) or not item.get("request_id") or not item.get("robot_id"):
                raise VerificationError(f"missing E3 {version} request ID or robot ID")
            rid, req_id = item["robot_id"], item["request_id"]
            if req_id in seen_ids:
                raise VerificationError(f"E3 reused request ID {req_id}")
            seen_ids.add(req_id)
            if item.get("terminal_status") != status:
                raise VerificationError(f"{version} client did not report a successful navigation")
            matches = [r for r in client_records
                       if r.get("request", {}).get("request_id") == req_id]
            if len(matches) != 1:
                raise VerificationError(f"expected one client RPC for E3 {version}; found {len(matches)}")
            client_rpc = matches[0]
            if (client_rpc.get("status") != "passed"
                    or client_rpc.get("request", {}).get("robot_id") != rid
                    or client_rpc["request"].get("operation") != "navigate"
                    or client_rpc.get("reply", {}).get("request_id") != req_id
                    or client_rpc["reply"].get("ok") is not True):
                raise VerificationError(f"{version} client RPC is not independently successful/correlated")
            sent = unique_event(server_records, req_id, component, "goal_sent", rid)
            terminal = unique_event(server_records, req_id, component, "goal_terminal", rid)
            received = unique_event(server_records, req_id, "iswire", "request_received", rid)
            prepared = unique_event(server_records, req_id, "core", "navigation_goal_prepared", rid)
            dispatched = unique_event(server_records, req_id, "bridge", "action_dispatched", rid)
            replied = unique_event(server_records, req_id, "iswire", "reply_prepared", rid)
            if (received.get("operation") != "navigate"
                    or received.get("correlation_id") != client_rpc.get("correlation_id")
                    or received.get("navigation_args") != client_rpc["request"].get("args")
                    or replied.get("ok") is not True):
                raise VerificationError(f"{version} Core request/correlation/payload mismatch")
            if (prepared.get("ros_version") != (1 if version == "ros1" else 2)
                    or dispatched.get("ros_version") != (1 if version == "ros1" else 2)):
                raise VerificationError(f"{version} dispatched through the wrong ROS adapter")
            if terminal.get("terminal_status") != status:
                raise VerificationError(f"{version} ROS action did not terminate successfully")
            if (sent.get("goal_id") and terminal.get("goal_id")
                    and sent["goal_id"] != terminal["goal_id"]):
                raise VerificationError(f"{version} Core goal IDs do not match")
            if (item.get("goal_id") and sent.get("goal_id")
                    and item["goal_id"] != sent["goal_id"]):
                raise VerificationError(f"{version} client/Core goal IDs do not match")
            start_ns, finish_ns = sent.get("monotonic_ns"), terminal.get("monotonic_ns")
            if (not isinstance(start_ns, int) or not isinstance(finish_ns, int)
                    or start_ns >= finish_ns):
                raise VerificationError(f"{version} invalid Core monotonic goal interval")
            if version in last_terminal and start_ns <= last_terminal[version]:
                raise VerificationError(f"E3 {version} did not finish the previous leg before the next")
            last_terminal[version] = finish_ns
            proof[version] = {"robot_id": rid, "request_id": req_id,
                              "goal_id": sent.get("goal_id"),
                              "goal_sent_monotonic_ns": start_ns,
                              "goal_terminal_monotonic_ns": finish_ns,
                              "terminal_status": status,
                              "target_world": item.get("target_world"),
                              "frame_id": item.get("frame_id")}

        if proof["ros1"]["robot_id"] == proof["ros2"]["robot_id"]:
            raise VerificationError("ROS1 and ROS2 share a robot ID")
        if phase == "target":
            if (not isinstance(proof["ros1"]["target_world"], list)
                    or not isinstance(proof["ros2"]["target_world"], list)
                    or len(proof["ros1"]["target_world"]) != 2
                    or len(proof["ros2"]["target_world"]) != 2
                    or sum((a-b)**2 for a, b in zip(proof["ros1"]["target_world"],
                                                    proof["ros2"]["target_world"])) < 0.25):
                raise VerificationError(f"E3 loop {index} did not dispatch distinct destinations")
        elif (proof["ros1"]["target_world"] != [0.0, 0.0]
              or proof["ros2"]["target_world"] != [0.0, 0.0]):
            raise VerificationError(f"E3 loop {index} did not return both robots to origin")

        overlap_ns = min(proof["ros1"]["goal_terminal_monotonic_ns"],
                         proof["ros2"]["goal_terminal_monotonic_ns"]) - max(
                             proof["ros1"]["goal_sent_monotonic_ns"],
                             proof["ros2"]["goal_sent_monotonic_ns"])
        if overlap_ns <= 0:
            raise VerificationError(f"Core recorded no overlap between ROS1/ROS2 "
                                    f"action intervals in E3 loop {index} {phase}")
        verified_pairs.append({"loop": index, "phase": phase,
                               "core_overlap_ms": round(overlap_ns / 1_000_000, 3),
                               "client_rpc_overlap_ms": runs["client_rpc_overlap_ms"],
                               "goals": proof})

    minimum_overlap = min(pair["core_overlap_ms"] for pair in verified_pairs)
    return {"status": "PASS", "criterion": "server_side_action_interval_overlap",
            "method": "Core monotonic_ns interval intersection, goal_sent to goal_terminal",
            "core_overlap_ms": minimum_overlap,
            "pairs_verified": len(verified_pairs),
            "cycles_verified": len(cycles) if cycles is not None else None,
            "pairs": verified_pairs,
            "verified_at_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "client_run_id": summary.get("run_id"),
            "core_evidence_file": str(core_evidence),
            "goals": verified_pairs[0]["goals"] if cycles is None else None,
            "limits": ["Instrumented Core timing, not independently measured robot movement",
                       "The goal intervals are action-client acceptance-to-terminal windows",
                       "No claim of coordinate accuracy or physical trajectory overlap"]}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--client-run", type=Path, required=True)
    parser.add_argument("--core-evidence", type=Path, required=True)
    args = parser.parse_args(argv)
    target = args.client_run.expanduser().resolve() / "e3_overlap_verification.json"
    try:
        result = verify(args.client_run.expanduser().resolve(),
                        args.core_evidence.expanduser().resolve())
    except (OSError, ValueError, TypeError, KeyError) as exc:
        result = {"status": "FAIL", "criterion": "server_side_action_interval_overlap",
                  "error": f"{type(exc).__name__}: {exc}"}
    target.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    if result["status"] == "PASS":
        print(f"[PASS] E3 Core ROS1/ROS2 goal overlap: "
              f"{result['pairs_verified']} pairs verified "
              f"(minimum={result['core_overlap_ms']} ms)")
    else:
        print(f"[FAIL] E3 overlap not verified: {result['error']}", file=sys.stderr)
    print(f"Evidence: {target}")
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
