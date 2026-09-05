#!/usr/bin/env python3
"""LAAS Parking Server V1 transport stub.

This intentionally does NOT implement Hybrid A*. It freezes the TCP/NDJSON
contract and lets Pi/server integration be tested before planner integration.
"""

import argparse
import json
import socketserver
import threading
from typing import Any

PROTOCOL_VERSION = 1
MAP_ID = "map_v1"
SLOT_IDS = {"P_B1", "P_B2", "P_T1", "P_T2"}
SLOT_STATES = {"UNKNOWN", "FREE", "OCCUPIED"}

latest_lock = threading.Lock()
latest_parking_status: dict[str, Any] | None = None


def validate_parking_status(msg: dict[str, Any]) -> tuple[bool, str]:
    if msg.get("version") != PROTOCOL_VERSION:
        return False, "unsupported_version"
    if msg.get("map_id") != MAP_ID:
        return False, "map_id_mismatch"
    slots = msg.get("slots")
    if not isinstance(slots, list):
        return False, "slots_not_list"
    seen: set[str] = set()
    for slot in slots:
        if not isinstance(slot, dict):
            return False, "slot_not_object"
        slot_id = slot.get("id")
        state = slot.get("state")
        conf = slot.get("confidence")
        if slot_id not in SLOT_IDS:
            return False, f"invalid_slot:{slot_id}"
        if slot_id in seen:
            return False, f"duplicate_slot:{slot_id}"
        seen.add(slot_id)
        if state not in SLOT_STATES:
            return False, f"invalid_state:{state}"
        if not isinstance(conf, (int, float)) or not 0.0 <= float(conf) <= 1.0:
            return False, f"invalid_confidence:{slot_id}"
    if seen != SLOT_IDS:
        return False, "all_four_slots_required"
    return True, "ok"


class Handler(socketserver.StreamRequestHandler):
    def send_json(self, obj: dict[str, Any]) -> None:
        self.wfile.write((json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8"))
        self.wfile.flush()

    def handle(self) -> None:
        global latest_parking_status
        peer = f"{self.client_address[0]}:{self.client_address[1]}"
        print(f"[SERVER] connected {peer}")
        for raw in self.rfile:
            try:
                msg = json.loads(raw.decode("utf-8"))
            except Exception as exc:
                self.send_json({"type": "error", "version": 1, "reason": f"invalid_json:{exc}"})
                continue

            msg_type = msg.get("type")
            if msg_type == "parking_status":
                ok, reason = validate_parking_status(msg)
                if ok:
                    with latest_lock:
                        latest_parking_status = msg
                    free_slots = [s["id"] for s in msg["slots"] if s["state"] == "FREE"]
                    print(f"[PARKING] seq={msg.get('seq')} free={free_slots}")
                self.send_json({
                    "type": "ack",
                    "version": PROTOCOL_VERSION,
                    "seq": msg.get("seq"),
                    "accepted": ok,
                    "reason": reason,
                })
            elif msg_type in {"safety_event", "trajectory_status"}:
                print(f"[{msg_type.upper()}] {msg}")
                self.send_json({"type": "ack", "version": 1, "accepted": True})
            else:
                self.send_json({"type": "error", "version": 1, "reason": "unsupported_message_type"})
        print(f"[SERVER] disconnected {peer}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, required=True)
    args = parser.parse_args()

    class ReusableTCPServer(socketserver.ThreadingTCPServer):
        allow_reuse_address = True
        daemon_threads = True

    with ReusableTCPServer((args.host, args.port), Handler) as server:
        print(f"[SERVER] listening {args.host}:{args.port} protocol=v1 map={MAP_ID}")
        server.serve_forever()


if __name__ == "__main__":
    main()
