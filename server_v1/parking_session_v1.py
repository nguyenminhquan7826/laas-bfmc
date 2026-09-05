#!/usr/bin/env python3
"""Parking session state machine for LAAS server_v1.

This module manages orchestration only. It never commands actuators.
"""

from __future__ import annotations

import threading
from typing import Any, Optional

SESSION_STATES = {
    "IDLE",
    "WAITING_INPUT",
    "PLANNING",
    "TRAJECTORY_READY",
    "EXECUTING",
    "PAUSED",
    "REPLAN",
    "COMPLETED",
}


class ParkingSession:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._session_counter = 0
        self._session_id = 0
        self._state = "IDLE"
        self._reason = "startup"
        self._transition_seq = 0
        self._active_trajectory_id: Optional[int] = None
        self._target_slot: Optional[str] = None
        self._last_target_slot: Optional[str] = None
        self._replan_count = 0
        self._replan_pending = False
        self._pause_reason: Optional[str] = None

    def _transition(self, new_state: str, reason: str) -> None:
        if new_state not in SESSION_STATES:
            raise ValueError(f"invalid_session_state:{new_state}")
        self._state = new_state
        self._reason = reason
        self._transition_seq += 1

    def _new_session_locked(self, reason: str) -> None:
        self._session_counter += 1
        self._session_id = self._session_counter
        self._active_trajectory_id = None
        self._target_slot = None
        self._last_target_slot = None
        self._replan_count = 0
        self._replan_pending = False
        self._pause_reason = None
        self._transition("WAITING_INPUT", reason)

    def ensure_started(self, reason: str = "input_received") -> None:
        with self._lock:
            if self._state == "IDLE":
                self._new_session_locked(reason)

    def start_new_session(self, reason: str = "new_session_requested") -> None:
        with self._lock:
            self._new_session_locked(reason)

    def set_waiting(self, reason: str, *, replan_pending: Optional[bool] = None) -> None:
        with self._lock:
            if self._state == "IDLE":
                self._new_session_locked(reason)
            else:
                self._transition("WAITING_INPUT", reason)
            if replan_pending is not None:
                self._replan_pending = bool(replan_pending)

    def start_planning(self, reason: str) -> None:
        with self._lock:
            if self._state == "IDLE":
                self._new_session_locked("planning_started_without_prior_input")
            self._transition("PLANNING", reason)

    def mark_trajectory_ready(self, trajectory_id: int, target_slot: str, reason: str = "trajectory_validated") -> None:
        with self._lock:
            self._active_trajectory_id = int(trajectory_id)
            self._target_slot = str(target_slot)
            self._last_target_slot = str(target_slot)
            self._pause_reason = None
            self._replan_pending = False
            self._transition("TRAJECTORY_READY", reason)

    def mark_received(self, trajectory_id: int) -> tuple[bool, str]:
        with self._lock:
            ok, reason = self._match_active_locked(trajectory_id)
            if not ok:
                return ok, reason
            if self._state not in {"TRAJECTORY_READY", "EXECUTING"}:
                return False, f"invalid_state_for_RECEIVED:{self._state}"
            return True, "ok"

    def mark_executing(self, trajectory_id: int) -> tuple[bool, str]:
        with self._lock:
            ok, reason = self._match_active_locked(trajectory_id)
            if not ok:
                return ok, reason
            if self._state not in {"TRAJECTORY_READY", "EXECUTING"}:
                return False, f"invalid_state_for_EXECUTING:{self._state}"
            self._transition("EXECUTING", "trajectory_status_EXECUTING")
            return True, "ok"

    def mark_paused(self, trajectory_id: Optional[int], reason: str, *, replan_pending: bool = False) -> tuple[bool, str]:
        with self._lock:
            if trajectory_id is not None:
                ok, match_reason = self._match_active_locked(trajectory_id)
                if not ok:
                    return ok, match_reason
            if self._state not in {"TRAJECTORY_READY", "EXECUTING", "PAUSED"}:
                return False, f"invalid_state_for_PAUSED:{self._state}"
            self._pause_reason = reason
            self._replan_pending = self._replan_pending or bool(replan_pending)
            self._transition("PAUSED", reason)
            return True, "ok"

    def mark_completed(self, trajectory_id: int) -> tuple[bool, str]:
        with self._lock:
            ok, reason = self._match_active_locked(trajectory_id)
            if not ok:
                return ok, reason
            if self._state not in {"TRAJECTORY_READY", "EXECUTING"}:
                return False, f"invalid_state_for_COMPLETED:{self._state}"
            self._pause_reason = None
            self._replan_pending = False
            self._transition("COMPLETED", "trajectory_status_COMPLETED")
            return True, "ok"

    def request_replan(self, reason: str, *, increment: bool = True) -> None:
        with self._lock:
            if self._state == "IDLE":
                self._new_session_locked("replan_requested_without_session")
            if increment:
                self._replan_count += 1
            if self._target_slot is not None:
                self._last_target_slot = self._target_slot
            self._active_trajectory_id = None
            self._target_slot = None
            self._pause_reason = None
            self._replan_pending = True
            self._transition("REPLAN", reason)

    def mark_replan_pending_while_paused(self, reason: str) -> None:
        with self._lock:
            if self._state != "PAUSED":
                self.request_replan(reason)
                return
            self._replan_pending = True
            self._reason = reason
            self._transition_seq += 1

    def clear_safety_and_request_replan(self, reason: str = "safety_cleared") -> tuple[bool, str]:
        with self._lock:
            if self._state != "PAUSED":
                return False, f"invalid_state_for_SAFETY_CLEARED:{self._state}"
            self._replan_count += 1
            if self._target_slot is not None:
                self._last_target_slot = self._target_slot
            self._active_trajectory_id = None
            self._target_slot = None
            self._pause_reason = None
            self._replan_pending = True
            self._transition("REPLAN", reason)
            return True, "ok"

    def target_became_invalid(self, new_state: str) -> str:
        with self._lock:
            if self._target_slot is None:
                return "NO_ACTIVE_TARGET"
            reason = f"target_slot_{self._target_slot}_became_{new_state}"
            if self._state == "PAUSED":
                self._replan_pending = True
                self._reason = reason
                self._transition_seq += 1
                return "DEFERRED_WHILE_PAUSED"
            self._replan_count += 1
            self._last_target_slot = self._target_slot
            self._active_trajectory_id = None
            self._target_slot = None
            self._pause_reason = None
            self._replan_pending = True
            self._transition("REPLAN", reason)
            return "REPLAN"

    def _match_active_locked(self, trajectory_id: int) -> tuple[bool, str]:
        if not isinstance(trajectory_id, int) or isinstance(trajectory_id, bool) or trajectory_id <= 0:
            return False, "invalid_trajectory_id"
        if self._active_trajectory_id is None:
            return False, "no_active_trajectory"
        if int(trajectory_id) != self._active_trajectory_id:
            return False, "trajectory_id_mismatch"
        return True, "ok"

    def active_matches(self, trajectory_id: Optional[int]) -> bool:
        with self._lock:
            return trajectory_id is not None and self._active_trajectory_id == trajectory_id

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "session_id": self._session_id,
                "state": self._state,
                "reason": self._reason,
                "transition_seq": self._transition_seq,
                "active_trajectory_id": self._active_trajectory_id,
                "target_slot": self._target_slot,
                "last_target_slot": self._last_target_slot,
                "replan_count": self._replan_count,
                "replan_pending": self._replan_pending,
                "pause_reason": self._pause_reason,
            }
