#!/usr/bin/env python3
"""CPU-affinity helpers for the Raspberry Pi AI process."""

from __future__ import annotations

import os
from typing import Iterable, Optional, Tuple


def parse_cpu_list(text: str) -> Optional[Tuple[int, ...]]:
    value = text.strip()
    if not value:
        raise ValueError("empty_cpu_list")

    if value.lower() in {"off", "none", "disabled"}:
        return None

    cpus = []
    seen = set()
    for token in value.split(","):
        token = token.strip()
        if not token:
            raise ValueError("empty_cpu_token")
        try:
            cpu = int(token, 10)
        except ValueError as exc:
            raise ValueError(f"cpu_not_integer:{token}") from exc
        if cpu < 0:
            raise ValueError(f"cpu_must_be_nonnegative:{cpu}")
        if cpu in seen:
            raise ValueError(f"duplicate_cpu:{cpu}")
        seen.add(cpu)
        cpus.append(cpu)

    if not cpus:
        raise ValueError("empty_cpu_list")

    return tuple(sorted(cpus))


def format_cpu_list(cpus: Iterable[int]) -> str:
    return ",".join(str(cpu) for cpu in sorted(cpus))


def configure_ai_affinity(
    requested_text: Optional[str] = None,
    *,
    expected_cpu_count: int = 2,
) -> Optional[Tuple[int, ...]]:
    """Pin this process to exactly two CPUs by default.

    LAAS_AI_CPUS defaults to ``2,3``. Use ``off`` to disable affinity.
    The function is intentionally strict so an invalid Raspberry Pi CPU layout
    does not silently fall back to unrestricted scheduling.
    """

    if requested_text is None:
        requested_text = os.environ.get("LAAS_AI_CPUS", "2,3")

    requested = parse_cpu_list(requested_text)
    if requested is None:
        return None

    if len(requested) != expected_cpu_count:
        raise RuntimeError(
            f"expected_{expected_cpu_count}_ai_cpus:got_{len(requested)}"
        )

    if not hasattr(os, "sched_getaffinity") or not hasattr(os, "sched_setaffinity"):
        raise RuntimeError("cpu_affinity_requires_linux")

    allowed = set(os.sched_getaffinity(0))
    missing = [cpu for cpu in requested if cpu not in allowed]
    if missing:
        raise RuntimeError(
            "requested_cpu_not_allowed:"
            f"requested={format_cpu_list(requested)} "
            f"allowed={format_cpu_list(allowed)}"
        )

    os.sched_setaffinity(0, set(requested))
    actual = tuple(sorted(os.sched_getaffinity(0)))
    if actual != requested:
        raise RuntimeError(
            "affinity_verification_mismatch:"
            f"requested={format_cpu_list(requested)} "
            f"actual={format_cpu_list(actual)}"
        )

    return actual
