#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = ROOT / "ai" / "cpu_affinity.py"
SPEC = importlib.util.spec_from_file_location("laas_ai_cpu_affinity", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class AiCpuAffinityTests(unittest.TestCase):
    def test_default_policy_applies_cpu_2_3(self) -> None:
        state = {0, 1, 2, 3}

        def fake_get(_pid: int):
            return set(state)

        def fake_set(_pid: int, cpus):
            state.clear()
            state.update(cpus)

        with mock.patch.dict(MODULE.os.environ, {}, clear=True), \
             mock.patch.object(MODULE.os, "sched_getaffinity", side_effect=fake_get), \
             mock.patch.object(MODULE.os, "sched_setaffinity", side_effect=fake_set):
            actual = MODULE.configure_ai_affinity()

        self.assertEqual(actual, (2, 3))
        self.assertEqual(state, {2, 3})

    def test_off_is_available_for_explicit_benchmarks(self) -> None:
        self.assertIsNone(MODULE.configure_ai_affinity("off"))

    def test_wrong_cpu_count_fails_closed(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "expected_2_ai_cpus"):
            MODULE.configure_ai_affinity("2")

    def test_disallowed_cpu_fails_closed(self) -> None:
        with mock.patch.object(MODULE.os, "sched_getaffinity", return_value={0, 1}), \
             mock.patch.object(MODULE.os, "sched_setaffinity"):
            with self.assertRaisesRegex(RuntimeError, "requested_cpu_not_allowed"):
                MODULE.configure_ai_affinity("2,3")

    def test_invalid_cpu_token_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            MODULE.configure_ai_affinity("2,x")


if __name__ == "__main__":
    unittest.main()
