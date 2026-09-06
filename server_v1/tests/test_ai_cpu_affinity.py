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
    def test_parse_default_pair(self) -> None:
        self.assertEqual(MODULE.parse_cpu_list("2,3"), (2, 3))

    def test_parse_is_sorted(self) -> None:
        self.assertEqual(MODULE.parse_cpu_list("3,2"), (2, 3))

    def test_off_disables_affinity(self) -> None:
        self.assertIsNone(MODULE.parse_cpu_list("off"))

    def test_duplicate_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "duplicate_cpu"):
            MODULE.parse_cpu_list("2,2")

    def test_invalid_tokens_are_rejected(self) -> None:
        for value in ("", "2,", "-1,2", "x,2"):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    MODULE.parse_cpu_list(value)

    def test_exact_two_cpu_requirement(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "expected_2_ai_cpus"):
            MODULE.configure_ai_affinity("2")

    def test_disallowed_cpu_fails_closed(self) -> None:
        with mock.patch.object(MODULE.os, "sched_getaffinity", return_value={0, 1}), \
             mock.patch.object(MODULE.os, "sched_setaffinity"):
            with self.assertRaisesRegex(RuntimeError, "requested_cpu_not_allowed"):
                MODULE.configure_ai_affinity("2,3")

    def test_affinity_is_applied_and_verified(self) -> None:
        state = {0, 1, 2, 3}

        def fake_get(_pid: int):
            return set(state)

        def fake_set(_pid: int, cpus):
            state.clear()
            state.update(cpus)

        with mock.patch.object(MODULE.os, "sched_getaffinity", side_effect=fake_get), \
             mock.patch.object(MODULE.os, "sched_setaffinity", side_effect=fake_set):
            actual = MODULE.configure_ai_affinity("2,3")

        self.assertEqual(actual, (2, 3))
        self.assertEqual(state, {2, 3})


if __name__ == "__main__":
    unittest.main()
