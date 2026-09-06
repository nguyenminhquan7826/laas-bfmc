#!/usr/bin/env python3
"""Launch best_AI.py with the measured Raspberry Pi 5 CPU policy.

Measured default:
  laas_pp -> affinity OFF; Linux may schedule it on CPU0-3
  AI      -> constrained to CPU2,3

Whole-process pinning of laas_pp to CPU1 or CPU0,1 did not improve control and
reduced vision throughput, so LAAS_MAIN_CPU is kept only as a diagnostic
benchmark override. Override the AI mask with LAAS_AI_CPUS; use ``off`` only
for explicit comparison tests.
"""

from __future__ import annotations

import os
import sys

from cpu_affinity import configure_ai_affinity, format_cpu_list


def main() -> int:
    # Limit helper-library worker pools before importing NumPy/OpenCV/ONNX.
    # ONNX itself uses two intra-op threads and one inter-op thread.
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    os.environ.setdefault("MKL_NUM_THREADS", "1")
    os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

    try:
        actual = configure_ai_affinity()
    except (ValueError, RuntimeError, OSError) as exc:
        print(f"[AI][CPU] affinity configuration failed: {exc}", file=sys.stderr)
        return 2

    if actual is None:
        print("[AI][CPU] affinity=OFF")
    else:
        print(
            f"[AI][CPU] affinity={format_cpu_list(actual)} "
            "source=LAAS_AI_CPUS/default"
        )

    # Import only after affinity and thread limits are established.
    import best_AI

    # Avoid an extra OpenCV worker pool competing with ONNX on CPU2/CPU3.
    best_AI.cv2.setNumThreads(1)

    print(
        "[AI][THREADS] ONNX intra=2 inter=1 OpenCV=1 "
        "BLAS/OMP env=1"
    )
    return best_AI.main()


if __name__ == "__main__":
    raise SystemExit(main())
