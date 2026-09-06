#!/usr/bin/env python3
"""Launch best_AI.py with strict Raspberry Pi CPU affinity.

Default layout:
  CPU0 -> left available for OS / IRQ / background work
  CPU1 -> laas_pp
  CPU2,3 -> this AI process

Override the AI mask with LAAS_AI_CPUS=2,3. Use LAAS_AI_CPUS=off to disable.
"""

from __future__ import annotations

import os
import sys

from cpu_affinity import configure_ai_affinity, format_cpu_list


def main() -> int:
    # Keep helper libraries from creating additional large thread pools before
    # importing NumPy/OpenCV/ONNX Runtime. ONNX itself is explicitly configured
    # in best_AI.py with intra_op_num_threads=2 and inter_op_num_threads=1.
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

    # Import only after affinity and thread-limit environment are established.
    import best_AI

    # Avoid an independent OpenCV worker pool competing with ONNX on CPU2/CPU3.
    best_AI.cv2.setNumThreads(1)

    print(
        "[AI][THREADS] ONNX intra=2 inter=1 OpenCV=1 "
        "BLAS/OMP env=1"
    )
    return best_AI.main()


if __name__ == "__main__":
    raise SystemExit(main())
