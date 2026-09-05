import time
import statistics
import subprocess
import numpy as np
import onnxruntime as ort

MODEL = "ai/best.onnx"
THREADS = [1, 2, 3, 4]
WARMUP = 5
RUNS = 30
COOL_TEMP = 60.0


def temp():
    try:
        out = subprocess.check_output(
            ["vcgencmd", "measure_temp"],
            text=True
        ).strip()
        return float(out.split("=")[1].replace("'C", ""))
    except Exception:
        return 0.0


def throttled():
    try:
        return subprocess.check_output(
            ["vcgencmd", "get_throttled"],
            text=True
        ).strip()
    except Exception:
        return "N/A"


def clock():
    try:
        out = subprocess.check_output(
            ["vcgencmd", "measure_clock", "arm"],
            text=True
        ).strip()
        hz = int(out.split("=")[1])
        return hz / 1_000_000
    except Exception:
        return 0.0


def wait_cool():
    while True:
        t = temp()
        if t <= COOL_TEMP:
            return
        print(f"Cooling... {t:.1f} C")
        time.sleep(5)


x = np.random.rand(
    1, 3, 640, 640
).astype(np.float32)

print("==============================================")
print(" Raspberry Pi 5 - ONNX Runtime Thread Test")
print("==============================================")
print("Model :", MODEL)
print("ORT   :", ort.__version__)
print()


for threads in THREADS:

    wait_cool()

    opts = ort.SessionOptions()

    opts.intra_op_num_threads = threads
    opts.inter_op_num_threads = 1

    opts.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL

    session = ort.InferenceSession(
        MODEL,
        sess_options=opts,
        providers=["CPUExecutionProvider"]
    )

    inp = session.get_inputs()[0].name

    print()
    print("----------------------------------------------")
    print(f"THREADS = {threads}")
    print("----------------------------------------------")

    print(
        f"Before: temp={temp():.1f} C "
        f"clock={clock():.0f} MHz "
        f"{throttled()}"
    )

    for _ in range(WARMUP):
        session.run(
            None,
            {inp: x}
        )

    latency = []

    wall_start = time.perf_counter()
    cpu_start = time.process_time()

    for _ in range(RUNS):

        t0 = time.perf_counter()

        session.run(
            None,
            {inp: x}
        )

        latency.append(
            (time.perf_counter() - t0) * 1000
        )

    cpu_end = time.process_time()
    wall_end = time.perf_counter()

    avg = statistics.mean(latency)
    p50 = statistics.median(latency)

    sorted_latency = sorted(latency)

    p95 = sorted_latency[
        int(0.95 * (len(sorted_latency) - 1))
    ]

    fps = 1000.0 / avg

    wall = wall_end - wall_start
    cpu = cpu_end - cpu_start

    cpu_pct = (
        cpu / wall * 100.0
        if wall > 0
        else 0.0
    )

    print(f"Average : {avg:.2f} ms")
    print(f"P50     : {p50:.2f} ms")
    print(f"P95     : {p95:.2f} ms")
    print(f"FPS     : {fps:.2f}")
    print(f"CPU     : {cpu_pct:.1f}%")

    print(
        f"After : temp={temp():.1f} C "
        f"clock={clock():.0f} MHz "
        f"{throttled()}"
    )

print()
print("===== FINISHED =====")
