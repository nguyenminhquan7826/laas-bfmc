import time
import statistics
import subprocess
import numpy as np
import onnxruntime as ort

MODEL = "ai/best.onnx"
WARMUP = 10
RUNS = 100


def get_temp():
    try:
        out = subprocess.check_output(
            ["vcgencmd", "measure_temp"],
            text=True
        ).strip()
        return float(out.split("=")[1].replace("'C", ""))
    except Exception:
        return None


def get_throttled():
    try:
        return subprocess.check_output(
            ["vcgencmd", "get_throttled"],
            text=True
        ).strip()
    except Exception:
        return "N/A"


def get_rss_mib():
    try:
        with open("/proc/self/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    kb = int(line.split()[1])
                    return kb / 1024.0
    except Exception:
        pass
    return 0.0


print("===== RASPBERRY PI ONNX BENCHMARK =====")
print("Model:", MODEL)
print("ONNX Runtime:", ort.__version__)

opts = ort.SessionOptions()

session = ort.InferenceSession(
    MODEL,
    sess_options=opts,
    providers=["CPUExecutionProvider"]
)

input_info = session.get_inputs()[0]

print("Provider:", session.get_providers())
print("Input:", input_info.name)
print("Shape:", input_info.shape)
print("Output:", session.get_outputs()[0].shape)

# tensor giả lập ảnh đã preprocess
x = np.random.rand(
    1, 3, 640, 640
).astype(np.float32)

print("\nWarmup...")

for _ in range(WARMUP):
    session.run(
        None,
        {input_info.name: x}
    )

temp_before = get_temp()
ram_before = get_rss_mib()

latencies = []

wall_start = time.perf_counter()
cpu_start = time.process_time()

for _ in range(RUNS):

    start = time.perf_counter()

    session.run(
        None,
        {input_info.name: x}
    )

    end = time.perf_counter()

    latencies.append(
        (end - start) * 1000.0
    )

cpu_end = time.process_time()
wall_end = time.perf_counter()

temp_after = get_temp()
ram_after = get_rss_mib()

latencies_sorted = sorted(latencies)

avg = statistics.mean(latencies)
p50 = statistics.median(latencies)

p95_idx = int(
    0.95 * (len(latencies_sorted) - 1)
)

p95 = latencies_sorted[p95_idx]

fps = 1000.0 / avg

wall_time = wall_end - wall_start
cpu_time = cpu_end - cpu_start

cpu_pct = (
    cpu_time / wall_time * 100.0
    if wall_time > 0
    else 0.0
)

print("\n===== RESULTS =====")
print(f"Runs          : {RUNS}")
print(f"Average       : {avg:.2f} ms")
print(f"P50           : {p50:.2f} ms")
print(f"P95           : {p95:.2f} ms")
print(f"Min           : {min(latencies):.2f} ms")
print(f"Max           : {max(latencies):.2f} ms")
print(f"Inference FPS : {fps:.2f}")
print(f"Process CPU   : {cpu_pct:.1f}%")
print(f"RAM before    : {ram_before:.1f} MiB")
print(f"RAM after     : {ram_after:.1f} MiB")

if temp_before is not None:
    print(f"Temp before   : {temp_before:.1f} C")

if temp_after is not None:
    print(f"Temp after    : {temp_after:.1f} C")

print(f"Power state   : {get_throttled()}")
