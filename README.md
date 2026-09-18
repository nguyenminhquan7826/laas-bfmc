# LAAS lane keeping and obstacle avoidance

Canonical Raspberry Pi 5 / C++14 project for lane keeping and committed lane
change around a detected red-car obstacle. Version: `0.6.0-p0-batch`.

## Build Pure Pursuit on Raspberry Pi

```bash
sudo apt install build-essential cmake libopencv-dev pkg-config libserial-dev
cmake -S . -B build -DLAAS_USE_LIBSERIAL=ON -DLAAS_BUILD_MPC=OFF
cmake --build build -j2
./build/laas_pp pp
```

For a bench build that prints UART commands instead of driving the vehicle,
omit `-DLAAS_USE_LIBSERIAL=ON`.

Use camera device `/dev/video0` for V4L2/USB or set it to `libcamera` for a CSI
camera. The camera calibration and four IPM source points must be calibrated on
the real vehicle before driving.

## Run the PC-side detector

```bash
cd ai
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
python best_AI.py
```

The detector loads `ai/best.onnx` by default. Override it with the environment
variable `LAAS_MODEL_PATH` when necessary.

`config/laas_config.example.yaml` documents the current defaults. The C++
runtime still uses `src/laas_core/Config.hpp`; a YAML loader is not implemented.

P1a host/MCU watchdog changes are intentionally not included in this version.



////// server

// cập nhật git
cd "D:\GitHub\RaspberryPi5\laas-bfmc"

git switch step12-status-sync
git pull --ff-only origin step12-status-sync
git log -1 --oneline

// build lại dashboard

cd .\server_v1\dashboard\frontend
npm.cmd run build

// run server

cd ..\..\

py .\server_stub.py `
  --host 0.0.0.0 `
  --port 5000 `
  --monitor-host 0.0.0.0 `
  --monitor-port 5006 `
  --planning-owner client

// mở dashboard

Start-Process "http://127.0.0.1:5006/"



///run model AI

cd ~/Documents/laas_v0.6.0-p0-batch

.venv-ai/bin/python ai/best_AI.py \
  --model ai/best_int8.onnx \
  --monitor-ip 192.168.1.105 \
  --monitor-port 9998 \
  --cpu-affinity 2,3 \
  --cpu-threads 2

/// run process chính
LAAS_PARKING_POSE_BENCH=1 \
LAAS_PARKING_BENCH_YOLO=1 \
LAAS_PARKING_BENCH_DEBUG=1 \
LAAS_PARKING_SERVER_HOST=192.168.1.105 \
LAAS_MONITOR_IP=192.168.1.105 \
./build/laas_pp pp
