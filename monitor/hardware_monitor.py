#!/usr/bin/env python3
import os
import socket
import subprocess
import time
from collections import deque
from pathlib import Path
import tkinter as tk

UPDATE_MS = 1000
HISTORY = 60

BG = "#111318"
PANEL = "#171a21"
PANEL2 = "#1e222b"
BORDER = "#2b303b"
TEXT = "#e6edf3"
MUTED = "#9aa4b2"
ACCENT = "#4cc2ff"
GREEN = "#56d364"
YELLOW = "#f2cc60"
RED = "#ff6b6b"
PURPLE = "#b48ead"

LAAS_PROCESSES = ["laas_pp", "laas_mpc"]

_prev_cpu = None
_prev_net = {}
_last_net_ts = None


def run_cmd(cmd):
    try:
        p = subprocess.run(
            cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=1.5,
        )
        return p.stdout.strip()
    except Exception:
        return ""


def read_file(path):
    try:
        return Path(path).read_text().strip()
    except Exception:
        return ""


def format_bytes(n):
    try:
        n = float(n)
    except Exception:
        return "N/A"
    units = ["B", "KB", "MB", "GB", "TB"]
    i = 0
    while n >= 1024 and i < len(units) - 1:
        n /= 1024.0
        i += 1
    return f"{n:.1f} {units[i]}"


def format_rate(n):
    if n < 1024:
        return f"{n:.0f} B/s"
    if n < 1024 * 1024:
        return f"{n / 1024:.1f} KB/s"
    return f"{n / (1024 * 1024):.2f} MB/s"


def get_meminfo():
    data = {}
    for line in read_file("/proc/meminfo").splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        parts = value.strip().split()
        if parts:
            try:
                data[key] = int(parts[0]) * 1024
            except ValueError:
                pass
    total = data.get("MemTotal", 0)
    available = data.get("MemAvailable", 0)
    used = max(total - available, 0)
    swap_total = data.get("SwapTotal", 0)
    swap_free = data.get("SwapFree", 0)
    swap_used = max(swap_total - swap_free, 0)
    ram_pct = (used / total * 100.0) if total else 0
    swap_pct = (swap_used / swap_total * 100.0) if swap_total else 0
    return {
        "ram_used": used,
        "ram_total": total,
        "ram_pct": ram_pct,
        "swap_used": swap_used,
        "swap_total": swap_total,
        "swap_pct": swap_pct,
    }


def get_cpu_usage():
    global _prev_cpu
    raw = read_file("/proc/stat").splitlines()
    if not raw:
        return None
    parts = raw[0].split()
    if len(parts) < 8:
        return None
    vals = list(map(int, parts[1:]))
    idle = vals[3] + vals[4]
    total = sum(vals)

    current = (idle, total)
    if _prev_cpu is None:
        _prev_cpu = current
        return None

    idle_delta = idle - _prev_cpu[0]
    total_delta = total - _prev_cpu[1]
    _prev_cpu = current

    if total_delta <= 0:
        return None
    usage = 100.0 * (1.0 - idle_delta / total_delta)
    return max(0.0, min(100.0, usage))


def get_loadavg():
    try:
        a, b, c = os.getloadavg()
        return f"{a:.2f} / {b:.2f} / {c:.2f}"
    except Exception:
        return "N/A"


def get_cpu_temp():
    out = run_cmd("vcgencmd measure_temp")
    if "temp=" in out:
        try:
            value = float(out.split("=")[1].replace("'C", ""))
            return value
        except Exception:
            pass
    raw = read_file("/sys/class/thermal/thermal_zone0/temp")
    if raw:
        try:
            return int(raw) / 1000.0
        except Exception:
            pass
    return None


def get_cpu_clock():
    out = run_cmd("vcgencmd measure_clock arm")
    if "=" in out:
        try:
            hz = int(out.split("=")[1])
            return f"{hz / 1_000_000:.0f} MHz"
        except Exception:
            pass
    raw = read_file("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
    if raw:
        try:
            return f"{int(raw) / 1000:.0f} MHz"
        except Exception:
            pass
    return "N/A"


def decode_throttled():
    out = run_cmd("vcgencmd get_throttled")
    if not out:
        return "N/A", MUTED
    try:
        value = int(out.split("=")[1], 16)
    except Exception:
        return out, MUTED

    if value == 0:
        return "0x0 — OK", GREEN

    flags = [
        (0, "undervoltage hiện tại"),
        (1, "giới hạn ARM hiện tại"),
        (2, "đang throttling"),
        (3, "soft temp limit"),
        (16, "đã từng undervoltage"),
        (17, "đã từng giới hạn ARM"),
        (18, "đã từng throttling"),
        (19, "đã từng soft temp limit"),
    ]
    active = [text for bit, text in flags if value & (1 << bit)]
    return f"0x{value:X} — " + "; ".join(active), RED


def get_disk():
    try:
        st = os.statvfs("/")
        total = st.f_blocks * st.f_frsize
        free = st.f_bavail * st.f_frsize
        used = total - free
        pct = (used / total * 100.0) if total else 0
        return {
            "used": used,
            "total": total,
            "pct": pct,
            "text": f"{format_bytes(used)} / {format_bytes(total)} ({pct:.1f}%)",
        }
    except Exception:
        return {"used": 0, "total": 0, "pct": 0, "text": "N/A"}


def get_uptime():
    raw = read_file("/proc/uptime")
    if not raw:
        return "N/A"
    try:
        s = int(float(raw.split()[0]))
        days, s = divmod(s, 86400)
        hours, s = divmod(s, 3600)
        mins, sec = divmod(s, 60)
        if days:
            return f"{days}d {hours:02d}:{mins:02d}:{sec:02d}"
        return f"{hours:02d}:{mins:02d}:{sec:02d}"
    except Exception:
        return "N/A"


def get_ip():
    out = run_cmd("hostname -I")
    return out.split()[0] if out else "N/A"


def get_wifi():
    state = run_cmd("nmcli -t -f GENERAL.STATE,GENERAL.CONNECTION device show wlan0")
    if state:
        lines = []
        for item in state.splitlines():
            lines.append(item.replace("GENERAL.STATE:", "").replace("GENERAL.CONNECTION:", ""))
        if len(lines) >= 2:
            return f"{lines[0]} | {lines[1]}"
        return " | ".join(lines)
    return "N/A"


def get_eth():
    state = run_cmd("nmcli -t -f GENERAL.STATE,GENERAL.CONNECTION device show eth0")
    if state:
        lines = []
        for item in state.splitlines():
            lines.append(item.replace("GENERAL.STATE:", "").replace("GENERAL.CONNECTION:", ""))
        if len(lines) >= 2:
            return f"{lines[0]} | {lines[1]}"
        return " | ".join(lines)
    return "N/A"


def get_network_rates(interface="wlan0"):
    global _prev_net, _last_net_ts

    base = Path(f"/sys/class/net/{interface}/statistics")
    rx_raw = read_file(base / "rx_bytes")
    tx_raw = read_file(base / "tx_bytes")
    now = time.time()

    try:
        rx = int(rx_raw)
        tx = int(tx_raw)
    except Exception:
        return 0.0, 0.0

    if interface not in _prev_net or _last_net_ts is None:
        _prev_net[interface] = (rx, tx)
        _last_net_ts = now
        return 0.0, 0.0

    prev_rx, prev_tx = _prev_net[interface]
    dt = max(now - _last_net_ts, 1e-6)
    _prev_net[interface] = (rx, tx)
    _last_net_ts = now

    rx_rate = max(0.0, (rx - prev_rx) / dt)
    tx_rate = max(0.0, (tx - prev_tx) / dt)
    return rx_rate, tx_rate


def get_laas_status():
    rows = []
    for name in LAAS_PROCESSES:
        out = run_cmd(
            f"ps -C {name} -o pid=,%cpu=,%mem=,rss=,etime= --no-headers | head -1"
        )
        if out:
            parts = out.split()
            if len(parts) >= 5:
                try:
                    rss_mib = float(parts[3]) / 1024.0
                    rss_text = f"{rss_mib:.1f} MiB"
                except Exception:
                    rss_text = "N/A"
                rows.append(
                    f"{name}: PID {parts[0]} | CPU {parts[1]}% | "
                    f"MEM {parts[2]}% | RAM {rss_text} | TIME {parts[4]}"
                )
    return "\n".join(rows) if rows else "Không chạy"


def get_laas_ram_mib():
    result = {"laas_pp": 0.0, "laas_mpc": 0.0}

    for name in LAAS_PROCESSES:
        out = run_cmd(
            f"ps -C {name} -o rss= --no-headers | "
            "awk '{sum += $1} END {print sum+0}'"
        )
        try:
            result[name] = float(out) / 1024.0
        except Exception:
            result[name] = 0.0

    return result


def get_top_processes():
    # RSS là resident set size: lượng RAM vật lý process đang chiếm, đơn vị KiB.
    out = run_cmd(
        "ps -eo pid=,comm=,%cpu=,%mem=,rss=,etime= "
        "--sort=-rss | head -n 12"
    )
    if not out:
        return "Không đọc được process list"

    lines = [
        f"{'PID':>7}  {'COMMAND':<22} {'CPU%':>6} {'MEM%':>6} {'RAM(MiB)':>10} {'ELAPSED':>12}"
    ]

    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 6:
            continue

        pid, command, cpu, mem, rss_kib = parts[:5]
        elapsed = parts[5]

        try:
            ram_mib = float(rss_kib) / 1024.0
            ram_text = f"{ram_mib:.1f}"
        except Exception:
            ram_text = "N/A"

        lines.append(
            f"{pid:>7}  {command:<22.22} {cpu:>6} {mem:>6} "
            f"{ram_text:>10} {elapsed:>12}"
        )

    return "\n".join(lines)


class LineChart(tk.Canvas):
    def __init__(self, master, title, unit="", color=ACCENT, max_value=100, autoscale=False, **kwargs):
        super().__init__(master, bg=PANEL2, highlightthickness=1, highlightbackground=BORDER, **kwargs)
        self.title = title
        self.unit = unit
        self.color = color
        self.fixed_max = max_value
        self.autoscale = autoscale
        self.history = deque(maxlen=HISTORY)
        self.current_text = "N/A"
        self.bind("<Configure>", lambda e: self.redraw())

    def push(self, value, text=None):
        self.history.append(float(value))
        if text is None:
            self.current_text = f"{value:.1f}{self.unit}"
        else:
            self.current_text = text
        self.redraw()

    def redraw(self):
        self.delete("all")
        w = max(self.winfo_width(), 50)
        h = max(self.winfo_height(), 50)
        pad = 14
        header_h = 34
        plot_x0 = pad
        plot_y0 = pad + header_h
        plot_x1 = w - pad
        plot_y1 = h - pad
        plot_h = max(plot_y1 - plot_y0, 10)
        plot_w = max(plot_x1 - plot_x0, 10)

        self.create_text(pad, pad, anchor="nw", text=self.title, fill=TEXT, font=("Segoe UI", 11, "bold"))
        self.create_text(w - pad, pad, anchor="ne", text=self.current_text, fill=MUTED, font=("Consolas", 10))

        # grid lines
        for i in range(5):
            y = plot_y0 + plot_h * i / 4.0
            self.create_line(plot_x0, y, plot_x1, y, fill="#263041")

        if len(self.history) < 2:
            return

        vmax = self.fixed_max
        if self.autoscale:
            vmax = max(max(self.history) * 1.2, 1.0)
        vmax = max(vmax, 1.0)

        points = []
        n = len(self.history)
        for idx, value in enumerate(self.history):
            x = plot_x0 + plot_w * idx / max(n - 1, 1)
            y = plot_y1 - (min(value, vmax) / vmax) * plot_h
            points.extend((x, y))

        # area
        area_points = [plot_x0, plot_y1] + points + [plot_x1, plot_y1]
        self.create_polygon(area_points, fill=self.color, stipple="gray25", outline="")

        self.create_line(*points, fill=self.color, width=2, smooth=True)
        self.create_text(plot_x0, plot_y0 - 4, anchor="sw", text="0", fill=MUTED, font=("Consolas", 8))
        self.create_text(plot_x1, plot_y0 - 4, anchor="se", text=f"{vmax:.0f}{self.unit}", fill=MUTED, font=("Consolas", 8))


class MultiLineChart(tk.Canvas):
    def __init__(self, master, title, series, unit="MiB", **kwargs):
        super().__init__(
            master,
            bg=PANEL2,
            highlightthickness=1,
            highlightbackground=BORDER,
            **kwargs
        )
        self.title = title
        self.series = series
        self.unit = unit
        self.history = {
            key: deque(maxlen=HISTORY)
            for key in series
        }
        self.current = {
            key: 0.0
            for key in series
        }
        self.bind("<Configure>", lambda e: self.redraw())

    def push(self, values):
        for key in self.series:
            value = float(values.get(key, 0.0))
            self.current[key] = value
            self.history[key].append(value)
        self.redraw()

    def redraw(self):
        self.delete("all")

        w = max(self.winfo_width(), 50)
        h = max(self.winfo_height(), 50)

        pad = 14
        header_h = 42
        plot_x0 = pad
        plot_y0 = pad + header_h
        plot_x1 = w - pad
        plot_y1 = h - pad

        plot_h = max(plot_y1 - plot_y0, 10)
        plot_w = max(plot_x1 - plot_x0, 10)

        self.create_text(
            pad, pad,
            anchor="nw",
            text=self.title,
            fill=TEXT,
            font=("Segoe UI", 11, "bold")
        )

        # Legend
        legend_x = w - pad
        legend_y = pad
        for key, meta in reversed(list(self.series.items())):
            name = meta["label"]
            color = meta["color"]
            value = self.current.get(key, 0.0)

            text = f"{name}: {value:.1f} {self.unit}"
            self.create_text(
                legend_x,
                legend_y,
                anchor="ne",
                text=text,
                fill=color,
                font=("Consolas", 9, "bold")
            )
            legend_y += 16

        # Grid
        for i in range(5):
            y = plot_y0 + plot_h * i / 4.0
            self.create_line(
                plot_x0, y, plot_x1, y,
                fill="#263041"
            )

        all_values = []
        for hist in self.history.values():
            all_values.extend(hist)

        vmax = max(max(all_values, default=0.0) * 1.25, 32.0)

        self.create_text(
            plot_x0,
            plot_y0 - 4,
            anchor="sw",
            text="0",
            fill=MUTED,
            font=("Consolas", 8)
        )
        self.create_text(
            plot_x1,
            plot_y0 - 4,
            anchor="se",
            text=f"{vmax:.0f} {self.unit}",
            fill=MUTED,
            font=("Consolas", 8)
        )

        for key, meta in self.series.items():
            hist = self.history[key]
            if len(hist) < 2:
                continue

            color = meta["color"]
            points = []
            n = len(hist)

            for idx, value in enumerate(hist):
                x = plot_x0 + plot_w * idx / max(n - 1, 1)
                y = plot_y1 - (min(value, vmax) / vmax) * plot_h
                points.extend((x, y))

            self.create_line(
                *points,
                fill=color,
                width=2,
                smooth=True
            )

        # warning if rising strongly
        for key, meta in self.series.items():
            hist = list(self.history[key])

            if len(hist) >= 30:
                first = sum(hist[:10]) / 10.0
                last = sum(hist[-10:]) / 10.0
                increase = last - first

                if first > 0.0 and increase > max(25.0, first * 0.25):
                    self.create_text(
                        pad,
                        h - pad,
                        anchor="sw",
                        text=f"⚠ {meta['label']} RAM tăng {increase:.1f} MiB trong cửa sổ theo dõi",
                        fill=RED,
                        font=("Segoe UI", 9, "bold")
                    )
                    break


class Dashboard(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Raspberry Pi 5 — Hardware Monitor Dashboard")
        self.geometry("1220x780")
        self.minsize(1000, 680)
        self.configure(bg=BG)

        self.metric_vars = {}
        self.detail_vars = {}
        self.chart_widgets = {}

        self.build_ui()
        self.refresh()

    def make_metric_card(self, parent, title, key, color=ACCENT):
        outer = tk.Frame(parent, bg=PANEL, highlightthickness=1, highlightbackground=BORDER)
        outer.pack(side="left", fill="both", expand=True, padx=6, pady=6)

        tk.Label(outer, text=title, bg=PANEL, fg=MUTED, font=("Segoe UI", 10)).pack(anchor="w", padx=14, pady=(10, 2))
        val = tk.StringVar(value="Đang đọc...")
        self.metric_vars[key] = (val, outer)

        lbl = tk.Label(outer, textvariable=val, bg=PANEL, fg=color, font=("Segoe UI", 20, "bold"))
        lbl.pack(anchor="w", padx=14, pady=(0, 12))
        return outer

    def make_detail(self, parent, title, key):
        row = tk.Frame(parent, bg=PANEL)
        row.pack(fill="x", pady=3)
        tk.Label(row, text=title, bg=PANEL, fg=MUTED, font=("Segoe UI", 10), width=16, anchor="w").pack(side="left")
        var = tk.StringVar(value="...")
        self.detail_vars[key] = var
        tk.Label(row, textvariable=var, bg=PANEL, fg=TEXT, font=("Consolas", 10), anchor="w", justify="left", wraplength=380).pack(side="left", fill="x", expand=True)
        return row

    def section(self, parent, title):
        frame = tk.Frame(parent, bg=PANEL, highlightthickness=1, highlightbackground=BORDER)
        header = tk.Label(frame, text=title, bg=PANEL, fg=TEXT, font=("Segoe UI", 11, "bold"))
        header.pack(anchor="w", padx=12, pady=(10, 8))
        return frame

    def build_ui(self):
        header = tk.Frame(self, bg=BG)
        header.pack(fill="x", padx=12, pady=(10, 4))
        tk.Label(header, text="Raspberry Pi 5 — Hardware Monitor", bg=BG, fg=TEXT, font=("Segoe UI", 20, "bold")).pack(side="left")
        self.time_var = tk.StringVar(value="")
        tk.Label(header, textvariable=self.time_var, bg=BG, fg=MUTED, font=("Consolas", 11)).pack(side="right")

        cards = tk.Frame(self, bg=BG)
        cards.pack(fill="x", padx=12, pady=(0, 4))
        self.make_metric_card(cards, "CPU", "cpu", ACCENT)
        self.make_metric_card(cards, "RAM", "ram", GREEN)
        self.make_metric_card(cards, "Temperature", "temp", YELLOW)
        self.make_metric_card(cards, "Disk", "disk", PURPLE)
        self.make_metric_card(cards, "Network", "net", ACCENT)

        content = tk.Frame(self, bg=BG)
        content.pack(fill="both", expand=True, padx=12, pady=8)

        # left main
        left = tk.Frame(content, bg=BG)
        left.pack(side="left", fill="both", expand=True)

        # charts 2x2
        charts = tk.Frame(left, bg=BG)
        charts.pack(fill="both", expand=True)

        self.chart_widgets["cpu"] = LineChart(charts, "CPU Usage", "%", ACCENT, max_value=100, width=400, height=220)
        self.chart_widgets["ram"] = LineChart(charts, "RAM Usage", "%", GREEN, max_value=100, width=400, height=220)
        self.chart_widgets["temp"] = LineChart(charts, "CPU Temperature", "°C", YELLOW, max_value=100, width=400, height=220)
        self.chart_widgets["net"] = LineChart(charts, "Network Throughput", "", PURPLE, max_value=1024, autoscale=True, width=400, height=220)

        self.chart_widgets["cpu"].grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
        self.chart_widgets["ram"].grid(row=0, column=1, sticky="nsew", padx=6, pady=6)
        self.chart_widgets["temp"].grid(row=1, column=0, sticky="nsew", padx=6, pady=6)
        self.chart_widgets["net"].grid(row=1, column=1, sticky="nsew", padx=6, pady=6)
        charts.rowconfigure(0, weight=1)
        charts.rowconfigure(1, weight=1)
        charts.columnconfigure(0, weight=1)
        charts.columnconfigure(1, weight=1)

        # LAAS RAM history
        laas_ram_sec = tk.Frame(left, bg=BG)
        laas_ram_sec.pack(fill="x", padx=6, pady=(6, 0))

        self.laas_ram_chart = MultiLineChart(
            laas_ram_sec,
            "LAAS RAM History",
            {
                "laas_pp": {
                    "label": "laas_pp",
                    "color": ACCENT,
                },
                "laas_mpc": {
                    "label": "laas_mpc",
                    "color": GREEN,
                },
            },
            unit="MiB",
            height=180,
        )
        self.laas_ram_chart.pack(fill="x", expand=False)

        # bottom left process table
        proc_sec = self.section(left, "Processes — RAM usage (cao → thấp)")
        proc_sec.pack(fill="both", expand=False, padx=6, pady=(6, 0))
        self.proc_text = tk.Text(proc_sec, height=9, bg=PANEL2, fg=TEXT, insertbackground=TEXT,
                                 relief="flat", font=("Consolas", 10))
        self.proc_text.pack(fill="both", expand=True, padx=12, pady=(0, 12))
        self.proc_text.config(state="disabled")

        # right side details
        right = tk.Frame(content, bg=BG, width=380)
        right.pack(side="left", fill="y", padx=(6, 0))
        right.pack_propagate(False)

        sys_sec = self.section(right, "System Details")
        sys_sec.pack(fill="x", padx=6, pady=6)
        for title, key in [
            ("Load avg", "load"),
            ("ARM clock", "clock"),
            ("Throttled", "throttled"),
            ("Uptime", "uptime"),
            ("IP address", "ip"),
            ("Wi-Fi wlan0", "wifi"),
            ("Ethernet eth0", "eth"),
            ("Swap", "swap"),
            ("Disk /", "disk_text"),
        ]:
            self.make_detail(sys_sec, title, key)

        laas_sec = self.section(right, "LAAS")
        laas_sec.pack(fill="x", padx=6, pady=6)
        self.laas_label = tk.Label(laas_sec, text="Đang đọc...", bg=PANEL2, fg=TEXT, font=("Consolas", 10),
                                   justify="left", anchor="nw", wraplength=330, padx=10, pady=10)
        self.laas_label.pack(fill="both", expand=True, padx=12, pady=(0, 12))

        help_sec = self.section(right, "Notes")
        help_sec.pack(fill="both", expand=True, padx=6, pady=6)
        note = (
            "• Giao diện này chạy tốt qua VNC.\n"
            "• Nếu 'Throttled' khác 0x0 thì cần kiểm tra nguồn.\n"
            "• Bảng Processes hiển thị RAM vật lý theo MiB (RSS).\n""• Có thể mở cùng lúc với laas_pp / laas_mpc.\n"
            "• Sau này có thể mở rộng thêm telemetry STM32."
        )
        tk.Label(help_sec, text=note, bg=PANEL2, fg=MUTED, font=("Segoe UI", 10),
                 justify="left", anchor="nw", wraplength=330, padx=10, pady=10).pack(fill="both", expand=True, padx=12, pady=(0, 12))

    def set_proc_text(self, text):
        self.proc_text.config(state="normal")
        self.proc_text.delete("1.0", "end")
        self.proc_text.insert("1.0", text)
        self.proc_text.config(state="disabled")

    def refresh(self):
        self.time_var.set(time.strftime("%Y-%m-%d %H:%M:%S"))

        cpu = get_cpu_usage()
        mem = get_meminfo()
        temp = get_cpu_temp()
        clock = get_cpu_clock()
        throttled_text, throttled_color = decode_throttled()
        disk = get_disk()
        uptime = get_uptime()
        ip = get_ip()
        wifi = get_wifi()
        eth = get_eth()
        rx_rate, tx_rate = get_network_rates("wlan0")
        total_rate = rx_rate + tx_rate
        laas = get_laas_status()
        laas_ram = get_laas_ram_mib()

        # top cards
        cpu_text = "..." if cpu is None else f"{cpu:.1f}%"
        self.metric_vars["cpu"][0].set(cpu_text)
        self.metric_vars["ram"][0].set(f"{mem['ram_pct']:.1f}%")
        self.metric_vars["temp"][0].set("N/A" if temp is None else f"{temp:.1f} °C")
        self.metric_vars["disk"][0].set(f"{disk['pct']:.1f}%")
        self.metric_vars["net"][0].set(format_rate(total_rate))

        # details
        self.detail_vars["load"].set(get_loadavg())
        self.detail_vars["clock"].set(clock)
        self.detail_vars["throttled"].set(throttled_text)
        self.detail_vars["uptime"].set(uptime)
        self.detail_vars["ip"].set(ip)
        self.detail_vars["wifi"].set(wifi)
        self.detail_vars["eth"].set(eth)
        self.detail_vars["swap"].set(f"{format_bytes(mem['swap_used'])} / {format_bytes(mem['swap_total'])}")
        self.detail_vars["disk_text"].set(disk["text"])

        self.laas_label.config(text=laas, fg=GREEN if laas != "Không chạy" else MUTED)

        # charts
        if cpu is not None:
            self.chart_widgets["cpu"].push(cpu, f"{cpu:.1f}%")
        if temp is not None:
            self.chart_widgets["temp"].push(temp, f"{temp:.1f} °C")
        self.chart_widgets["ram"].push(mem["ram_pct"], f"{mem['ram_pct']:.1f}%")
        self.chart_widgets["net"].push(total_rate / 1024.0, f"RX {format_rate(rx_rate)} | TX {format_rate(tx_rate)}")

        # recolor throttled detail
        # access via children walk
        for widget in self.children.values():
            pass

        # LAAS RAM history
        self.laas_ram_chart.push(laas_ram)

        # process list
        self.set_proc_text(get_top_processes())

        self.after(UPDATE_MS, self.refresh)


if __name__ == "__main__":
    app = Dashboard()
    app.mainloop()