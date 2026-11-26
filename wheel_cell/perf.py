"""Runtime performance monitor: GPU, memory, CPU and where each physics step spends its time.

Live (play.py) shows the values in a "CN7 | Perf" window and writes them to CSV; the headless
check prints a summary. GPU values come from nvidia-smi on a background thread so a slow query
never stalls the simulation.
"""
import csv
import subprocess
import threading
import time
from collections import defaultdict
from pathlib import Path

import psutil

GPU_QUERY = "utilization.gpu,memory.used,memory.total,temperature.gpu,power.draw"


class GpuSampler(threading.Thread):
    def __init__(self, period=1.):
        super().__init__(daemon=True)
        self.period = period
        self.values = {}
        self.running = True

    def run(self):
        while self.running:
            try:
                line = subprocess.check_output(
                    ["nvidia-smi", f"--query-gpu={GPU_QUERY}", "--format=csv,noheader,nounits"],
                    text=True, timeout=3, stderr=subprocess.DEVNULL).splitlines()[0]
                util, used, total, temp, power = (float(v) if v.strip() not in ("[N/A]", "N/A") else float("nan")
                                                  for v in line.split(","))
                self.values = {"gpu_util_pct": util, "vram_used_mib": used, "vram_total_mib": total,
                               "gpu_temp_c": temp, "gpu_power_w": power}
            except (OSError, subprocess.SubprocessError, ValueError, IndexError):
                self.values = {}
            time.sleep(self.period)


class StepTimer:
    """Accumulates wall time per named section of a physics step."""

    def __init__(self):
        self.total = defaultdict(float)
        self.window = defaultdict(float)
        self.steps = 0
        self.window_steps = 0
        self._open = {}

    def start(self, name):
        self._open[name] = time.perf_counter()

    def stop(self, name):
        began = self._open.pop(name, None)
        if began is not None:
            elapsed = time.perf_counter() - began
            self.total[name] += elapsed
            self.window[name] += elapsed

    def section(self, name):
        timer = self

        class _Section:
            def __enter__(self):
                timer.start(name)

            def __exit__(self, *exc):
                timer.stop(name)
        return _Section()

    def count_step(self):
        self.steps += 1
        self.window_steps += 1

    def window_ms(self):
        n = max(self.window_steps, 1)
        result = {name: 1000 * value / n for name, value in self.window.items()}
        self.window.clear()
        self.window_steps = 0
        return result

    def summary_ms(self):
        n = max(self.steps, 1)
        return {name: round(1000 * value / n, 3) for name, value in sorted(self.total.items())}


class PerfMonitor:
    def __init__(self, csv_path=None, period=.5):
        self.timer = StepTimer()
        self.gpu = GpuSampler()
        self.gpu.start()
        self.process = psutil.Process()
        self.process.cpu_percent(None)
        self.period = period
        self.last_sample = time.perf_counter()
        self.last_sim = 0.
        self.started = time.perf_counter()
        self.latest = {}
        self.peaks = defaultdict(float)
        self.rows = []
        self.csv_path = Path(csv_path) if csv_path else None
        self.frame_start = None
        self.frame_ms = []

    def frame_begin(self):
        self.frame_start = time.perf_counter()

    def frame_end(self):
        if self.frame_start is not None:
            self.frame_ms.append(1000 * (time.perf_counter() - self.frame_start))

    def sample(self, sim_seconds, force=False):
        now = time.perf_counter()
        if not force and now - self.last_sample < self.period:
            return None
        wall = now - self.last_sample
        memory = self.process.memory_info().rss / 2**20
        system = psutil.virtual_memory()
        row = {
            "wall_s": round(now - self.started, 2),
            "sim_s": round(sim_seconds, 3),
            "realtime_factor": round((sim_seconds - self.last_sim) / wall, 3) if wall > 0 else 0.,
            "process_rss_mib": round(memory, 1),
            "system_ram_used_pct": system.percent,
            "process_cpu_pct": round(self.process.cpu_percent(None), 1),
            "frame_ms": round(sum(self.frame_ms) / len(self.frame_ms), 2) if self.frame_ms else float("nan"),
            **self.gpu.values,
            **{f"step_{k}_ms": round(v, 3) for k, v in self.timer.window_ms().items()},
        }
        self.frame_ms.clear()
        self.last_sample = now
        self.last_sim = sim_seconds
        self.latest = row
        for key in ("vram_used_mib", "process_rss_mib", "gpu_util_pct", "gpu_temp_c"):
            if key in row and row[key] == row[key]:
                self.peaks[key] = max(self.peaks[key], row[key])
        self.rows.append(row)
        return row

    def lines(self):
        r = self.latest
        if not r:
            return ["measuring…"]
        steps = "  ".join(f"{k[5:-3]} {v:.2f}" for k, v in r.items() if k.startswith("step_"))
        return [
            f"GPU {r.get('gpu_util_pct', float('nan')):.0f} %   VRAM {r.get('vram_used_mib', 0):.0f} / "
            f"{r.get('vram_total_mib', 0):.0f} MiB   {r.get('gpu_temp_c', float('nan')):.0f} °C   "
            f"{r.get('gpu_power_w', float('nan')):.0f} W",
            f"RAM {r['process_rss_mib']:.0f} MiB (system {r['system_ram_used_pct']:.0f} %)   CPU {r['process_cpu_pct']:.0f} %",
            f"frame {r['frame_ms']:.1f} ms   sim/real ×{r['realtime_factor']:.2f}",
            f"per physics step (ms): {steps}",
        ]

    def summary(self):
        return {"step_ms": self.timer.summary_ms(), "steps": self.timer.steps,
                "peaks": {k: round(v, 1) for k, v in self.peaks.items()},
                "wall_s": round(time.perf_counter() - self.started, 1)}

    def close(self):
        self.gpu.running = False
        if self.csv_path and self.rows:
            self.csv_path.parent.mkdir(parents=True, exist_ok=True)
            keys = sorted({k for row in self.rows for k in row})
            with self.csv_path.open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=keys)
                writer.writeheader()
                writer.writerows(self.rows)


def enable_viewport_hud():
    """Built-in Kit viewport overlay: FPS, GPU memory and host memory."""
    import carb.settings

    settings = carb.settings.get_settings()
    for key in ("/persistent/app/viewport/displayOptions",):
        current = settings.get(key) or 0
        # FPS (1), resolution (1<<3), device memory (1<<12), host memory (1<<13)
        settings.set(key, int(current) | 1 | (1 << 3) | (1 << 12) | (1 << 13))
