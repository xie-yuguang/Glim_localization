#!/usr/bin/env python3

import csv
import json
import math
import os
from collections import defaultdict
from datetime import datetime
from html import escape
from pathlib import Path
from typing import Dict, List, Optional

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-glim-localization")

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "matplotlib is required for plot/report generation. "
        "Install it with: sudo apt install python3-matplotlib"
    ) from exc


TIME_KEY_MAP = {
    "User time (seconds)": "user_time_s",
    "System time (seconds)": "sys_time_s",
    "Percent of CPU this job got": "cpu_percent_time",
    "Maximum resident set size (kbytes)": "max_rss_kb_time",
    "Major (requiring I/O) page faults": "major_page_faults",
    "Minor (reclaiming a frame) page faults": "minor_page_faults",
    "Voluntary context switches": "voluntary_context_switches",
    "Involuntary context switches": "involuntary_context_switches",
    "File system inputs": "fs_inputs",
    "File system outputs": "fs_outputs",
    "Exit status": "exit_status_time",
}


def parse_number(text: str):
    text = (text or "").strip()
    if not text:
        return None

    try:
        if "." in text:
            return float(text)
        return int(text)
    except ValueError:
        return None


def parse_elapsed_to_seconds(value: str):
    value = value.strip()
    if not value:
        return None

    days = 0
    if "-" in value:
        day_part, value = value.split("-", 1)
        days = int(day_part)

    parts = value.split(":")
    if len(parts) == 3:
        hours, minutes, seconds = parts
    elif len(parts) == 2:
        hours = 0
        minutes, seconds = parts
    else:
        return None

    return days * 86400 + int(hours) * 3600 + int(minutes) * 60 + float(seconds)


def parse_iso_epoch(timestamp_iso: str):
    if not timestamp_iso:
        return None
    try:
        return datetime.fromisoformat(timestamp_iso).timestamp()
    except ValueError:
        return None


def mean(values: List[float]):
    if not values:
        return None
    return sum(values) / len(values)


def percentile(values: List[float], p: float):
    if not values:
        return None

    if p <= 0:
        return min(values)
    if p >= 100:
        return max(values)

    sorted_values = sorted(values)
    if len(sorted_values) == 1:
        return sorted_values[0]

    rank = (len(sorted_values) - 1) * (p / 100.0)
    low = math.floor(rank)
    high = math.ceil(rank)
    if low == high:
        return sorted_values[low]

    weight = rank - low
    return sorted_values[low] * (1.0 - weight) + sorted_values[high] * weight


def round_up(value: float, step: int):
    if value is None:
        return None
    return int(math.ceil(value / step) * step)


def load_text(path: Path):
    if not path.exists():
        return ""
    return path.read_text().strip()


def load_time_metrics(path: Path):
    if not path.exists():
        return {}

    metrics = {}
    for line in path.read_text().splitlines():
        if ": " not in line:
            continue
        key, value = line.rsplit(": ", 1)
        key = key.strip()
        value = value.strip()
        if key == "Elapsed (wall clock) time (h:mm:ss or m:ss)":
            metrics["wall_time_s"] = parse_elapsed_to_seconds(value)
        elif key in TIME_KEY_MAP:
            if key == "Percent of CPU this job got":
                metrics[TIME_KEY_MAP[key]] = parse_number(value.rstrip("%"))
            else:
                metrics[TIME_KEY_MAP[key]] = parse_number(value)
    return metrics


def load_ps_samples(path: Path):
    if not path.exists():
        return []

    rows = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        prev = None
        for index, row in enumerate(reader):
            epoch_s = parse_number(row.get("epoch_s"))
            if epoch_s is None:
                epoch_s = index

            sample = {
                "timestamp_iso": row.get("timestamp_iso", ""),
                "epoch_s": float(epoch_s),
                "pid": parse_number(row.get("pid")),
                "pcpu": parse_number(row.get("pcpu")),
                "pmem": parse_number(row.get("pmem")),
                "rss_kb": parse_number(row.get("rss_kb")),
                "rss_mb": (parse_number(row.get("rss_kb")) or 0) / 1024.0 if parse_number(row.get("rss_kb")) is not None else None,
                "vsz_kb": parse_number(row.get("vsz_kb")),
                "nlwp": parse_number(row.get("nlwp")),
                "etimes_s": parse_number(row.get("etimes_s")),
                "stat": row.get("stat", ""),
                "read_bytes": parse_number(row.get("read_bytes")),
                "write_bytes": parse_number(row.get("write_bytes")),
                "rchar": parse_number(row.get("rchar")),
                "wchar": parse_number(row.get("wchar")),
                "read_rate_mb_s": None,
                "write_rate_mb_s": None,
            }

            if prev is not None:
                dt = sample["epoch_s"] - prev["epoch_s"]
                if dt > 0:
                    if sample["read_bytes"] is not None and prev["read_bytes"] is not None:
                        sample["read_rate_mb_s"] = max(0.0, (sample["read_bytes"] - prev["read_bytes"]) / dt / (1024.0 * 1024.0))
                    if sample["write_bytes"] is not None and prev["write_bytes"] is not None:
                        sample["write_rate_mb_s"] = max(0.0, (sample["write_bytes"] - prev["write_bytes"]) / dt / (1024.0 * 1024.0))

            rows.append(sample)
            prev = sample

    return rows


def load_gpu_device_samples(path: Path):
    if not path.exists():
        return []

    rows = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            timestamp_iso = row.get("timestamp_iso", "")
            rows.append({
                "timestamp_iso": timestamp_iso,
                "epoch_s": parse_iso_epoch(timestamp_iso),
                "gpu_index": row.get("gpu_index", "").strip(),
                "name": row.get("name", "").strip(),
                "utilization_gpu_pct": parse_number(row.get("utilization_gpu_pct")),
                "utilization_mem_pct": parse_number(row.get("utilization_mem_pct")),
                "memory_used_mb": parse_number(row.get("memory_used_mb")),
                "memory_total_mb": parse_number(row.get("memory_total_mb")),
                "power_draw_w": parse_number(row.get("power_draw_w")),
            })
    return rows


def load_gpu_app_samples(path: Path):
    if not path.exists():
        return []

    rows = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            timestamp_iso = row.get("timestamp_iso", "")
            rows.append({
                "timestamp_iso": timestamp_iso,
                "epoch_s": parse_iso_epoch(timestamp_iso),
                "gpu_uuid": row.get("gpu_uuid", "").strip(),
                "pid": parse_number(row.get("pid")),
                "process_name": row.get("process_name", "").strip(),
                "used_gpu_memory_mb": parse_number(row.get("used_gpu_memory_mb")),
            })
    return rows


def get_global_start_epoch(ps_samples, gpu_device_samples, gpu_app_samples):
    epochs = []
    for collection in (ps_samples, gpu_device_samples, gpu_app_samples):
        for row in collection:
            if row.get("epoch_s") is not None:
                epochs.append(float(row["epoch_s"]))
    return min(epochs) if epochs else 0.0


def add_relative_time(rows, start_epoch):
    for row in rows:
        epoch_s = row.get("epoch_s")
        row["time_s"] = float(epoch_s) - start_epoch if epoch_s is not None else None


def aggregate_gpu_device_by_time(gpu_device_samples):
    grouped = defaultdict(list)
    for row in gpu_device_samples:
        grouped[row["timestamp_iso"]].append(row)

    aggregated = []
    for timestamp_iso, rows in sorted(grouped.items(), key=lambda item: parse_iso_epoch(item[0]) or 0):
        epoch_s = rows[0].get("epoch_s")
        util_values = [r["utilization_gpu_pct"] for r in rows if r.get("utilization_gpu_pct") is not None]
        mem_values = [r["memory_used_mb"] for r in rows if r.get("memory_used_mb") is not None]
        aggregated.append({
            "timestamp_iso": timestamp_iso,
            "epoch_s": epoch_s,
            "time_s": rows[0].get("time_s"),
            "utilization_gpu_pct_max": max(util_values) if util_values else None,
            "utilization_gpu_pct_avg": mean(util_values),
            "memory_used_mb_total": sum(mem_values) if mem_values else None,
        })
    return aggregated


def aggregate_gpu_app_by_time(gpu_app_samples):
    grouped = defaultdict(list)
    for row in gpu_app_samples:
        grouped[row["timestamp_iso"]].append(row)

    aggregated = []
    for timestamp_iso, rows in sorted(grouped.items(), key=lambda item: parse_iso_epoch(item[0]) or 0):
        epoch_s = rows[0].get("epoch_s")
        mem_values = [r["used_gpu_memory_mb"] for r in rows if r.get("used_gpu_memory_mb") is not None]
        aggregated.append({
            "timestamp_iso": timestamp_iso,
            "epoch_s": epoch_s,
            "time_s": rows[0].get("time_s"),
            "used_gpu_memory_mb_total": sum(mem_values) if mem_values else None,
        })
    return aggregated


def compute_summary(analysis):
    ps_samples = analysis["ps_samples"]
    gpu_device_samples = analysis["gpu_device_samples"]
    gpu_app_samples = analysis["gpu_app_samples"]
    time_metrics = analysis["time_metrics"]

    cpu_values = [row["pcpu"] for row in ps_samples if row.get("pcpu") is not None]
    rss_values = [row["rss_mb"] for row in ps_samples if row.get("rss_mb") is not None]
    thread_values = [row["nlwp"] for row in ps_samples if row.get("nlwp") is not None]
    read_rate_values = [row["read_rate_mb_s"] for row in ps_samples if row.get("read_rate_mb_s") is not None]
    write_rate_values = [row["write_rate_mb_s"] for row in ps_samples if row.get("write_rate_mb_s") is not None]

    gpu_device_by_time = aggregate_gpu_device_by_time(gpu_device_samples)
    gpu_app_by_time = aggregate_gpu_app_by_time(gpu_app_samples)

    gpu_util_values = [row["utilization_gpu_pct_max"] for row in gpu_device_by_time if row.get("utilization_gpu_pct_max") is not None]
    gpu_total_mem_values = [row["memory_used_mb_total"] for row in gpu_device_by_time if row.get("memory_used_mb_total") is not None]
    gpu_process_mem_values = [row["used_gpu_memory_mb_total"] for row in gpu_app_by_time if row.get("used_gpu_memory_mb_total") is not None]

    all_times = []
    for collection in (ps_samples, gpu_device_by_time, gpu_app_by_time):
        for row in collection:
            if row.get("time_s") is not None:
                all_times.append(float(row["time_s"]))

    max_rss_mb_time = None
    if time_metrics.get("max_rss_kb_time") is not None:
        max_rss_mb_time = float(time_metrics["max_rss_kb_time"]) / 1024.0

    summary = {
        "command": analysis["command"],
        "exit_code": analysis["exit_code"],
        "sampling_start_iso": analysis["sampling_start_iso"],
        "sampling_end_iso": analysis["sampling_end_iso"],
        "sampling_duration_s": (max(all_times) if all_times else None),
        "wall_time_s": time_metrics.get("wall_time_s"),
        "user_time_s": time_metrics.get("user_time_s"),
        "sys_time_s": time_metrics.get("sys_time_s"),
        "cpu_percent_time": time_metrics.get("cpu_percent_time"),
        "avg_cpu_pct": mean(cpu_values),
        "peak_cpu_pct": max(cpu_values) if cpu_values else None,
        "p95_cpu_pct": percentile(cpu_values, 95.0),
        "avg_rss_mb": mean(rss_values),
        "peak_rss_mb": max(rss_values) if rss_values else None,
        "p95_rss_mb": percentile(rss_values, 95.0),
        "max_rss_mb_time": max_rss_mb_time,
        "avg_threads": mean(thread_values),
        "peak_threads": max(thread_values) if thread_values else None,
        "p95_threads": percentile(thread_values, 95.0),
        "avg_read_rate_mb_s": mean(read_rate_values),
        "peak_read_rate_mb_s": max(read_rate_values) if read_rate_values else None,
        "avg_write_rate_mb_s": mean(write_rate_values),
        "peak_write_rate_mb_s": max(write_rate_values) if write_rate_values else None,
        "avg_gpu_util_pct": mean(gpu_util_values),
        "peak_gpu_util_pct": max(gpu_util_values) if gpu_util_values else None,
        "p95_gpu_util_pct": percentile(gpu_util_values, 95.0),
        "avg_gpu_total_mem_mb": mean(gpu_total_mem_values),
        "peak_gpu_total_mem_mb": max(gpu_total_mem_values) if gpu_total_mem_values else None,
        "avg_process_gpu_mem_mb": mean(gpu_process_mem_values),
        "peak_process_gpu_mem_mb": max(gpu_process_mem_values) if gpu_process_mem_values else None,
        "input_files": analysis["input_files"],
    }

    return summary


def metric_definitions():
    return [
        ("sampling_start_iso", "sampling_start_iso", "", "First sampled timestamp"),
        ("sampling_end_iso", "sampling_end_iso", "", "Last sampled timestamp"),
        ("sampling_duration_s", "sampling_duration_s", "s", "Observed sampling duration"),
        ("wall_time_s", "wall_time_s", "s", "Wall clock runtime from /usr/bin/time -v"),
        ("user_time_s", "user_time_s", "s", "User CPU time"),
        ("sys_time_s", "sys_time_s", "s", "System CPU time"),
        ("cpu_percent_time", "cpu_percent_time", "%", "Overall CPU percentage from /usr/bin/time -v"),
        ("avg_cpu_pct", "avg_cpu_pct", "%", "Average CPU usage from ps samples"),
        ("peak_cpu_pct", "peak_cpu_pct", "%", "Peak CPU usage from ps samples"),
        ("p95_cpu_pct", "p95_cpu_pct", "%", "P95 CPU usage from ps samples"),
        ("avg_rss_mb", "avg_rss_mb", "MiB", "Average resident memory size"),
        ("peak_rss_mb", "peak_rss_mb", "MiB", "Peak resident memory size"),
        ("p95_rss_mb", "p95_rss_mb", "MiB", "P95 resident memory size"),
        ("max_rss_mb_time", "max_rss_mb_time", "MiB", "Maximum RSS from /usr/bin/time -v"),
        ("avg_threads", "avg_threads", "", "Average thread count"),
        ("peak_threads", "peak_threads", "", "Peak thread count"),
        ("p95_threads", "p95_threads", "", "P95 thread count"),
        ("avg_read_rate_mb_s", "avg_read_rate_mb_s", "MiB/s", "Average read throughput"),
        ("peak_read_rate_mb_s", "peak_read_rate_mb_s", "MiB/s", "Peak read throughput"),
        ("avg_write_rate_mb_s", "avg_write_rate_mb_s", "MiB/s", "Average write throughput"),
        ("peak_write_rate_mb_s", "peak_write_rate_mb_s", "MiB/s", "Peak write throughput"),
        ("avg_gpu_util_pct", "avg_gpu_util_pct", "%", "Average max-per-sample GPU utilization"),
        ("peak_gpu_util_pct", "peak_gpu_util_pct", "%", "Peak max-per-sample GPU utilization"),
        ("p95_gpu_util_pct", "p95_gpu_util_pct", "%", "P95 max-per-sample GPU utilization"),
        ("avg_gpu_total_mem_mb", "avg_gpu_total_mem_mb", "MiB", "Average total GPU memory used across sampled devices"),
        ("peak_gpu_total_mem_mb", "peak_gpu_total_mem_mb", "MiB", "Peak total GPU memory used across sampled devices"),
        ("avg_process_gpu_mem_mb", "avg_process_gpu_mem_mb", "MiB", "Average target-process GPU memory"),
        ("peak_process_gpu_mem_mb", "peak_process_gpu_mem_mb", "MiB", "Peak target-process GPU memory"),
    ]


def format_metric_value(value):
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.3f}"
    if isinstance(value, list):
        return ";".join(str(v) for v in value)
    return str(value)


def write_summary_files(summary: Dict, output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_json_path = output_dir / "resource_summary.json"
    summary_json_path.write_text(json.dumps(summary, indent=2, sort_keys=True))

    summary_csv_path = output_dir / "resource_summary.csv"
    with summary_csv_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["metric", "value", "unit", "description"])
        for key, summary_key, unit, description in metric_definitions():
            writer.writerow([key, format_metric_value(summary.get(summary_key)), unit, description])
        writer.writerow(["input_files", format_metric_value(summary.get("input_files")), "", "Detected input files in the monitoring directory"])
        writer.writerow(["command", format_metric_value(summary.get("command")), "", "Original monitored command"])
        writer.writerow(["exit_code", format_metric_value(summary.get("exit_code")), "", "Process exit code"])

    return {"summary_json": summary_json_path.name, "summary_csv": summary_csv_path.name}


def collect_input_files(input_dir: Path):
    known_files = [
        "command.txt",
        "run_meta.txt",
        "time.txt",
        "command.stdout.log",
        "command.stderr.log",
        "ps_samples.csv",
        "summary.txt",
        "summary.json",
        "pidstat_cpu_mem_io.log",
        "pidstat_threads.log",
        "iostat.log",
        "gpu_gpus.csv",
        "gpu_apps.csv",
    ]
    return [name for name in known_files if (input_dir / name).exists()]


def build_analysis(input_dir: Path):
    ps_samples = load_ps_samples(input_dir / "ps_samples.csv")
    gpu_device_samples = load_gpu_device_samples(input_dir / "gpu_gpus.csv")
    gpu_app_samples = load_gpu_app_samples(input_dir / "gpu_apps.csv")

    start_epoch = get_global_start_epoch(ps_samples, gpu_device_samples, gpu_app_samples)
    add_relative_time(ps_samples, start_epoch)
    add_relative_time(gpu_device_samples, start_epoch)
    add_relative_time(gpu_app_samples, start_epoch)

    all_timestamps = []
    for collection in (ps_samples, gpu_device_samples, gpu_app_samples):
        for row in collection:
            if row.get("timestamp_iso"):
                all_timestamps.append(row["timestamp_iso"])

    return {
        "input_dir": input_dir,
        "command": load_text(input_dir / "command.txt"),
        "exit_code": parse_number(load_text(input_dir / "exit_code.txt")),
        "time_metrics": load_time_metrics(input_dir / "time.txt"),
        "ps_samples": ps_samples,
        "gpu_device_samples": gpu_device_samples,
        "gpu_app_samples": gpu_app_samples,
        "sampling_start_iso": min(all_timestamps, key=lambda x: parse_iso_epoch(x) or 0.0) if all_timestamps else "",
        "sampling_end_iso": max(all_timestamps, key=lambda x: parse_iso_epoch(x) or 0.0) if all_timestamps else "",
        "input_files": collect_input_files(input_dir),
    }


def annotate_peak(ax, times, values, color, unit):
    if not times or not values:
        return
    peak_index = max(range(len(values)), key=lambda idx: values[idx])
    peak_x = times[peak_index]
    peak_y = values[peak_index]
    ax.scatter([peak_x], [peak_y], color=color, s=36, zorder=4)
    ax.annotate(
        f"peak {peak_y:.1f} {unit}".rstrip(),
        xy=(peak_x, peak_y),
        xytext=(8, 10),
        textcoords="offset points",
        color=color,
        fontsize=9,
        bbox={"boxstyle": "round,pad=0.25", "fc": "white", "ec": color, "alpha": 0.85},
    )


def prepare_axes(title, ylabel):
    fig, ax = plt.subplots(figsize=(11, 4.8))
    ax.set_title(title)
    ax.set_xlabel("Time Since Start (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
    return fig, ax


def save_figure(fig, path: Path, dpi: int):
    fig.tight_layout()
    fig.savefig(path, dpi=dpi)
    plt.close(fig)


def plot_series_with_stats(path: Path, plot_title: str, ylabel: str, times: List[float], values: List[float], series_label: str, color: str, dpi: int):
    if not times or not values:
        return None

    avg_value = mean(values)
    p95_value = percentile(values, 95.0)

    fig, ax = prepare_axes(plot_title, ylabel)
    ax.plot(times, values, color=color, linewidth=2.0, label=series_label)
    if avg_value is not None:
        ax.axhline(avg_value, color="#2a9d8f", linestyle="--", linewidth=1.2, label=f"avg {avg_value:.1f}")
    if p95_value is not None:
        ax.axhline(p95_value, color="#e9c46a", linestyle=":", linewidth=1.4, label=f"p95 {p95_value:.1f}")

    unit = ylabel.split("(")[-1].rstrip(")") if "(" in ylabel else ""
    annotate_peak(ax, times, values, color, unit)
    ax.legend(loc="upper right")
    save_figure(fig, path, dpi)
    return path.name


def plot_cpu_usage(ps_samples, output_dir: Path, dpi: int):
    times = [row["time_s"] for row in ps_samples if row.get("time_s") is not None and row.get("pcpu") is not None]
    values = [row["pcpu"] for row in ps_samples if row.get("time_s") is not None and row.get("pcpu") is not None]
    return plot_series_with_stats(output_dir / "cpu_usage.png", "CPU Usage Over Time", "CPU (%)", times, values, "process CPU", "#264653", dpi)


def plot_memory_usage(ps_samples, output_dir: Path, dpi: int):
    times = [row["time_s"] for row in ps_samples if row.get("time_s") is not None and row.get("rss_mb") is not None]
    values = [row["rss_mb"] for row in ps_samples if row.get("time_s") is not None and row.get("rss_mb") is not None]
    return plot_series_with_stats(output_dir / "memory_usage.png", "Resident Memory Over Time", "RSS (MiB)", times, values, "RSS", "#e76f51", dpi)


def plot_thread_usage(ps_samples, output_dir: Path, dpi: int):
    times = [row["time_s"] for row in ps_samples if row.get("time_s") is not None and row.get("nlwp") is not None]
    values = [row["nlwp"] for row in ps_samples if row.get("time_s") is not None and row.get("nlwp") is not None]
    return plot_series_with_stats(output_dir / "thread_usage.png", "Thread Count Over Time", "Threads", times, values, "threads", "#457b9d", dpi)


def plot_io_usage(ps_samples, output_dir: Path, dpi: int):
    times = [row["time_s"] for row in ps_samples if row.get("time_s") is not None and (row.get("read_rate_mb_s") is not None or row.get("write_rate_mb_s") is not None)]
    read_values = [row.get("read_rate_mb_s") or 0.0 for row in ps_samples if row.get("time_s") is not None and (row.get("read_rate_mb_s") is not None or row.get("write_rate_mb_s") is not None)]
    write_values = [row.get("write_rate_mb_s") or 0.0 for row in ps_samples if row.get("time_s") is not None and (row.get("read_rate_mb_s") is not None or row.get("write_rate_mb_s") is not None)]

    if not times:
        return None

    fig, ax = prepare_axes("Disk I/O Throughput Over Time", "Throughput (MiB/s)")
    ax.plot(times, read_values, color="#2a9d8f", linewidth=1.8, label="read rate")
    ax.plot(times, write_values, color="#f4a261", linewidth=1.8, label="write rate")
    annotate_peak(ax, times, read_values, "#2a9d8f", "MiB/s")
    annotate_peak(ax, times, write_values, "#f4a261", "MiB/s")
    ax.legend(loc="upper right")
    save_figure(fig, output_dir / "io_usage.png", dpi)
    return "io_usage.png"


def plot_gpu_usage(gpu_device_samples, output_dir: Path, dpi: int):
    grouped = defaultdict(list)
    for row in gpu_device_samples:
        if row.get("time_s") is None or row.get("utilization_gpu_pct") is None:
            continue
        grouped[row["gpu_index"]].append(row)

    if not grouped:
        return None

    fig, ax = prepare_axes("GPU Utilization Over Time", "GPU Utilization (%)")
    colors = ["#6a4c93", "#1982c4", "#8ac926", "#ff595e"]
    for color_index, gpu_index in enumerate(sorted(grouped.keys())):
        rows = grouped[gpu_index]
        times = [row["time_s"] for row in rows]
        values = [row["utilization_gpu_pct"] for row in rows]
        color = colors[color_index % len(colors)]
        ax.plot(times, values, linewidth=1.8, label=f"GPU {gpu_index}", color=color)

    aggregated = aggregate_gpu_device_by_time(gpu_device_samples)
    agg_times = [row["time_s"] for row in aggregated if row.get("time_s") is not None and row.get("utilization_gpu_pct_max") is not None]
    agg_values = [row["utilization_gpu_pct_max"] for row in aggregated if row.get("time_s") is not None and row.get("utilization_gpu_pct_max") is not None]
    if agg_times:
        ax.plot(agg_times, agg_values, color="#111111", linestyle="--", linewidth=1.6, label="max across GPUs")
        annotate_peak(ax, agg_times, agg_values, "#111111", "%")

    ax.legend(loc="upper right")
    save_figure(fig, output_dir / "gpu_usage.png", dpi)
    return "gpu_usage.png"


def plot_gpu_memory_usage(gpu_device_samples, gpu_app_samples, output_dir: Path, dpi: int):
    grouped = defaultdict(list)
    for row in gpu_device_samples:
        if row.get("time_s") is None or row.get("memory_used_mb") is None:
            continue
        grouped[row["gpu_index"]].append(row)

    process_mem = aggregate_gpu_app_by_time(gpu_app_samples)

    if not grouped and not process_mem:
        return None

    fig, ax = prepare_axes("GPU Memory Over Time", "GPU Memory (MiB)")
    colors = ["#8338ec", "#3a86ff", "#ff006e", "#fb5607"]
    for color_index, gpu_index in enumerate(sorted(grouped.keys())):
        rows = grouped[gpu_index]
        times = [row["time_s"] for row in rows]
        values = [row["memory_used_mb"] for row in rows]
        color = colors[color_index % len(colors)]
        ax.plot(times, values, linewidth=1.8, label=f"GPU {gpu_index} total used", color=color)

    if process_mem:
        times = [row["time_s"] for row in process_mem if row.get("time_s") is not None and row.get("used_gpu_memory_mb_total") is not None]
        values = [row["used_gpu_memory_mb_total"] for row in process_mem if row.get("time_s") is not None and row.get("used_gpu_memory_mb_total") is not None]
        if times:
            ax.plot(times, values, color="#111111", linestyle="--", linewidth=1.8, label="target process GPU memory")
            annotate_peak(ax, times, values, "#111111", "MiB")

    ax.legend(loc="upper right")
    save_figure(fig, output_dir / "gpu_memory_usage.png", dpi)
    return "gpu_memory_usage.png"


def generate_plots(analysis, output_dir: Path, title: str, enable_gpu: bool = True, enable_io: bool = True, dpi: int = 150):
    output_dir.mkdir(parents=True, exist_ok=True)

    plot_paths = {}
    ps_samples = analysis["ps_samples"]
    gpu_device_samples = analysis["gpu_device_samples"]
    gpu_app_samples = analysis["gpu_app_samples"]

    plot_paths["cpu_usage"] = plot_cpu_usage(ps_samples, output_dir, dpi)
    plot_paths["memory_usage"] = plot_memory_usage(ps_samples, output_dir, dpi)
    plot_paths["thread_usage"] = plot_thread_usage(ps_samples, output_dir, dpi)

    if enable_io:
        plot_paths["io_usage"] = plot_io_usage(ps_samples, output_dir, dpi)

    if enable_gpu:
        plot_paths["gpu_usage"] = plot_gpu_usage(gpu_device_samples, output_dir, dpi)
        plot_paths["gpu_memory_usage"] = plot_gpu_memory_usage(gpu_device_samples, gpu_app_samples, output_dir, dpi)

    manifest_path = output_dir / "plot_manifest.json"
    manifest_path.write_text(json.dumps(plot_paths, indent=2, sort_keys=True))
    return plot_paths


def portability_notes(summary: Dict):
    notes = []

    peak_cpu_pct = summary.get("peak_cpu_pct")
    p95_cpu_pct = summary.get("p95_cpu_pct")
    if peak_cpu_pct is not None:
        peak_cores = peak_cpu_pct / 100.0
        cpu_budget = max(2, math.ceil(max(peak_cpu_pct, p95_cpu_pct or 0.0) / 100.0 * 1.25))
        notes.append(
            f"CPU-only 估算：观测到的峰值 CPU 约等于 {peak_cores:.1f} 个逻辑核，"
            f"移植时建议至少预留 {cpu_budget} 个逻辑核给 localization 主进程，并额外留出 ROS2 / 驱动余量。"
        )

    peak_rss_mb = summary.get("peak_rss_mb") or summary.get("max_rss_mb_time")
    if peak_rss_mb is not None:
        mem_budget = max(1024, round_up(peak_rss_mb * 1.5, 256))
        notes.append(
            f"内存估算：本次运行峰值 RSS 约 {peak_rss_mb:.1f} MiB，"
            f"移植时建议至少保证 {mem_budget} MiB 以上的可用内存余量。"
        )

    peak_gpu_util_pct = summary.get("peak_gpu_util_pct")
    peak_gpu_mem_mb = summary.get("peak_process_gpu_mem_mb") or summary.get("peak_gpu_total_mem_mb")
    if peak_gpu_util_pct is not None or peak_gpu_mem_mb is not None:
        if peak_gpu_mem_mb is not None:
            gpu_budget = max(1024, round_up(peak_gpu_mem_mb * 1.5, 256))
            notes.append(
                f"GPU 估算：目标进程峰值显存约 {peak_gpu_mem_mb:.1f} MiB，"
                f"如果继续使用 `gpu_vgicp`，建议设备至少预留 {gpu_budget} MiB 可用显存。"
            )
        if peak_gpu_util_pct is not None and peak_gpu_util_pct >= 80.0:
            notes.append("GPU 峰值利用率已超过 80%，配准阶段可能逼近 GPU 计算瓶颈，移植时优先关注 CUDA 核心能力和显存带宽。")
    else:
        notes.append("本次输入中未发现 GPU 采样日志，可按 CPU-only 路线评估；如果后续切到 `gpu_vgicp`，建议重新采一轮 GPU 日志。")

    peak_read_rate = summary.get("peak_read_rate_mb_s")
    peak_write_rate = summary.get("peak_write_rate_mb_s")
    if peak_read_rate is not None and peak_read_rate > 20.0:
        notes.append("磁盘读速率出现明显峰值，地图加载或 rosbag 解码阶段可能受 I/O 影响，移植时建议使用 SSD。")
    if peak_write_rate is not None and peak_write_rate > 10.0:
        notes.append("磁盘写速率存在峰值，若同时开启较多日志或轨迹输出，低速存储可能放大时延波动。")

    if not notes:
        notes.append("当前日志不足以给出可靠的移植建议，建议至少保留 `time.txt` 和 `ps_samples.csv` 后再生成报告。")

    return notes


def render_metrics_table(summary: Dict):
    rows = []
    for metric_name, summary_key, unit, description in metric_definitions():
        rows.append(
            "<tr>"
            f"<td>{escape(metric_name)}</td>"
            f"<td>{escape(format_metric_value(summary.get(summary_key)))}</td>"
            f"<td>{escape(unit)}</td>"
            f"<td>{escape(description)}</td>"
            "</tr>"
        )
    rows.append(
        "<tr>"
        "<td>input_files</td>"
        f"<td>{escape(format_metric_value(summary.get('input_files')))}</td>"
        "<td></td>"
        "<td>Detected input files</td>"
        "</tr>"
    )
    return "\n".join(rows)


def render_plot_section(title: str, filename: Optional[str], fallback: str):
    if not filename:
        return (
            "<section class='card'>"
            f"<h3>{escape(title)}</h3>"
            f"<p class='muted'>{escape(fallback)}</p>"
            "</section>"
        )
    return (
        "<section class='card'>"
        f"<h3>{escape(title)}</h3>"
        f"<img src='{escape(filename)}' alt='{escape(title)}'>"
        "</section>"
    )


def generate_html_report(analysis, summary: Dict, plot_manifest: Dict, output_dir: Path, title: str):
    notes_html = "\n".join(f"<li>{escape(note)}</li>" for note in portability_notes(summary))
    input_files_html = "\n".join(f"<li>{escape(name)}</li>" for name in summary.get("input_files", []))

    sections = [
        render_plot_section("CPU Usage", plot_manifest.get("cpu_usage"), "缺少 ps 采样，无法绘制 CPU 曲线。"),
        render_plot_section("Memory Usage", plot_manifest.get("memory_usage"), "缺少 ps 采样，无法绘制内存曲线。"),
        render_plot_section("Thread Usage", plot_manifest.get("thread_usage"), "缺少 ps 采样，无法绘制线程曲线。"),
        render_plot_section("I/O Throughput", plot_manifest.get("io_usage"), "缺少连续 I/O 累计值，无法计算读写速率。"),
        render_plot_section("GPU Usage", plot_manifest.get("gpu_usage"), "未检测到 GPU 日志，已跳过 GPU 利用率曲线。"),
        render_plot_section("GPU Memory Usage", plot_manifest.get("gpu_memory_usage"), "未检测到 GPU 显存日志，已跳过 GPU 显存曲线。"),
    ]

    html = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <title>{escape(title)}</title>
  <style>
    body {{
      margin: 0;
      padding: 0;
      font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
      background: #f4f1ea;
      color: #1f2933;
    }}
    .page {{
      max-width: 1240px;
      margin: 0 auto;
      padding: 28px 24px 40px;
    }}
    .hero {{
      background: linear-gradient(135deg, #1d3557, #457b9d);
      color: white;
      border-radius: 18px;
      padding: 24px 28px;
      margin-bottom: 24px;
      box-shadow: 0 12px 30px rgba(29, 53, 87, 0.2);
    }}
    .hero h1 {{
      margin: 0 0 10px;
      font-size: 30px;
    }}
    .meta {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
      gap: 10px 20px;
      font-size: 14px;
    }}
    .card {{
      background: white;
      border-radius: 16px;
      padding: 18px 20px;
      margin-bottom: 18px;
      box-shadow: 0 10px 24px rgba(15, 23, 42, 0.08);
    }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
      gap: 18px;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      font-size: 14px;
    }}
    th, td {{
      padding: 10px 8px;
      border-bottom: 1px solid #e5e7eb;
      text-align: left;
      vertical-align: top;
    }}
    th {{
      color: #3b4a5a;
      background: #f8fafc;
    }}
    img {{
      width: 100%;
      height: auto;
      border-radius: 12px;
      border: 1px solid #e5e7eb;
      background: #ffffff;
    }}
    .muted {{
      color: #5f6c7b;
    }}
    ul {{
      margin: 0;
      padding-left: 18px;
    }}
    code {{
      background: #eef3f7;
      padding: 2px 6px;
      border-radius: 6px;
    }}
  </style>
</head>
<body>
  <div class="page">
    <section class="hero">
      <h1>{escape(title)}</h1>
      <div class="meta">
        <div><strong>Input Dir:</strong> {escape(str(analysis["input_dir"]))}</div>
        <div><strong>Command:</strong> <code>{escape(summary.get("command", ""))}</code></div>
        <div><strong>Exit Code:</strong> {escape(format_metric_value(summary.get("exit_code")))}</div>
        <div><strong>Sampling Range:</strong> {escape(summary.get("sampling_start_iso", ""))} ~ {escape(summary.get("sampling_end_iso", ""))}</div>
      </div>
    </section>

    <section class="card">
      <h2>输入日志</h2>
      <ul>
        {input_files_html}
      </ul>
    </section>

    <section class="card">
      <h2>统计摘要</h2>
      <table>
        <thead>
          <tr><th>Metric</th><th>Value</th><th>Unit</th><th>Description</th></tr>
        </thead>
        <tbody>
          {render_metrics_table(summary)}
        </tbody>
      </table>
    </section>

    <section class="card">
      <h2>移植建议</h2>
      <ul>
        {notes_html}
      </ul>
    </section>

    <div class="grid">
      {' '.join(sections)}
    </div>
  </div>
</body>
</html>
"""

    report_path = output_dir / "resource_report.html"
    report_path.write_text(html)
    return report_path.name
