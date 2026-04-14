#!/usr/bin/env python3

import argparse
import csv
import json
from pathlib import Path


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


def load_exit_code(path: Path):
    if not path.exists():
        return None
    return parse_number(path.read_text().strip())


def load_command(path: Path):
    if not path.exists():
        return ""
    return path.read_text().strip()


def summarize_ps(path: Path):
    if not path.exists():
        return {}

    rows = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    if not rows:
        return {}

    cpu_values = [parse_number(r["pcpu"]) for r in rows if parse_number(r["pcpu"]) is not None]
    rss_values = [parse_number(r["rss_kb"]) for r in rows if parse_number(r["rss_kb"]) is not None]
    thread_values = [parse_number(r["nlwp"]) for r in rows if parse_number(r["nlwp"]) is not None]
    etimes_values = [parse_number(r["etimes_s"]) for r in rows if parse_number(r["etimes_s"]) is not None]
    read_bytes_values = [parse_number(r["read_bytes"]) for r in rows if parse_number(r["read_bytes"]) is not None]
    write_bytes_values = [parse_number(r["write_bytes"]) for r in rows if parse_number(r["write_bytes"]) is not None]
    rchar_values = [parse_number(r["rchar"]) for r in rows if parse_number(r["rchar"]) is not None]
    wchar_values = [parse_number(r["wchar"]) for r in rows if parse_number(r["wchar"]) is not None]

    return {
        "num_samples": len(rows),
        "avg_cpu_pct_ps": sum(cpu_values) / len(cpu_values) if cpu_values else None,
        "peak_cpu_pct_ps": max(cpu_values) if cpu_values else None,
        "avg_rss_mb_ps": (sum(rss_values) / len(rss_values) / 1024.0) if rss_values else None,
        "peak_rss_mb_ps": (max(rss_values) / 1024.0) if rss_values else None,
        "peak_threads": max(thread_values) if thread_values else None,
        "runtime_s_ps": max(etimes_values) if etimes_values else None,
        "read_bytes_delta": (max(read_bytes_values) - min(read_bytes_values)) if len(read_bytes_values) >= 2 else None,
        "write_bytes_delta": (max(write_bytes_values) - min(write_bytes_values)) if len(write_bytes_values) >= 2 else None,
        "rchar_delta": (max(rchar_values) - min(rchar_values)) if len(rchar_values) >= 2 else None,
        "wchar_delta": (max(wchar_values) - min(wchar_values)) if len(wchar_values) >= 2 else None,
    }


def summarize_gpu(gpu_path: Path, app_path: Path):
    summary = {}

    if gpu_path.exists():
        gpu_rows = []
        with gpu_path.open(newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                gpu_rows.append(row)

        util_values = [parse_number(r["utilization_gpu_pct"]) for r in gpu_rows if parse_number(r["utilization_gpu_pct"]) is not None]
        mem_values = [parse_number(r["memory_used_mb"]) for r in gpu_rows if parse_number(r["memory_used_mb"]) is not None]
        power_values = [parse_number(r["power_draw_w"]) for r in gpu_rows if parse_number(r["power_draw_w"]) is not None]

        summary.update({
            "gpu_samples": len(gpu_rows),
            "peak_gpu_util_pct": max(util_values) if util_values else None,
            "avg_gpu_util_pct": (sum(util_values) / len(util_values)) if util_values else None,
            "peak_gpu_mem_mb": max(mem_values) if mem_values else None,
            "peak_gpu_power_w": max(power_values) if power_values else None,
        })

    if app_path.exists():
        app_rows = []
        with app_path.open(newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                app_rows.append(row)

        app_mem_values = [parse_number(r["used_gpu_memory_mb"]) for r in app_rows if parse_number(r["used_gpu_memory_mb"]) is not None]
        summary.update({
            "gpu_process_samples": len(app_rows),
            "peak_process_gpu_mem_mb": max(app_mem_values) if app_mem_values else None,
        })

    return summary


def format_value(value):
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def main():
    parser = argparse.ArgumentParser(description="Summarize glim_localization resource monitoring logs")
    parser.add_argument("--input-dir", required=True, help="Directory created by run_with_time.sh")
    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    summary = {}
    summary["command"] = load_command(input_dir / "command.txt")
    summary["exit_code"] = load_exit_code(input_dir / "exit_code.txt")
    summary.update(load_time_metrics(input_dir / "time.txt"))
    summary.update(summarize_ps(input_dir / "ps_samples.csv"))
    summary.update(summarize_gpu(input_dir / "gpu_gpus.csv", input_dir / "gpu_apps.csv"))

    json_path = input_dir / "summary.json"
    json_path.write_text(json.dumps(summary, indent=2, sort_keys=True))

    max_rss_mb_time = None
    if summary.get("max_rss_kb_time") is not None:
        max_rss_mb_time = summary["max_rss_kb_time"] / 1024.0

    lines = [
        "glim_localization resource summary",
        f"input_dir: {input_dir}",
        f"command: {summary.get('command', '')}",
        f"exit_code: {format_value(summary.get('exit_code'))}",
        f"wall_time_s: {format_value(summary.get('wall_time_s'))}",
        f"user_time_s: {format_value(summary.get('user_time_s'))}",
        f"sys_time_s: {format_value(summary.get('sys_time_s'))}",
        f"cpu_percent_time: {format_value(summary.get('cpu_percent_time'))}",
        f"avg_cpu_pct_ps: {format_value(summary.get('avg_cpu_pct_ps'))}",
        f"peak_cpu_pct_ps: {format_value(summary.get('peak_cpu_pct_ps'))}",
        f"max_rss_mb_time: {format_value(max_rss_mb_time)}",
        f"peak_rss_mb_ps: {format_value(summary.get('peak_rss_mb_ps'))}",
        f"avg_rss_mb_ps: {format_value(summary.get('avg_rss_mb_ps'))}",
        f"peak_threads: {format_value(summary.get('peak_threads'))}",
        f"runtime_s_ps: {format_value(summary.get('runtime_s_ps'))}",
        f"read_bytes_delta: {format_value(summary.get('read_bytes_delta'))}",
        f"write_bytes_delta: {format_value(summary.get('write_bytes_delta'))}",
        f"rchar_delta: {format_value(summary.get('rchar_delta'))}",
        f"wchar_delta: {format_value(summary.get('wchar_delta'))}",
        f"peak_gpu_util_pct: {format_value(summary.get('peak_gpu_util_pct'))}",
        f"avg_gpu_util_pct: {format_value(summary.get('avg_gpu_util_pct'))}",
        f"peak_gpu_mem_mb: {format_value(summary.get('peak_gpu_mem_mb'))}",
        f"peak_process_gpu_mem_mb: {format_value(summary.get('peak_process_gpu_mem_mb'))}",
        f"summary_json: {json_path}",
    ]
    print("\n".join(lines))


if __name__ == "__main__":
    main()
