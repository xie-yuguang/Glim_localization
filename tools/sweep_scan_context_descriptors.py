#!/usr/bin/env python3
"""Run threshold sweeps over relocalization top-k descriptor distances.

This is an offline analysis helper. It reads a glim_localization debug CSV and
does not change runtime relocalization behavior.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from collections import Counter
from statistics import mean
from typing import Dict, Iterable, List, Sequence


def parse_float(value: str) -> float:
    try:
        return float(str(value).strip())
    except (TypeError, ValueError):
        return math.nan


def parse_int(value: str) -> int:
    try:
        return int(float(str(value).strip()))
    except (TypeError, ValueError):
        return -1


def parse_float_list(value: str) -> List[float]:
    return [parse_float(item) for item in str(value or "").replace("|", ";").split(";") if item.strip()]


def parse_int_list(value: str) -> List[int]:
    return [parse_int(item) for item in str(value or "").replace("|", ";").split(";") if item.strip()]


def finite(values: Iterable[float]) -> List[float]:
    return [v for v in values if math.isfinite(v)]


def percentile(values: Sequence[float], pct: float) -> float:
    values = sorted(finite(values))
    if not values:
        return math.nan
    if len(values) == 1:
        return values[0]
    pos = (len(values) - 1) * pct
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return values[lo]
    return values[lo] * (hi - pos) + values[hi] * (pos - lo)


def fmt(value: float, digits: int = 3) -> str:
    return "nan" if not math.isfinite(value) else f"{value:.{digits}f}"


def read_rows(path: str) -> List[Dict[str, str]]:
    with open(path, "r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"empty debug CSV: {path}")
    return rows


def frame(row: Dict[str, str]) -> int:
    return parse_int(row.get("frame_id", ""))


def topk_distances(row: Dict[str, str]) -> List[float]:
    distances = parse_float_list(row.get("relocalization_topk_distances", ""))
    if distances:
        return distances
    return finite([
        parse_float(row.get("relocalization_top1_distance", "")),
        parse_float(row.get("relocalization_top2_distance", "")),
        parse_float(row.get("relocalization_top3_distance", "")),
    ])


def topk_submaps(row: Dict[str, str]) -> List[int]:
    submaps = parse_int_list(row.get("relocalization_topk_submaps", ""))
    if submaps:
        return submaps
    return [v for v in [
        parse_int(row.get("relocalization_top1_submap", "")),
        parse_int(row.get("relocalization_top2_submap", "")),
        parse_int(row.get("relocalization_top3_submap", "")),
    ] if v >= 0]


def write_csv(path: str, fieldnames: Sequence[str], rows: Iterable[Dict[str, object]]) -> None:
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description="Sweep ScanContext relocalization descriptor thresholds from debug CSV")
    parser.add_argument("csv_path")
    parser.add_argument("--out", required=True)
    parser.add_argument("--frame-start", type=int, default=None)
    parser.add_argument("--frame-end", type=int, default=None)
    parser.add_argument("--thresholds", default="0.30,0.35,0.40,0.45,0.50,0.55,0.60,0.70")
    args = parser.parse_args()

    rows = read_rows(args.csv_path)
    query_rows = [
        r for r in rows
        if (args.frame_start is None or frame(r) >= args.frame_start)
        and (args.frame_end is None or frame(r) <= args.frame_end)
        and topk_distances(r)
    ]
    thresholds = [parse_float(v) for v in args.thresholds.split(",") if math.isfinite(parse_float(v))]

    os.makedirs(args.out, exist_ok=True)

    top1 = [topk_distances(r)[0] for r in query_rows if topk_distances(r)]
    submap_counter = Counter()
    topk_rows = []
    for r in query_rows:
        distances = topk_distances(r)
        submaps = topk_submaps(r)
        for submap in submaps[: min(5, len(submaps))]:
            submap_counter[submap] += 1
        topk_rows.append({
            "frame_id": frame(r),
            "state_after": r.get("state_after", ""),
            "topk_submaps": ";".join(str(v) for v in submaps),
            "topk_distances": ";".join(fmt(v, 6) for v in distances),
            "top1_submap": submaps[0] if submaps else "",
            "top1_distance": fmt(distances[0], 6) if distances else "nan",
            "failure_reason": r.get("relocalization_failure_reason", ""),
        })

    threshold_rows = []
    candidate_count_rows = []
    for threshold in thresholds:
        counts = []
        top1_accepted = 0
        submaps_at_threshold = Counter()
        for r in query_rows:
            distances = topk_distances(r)
            submaps = topk_submaps(r)
            count = sum(1 for d in distances if math.isfinite(d) and d <= threshold)
            counts.append(count)
            if distances and distances[0] <= threshold:
                top1_accepted += 1
            for submap, distance in zip(submaps, distances):
                if math.isfinite(distance) and distance <= threshold:
                    submaps_at_threshold[submap] += 1
            candidate_count_rows.append({
                "threshold": fmt(threshold, 2),
                "frame_id": frame(r),
                "candidate_count": count,
                "top1_accepted": 1 if distances and distances[0] <= threshold else 0,
            })
        threshold_rows.append({
            "threshold": fmt(threshold, 2),
            "frames": len(query_rows),
            "frames_with_candidates": sum(1 for c in counts if c > 0),
            "top1_accepted_frames": top1_accepted,
            "mean_candidate_count": fmt(mean(counts)) if counts else "nan",
            "max_candidate_count": max(counts) if counts else 0,
            "top_submaps": ";".join(f"{k}:{v}" for k, v in submaps_at_threshold.most_common(10)),
        })

    write_csv(os.path.join(args.out, "scan_context_topk_distance_summary.csv"), [
        "frame_id", "state_after", "topk_submaps", "topk_distances", "top1_submap", "top1_distance", "failure_reason",
    ], topk_rows)
    write_csv(os.path.join(args.out, "scan_context_threshold_sweep.csv"), [
        "threshold", "frames", "frames_with_candidates", "top1_accepted_frames", "mean_candidate_count", "max_candidate_count", "top_submaps",
    ], threshold_rows)
    write_csv(os.path.join(args.out, "scan_context_candidate_count_by_threshold.csv"), [
        "threshold", "frame_id", "candidate_count", "top1_accepted",
    ], candidate_count_rows)

    with open(os.path.join(args.out, "scan_context_sweep_summary.md"), "w", encoding="utf-8") as f:
        f.write("# ScanContext Descriptor Sweep Summary\n\n")
        f.write(f"- source_csv: `{args.csv_path}`\n")
        f.write(f"- query_rows: {len(query_rows)}\n")
        if top1:
            f.write(
                f"- top1_distance_min/mean/median/p90/max: {fmt(min(top1))}/{fmt(mean(top1))}/"
                f"{fmt(percentile(top1, 0.50))}/{fmt(percentile(top1, 0.90))}/{fmt(max(top1))}\n"
            )
        f.write(f"- common_topk_submaps: {';'.join(f'{k}:{v}' for k, v in submap_counter.most_common(10))}\n\n")
        f.write("| threshold | frames_with_candidates | top1_accepted_frames | mean_candidate_count | max_candidate_count | top_submaps |\n")
        f.write("| ---: | ---: | ---: | ---: | ---: | --- |\n")
        for row in threshold_rows:
            f.write(
                f"| {row['threshold']} | {row['frames_with_candidates']} | {row['top1_accepted_frames']} | "
                f"{row['mean_candidate_count']} | {row['max_candidate_count']} | {row['top_submaps']} |\n"
            )

    print(f"wrote ScanContext descriptor sweep to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
