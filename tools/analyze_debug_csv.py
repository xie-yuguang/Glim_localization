#!/usr/bin/env python3
"""Analyze glim_localization debug CSV files.

This tool is intentionally dependency-free so it can run on target machines
that only have Python 3 available.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from collections import Counter
from statistics import mean
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


LOST_STATES = {"LOST", "RELOCALIZING"}


def parse_float(value: str) -> float:
    if value is None:
        return math.nan
    value = value.strip()
    if not value:
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def parse_int(value: str) -> int:
    if value is None:
        return 0
    value = value.strip()
    if not value:
        return 0
    try:
        return int(float(value))
    except ValueError:
        return 0


def parse_float_list(value: str) -> List[float]:
    if value is None:
        return []
    values: List[float] = []
    for item in value.replace("|", ";").split(";"):
        item = item.strip()
        if not item:
            continue
        values.append(parse_float(item))
    return values


def parse_int_list(value: str) -> List[int]:
    if value is None:
        return []
    values: List[int] = []
    for item in value.replace("|", ";").split(";"):
        item = item.strip()
        if not item:
            continue
        values.append(parse_int(item))
    return values


def parse_thresholds(value: str) -> List[float]:
    thresholds = [parse_float(v) for v in value.split(",")]
    return [v for v in thresholds if math.isfinite(v)]


def truthy(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


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
    if not math.isfinite(value):
        return "nan"
    return f"{value:.{digits}f}"


def read_rows(path: str) -> List[Dict[str, str]]:
    with open(path, "r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError("CSV has no header")
        rows = list(reader)
    if not rows:
        raise ValueError("CSV has no data rows")
    return rows


def frame(row: Dict[str, str]) -> int:
    return parse_int(row.get("frame_id", "0"))


def state_after(row: Dict[str, str]) -> str:
    return row.get("state_after", "").strip()


def state_before(row: Dict[str, str]) -> str:
    return row.get("state_before", "").strip()


def value(row: Dict[str, str], key: str) -> float:
    return parse_float(row.get(key, ""))


def write_csv(path: str, fieldnames: Sequence[str], rows: Iterable[Dict[str, object]]) -> None:
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def contiguous_segments(rows: Sequence[Dict[str, str]], states: set[str]) -> List[Dict[str, object]]:
    segments: List[Dict[str, object]] = []
    start: Optional[Dict[str, str]] = None
    last: Optional[Dict[str, str]] = None
    for row in rows:
        if state_after(row) in states:
            if start is None:
                start = row
            last = row
        elif start is not None and last is not None:
            segments.append(segment_summary(start, last, rows))
            start = None
            last = None
    if start is not None and last is not None:
        segments.append(segment_summary(start, last, rows))
    return segments


def segment_summary(start: Dict[str, str], end: Dict[str, str], all_rows: Sequence[Dict[str, str]]) -> Dict[str, object]:
    start_frame = frame(start)
    end_frame = frame(end)
    sub_rows = [r for r in all_rows if start_frame <= frame(r) <= end_frame]
    actions = Counter(r.get("smoother_guard_action", "").strip() or "none" for r in sub_rows)
    candidates = [parse_int(r.get("relocalization_candidates", "")) for r in sub_rows if truthy(r.get("relocalization_requested", ""))]
    return {
        "start_frame": start_frame,
        "end_frame": end_frame,
        "frames": end_frame - start_frame + 1,
        "start_state_before": state_before(start),
        "end_state_after": state_after(end),
        "smoother_guard_actions": ";".join(f"{k}:{v}" for k, v in sorted(actions.items())),
        "relocalization_candidate_min": min(candidates) if candidates else "",
        "relocalization_candidate_max": max(candidates) if candidates else "",
    }


def xyz(row: Dict[str, str], prefix: str) -> Tuple[float, float, float]:
    return (value(row, f"{prefix}_x"), value(row, f"{prefix}_y"), value(row, f"{prefix}_z"))


def distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    if not all(math.isfinite(v) for v in (*a, *b)):
        return math.nan
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def topk_distances(row: Dict[str, str]) -> List[float]:
    distances = parse_float_list(row.get("relocalization_topk_distances", ""))
    if distances:
        return distances
    fallback = [
        value(row, "relocalization_top1_distance"),
        value(row, "relocalization_top2_distance"),
        value(row, "relocalization_top3_distance"),
    ]
    return [v for v in fallback if math.isfinite(v)]


def topk_submaps(row: Dict[str, str]) -> List[int]:
    submaps = parse_int_list(row.get("relocalization_topk_submaps", ""))
    if submaps:
        return submaps
    fallback = [
        parse_int(row.get("relocalization_top1_submap", "")),
        parse_int(row.get("relocalization_top2_submap", "")),
        parse_int(row.get("relocalization_top3_submap", "")),
    ]
    return [v for v in fallback if v >= 0]


def analyze(rows: List[Dict[str, str]], high_score: float, large_delta: float, thresholds: Sequence[float]) -> Dict[str, object]:
    states = Counter(state_after(r) or "UNKNOWN" for r in rows)
    transitions = Counter(f"{state_before(r)}->{state_after(r)}" for r in rows if state_before(r) or state_after(r))
    reject_reasons = Counter((r.get("reject_reason", "").strip() or "accepted") for r in rows)
    smoother_actions = Counter((r.get("smoother_guard_action", "").strip() or "none") for r in rows)
    target_actions = Counter((r.get("target_rebuild_guard_action", "").strip() or "none") for r in rows)

    first_degraded = next((frame(r) for r in rows if state_after(r) == "DEGRADED"), None)
    first_lost = next((frame(r) for r in rows if state_after(r) == "LOST"), None)
    first_relocalizing = next((frame(r) for r in rows if state_after(r) == "RELOCALIZING"), None)
    lost_segments = contiguous_segments(rows, LOST_STATES)
    longest_lost = max(lost_segments, key=lambda s: int(s["frames"]), default=None)

    delta_t = [value(r, "delta_t") for r in rows]
    delta_r = [value(r, "delta_r") for r in rows]
    scores = [value(r, "score") for r in rows]
    inliers = [value(r, "inliers") for r in rows]
    inlier_fraction = [value(r, "inlier_fraction") for r in rows]

    high_score_large_delta = [
        r for r in rows
        if value(r, "score") >= high_score and value(r, "delta_t") >= large_delta
    ]

    rebuild_rows = [r for r in rows if truthy(r.get("target_rebuilt", ""))]
    rebuild_events = []
    by_frame = {frame(r): r for r in rows}
    for r in rebuild_rows:
        f = frame(r)
        prev = by_frame.get(f - 1)
        center = xyz(r, "target_center")
        prev_center = xyz(prev, "target_center") if prev else (math.nan, math.nan, math.nan)
        row = {
            "frame_id": f,
            "state_before": state_before(r),
            "state_after": state_after(r),
            "active_submaps_before": prev.get("active_submaps", "") if prev else "",
            "active_submaps_after": r.get("active_submaps", ""),
            "target_center_jump": fmt(distance(prev_center, center)),
            "delta_t": fmt(value(r, "delta_t")),
            "reject_reason": r.get("reject_reason", ""),
        }
        for offset in (1, 2, 3, 5):
            row[f"delta_t_plus_{offset}"] = fmt(value(by_frame[f + offset], "delta_t")) if f + offset in by_frame else ""
            row[f"state_plus_{offset}"] = state_after(by_frame[f + offset]) if f + offset in by_frame else ""
        rebuild_events.append(row)

    relocalization_rows = [
        r for r in rows
        if truthy(r.get("relocalization_requested", "")) or r.get("relocalization_query_reason", "").strip() or r.get("relocalization_failure_reason", "").strip()
    ]
    relocalization_candidates = [parse_int(r.get("relocalization_candidates", "")) for r in relocalization_rows]
    relocalization_success = [r for r in rows if truthy(r.get("relocalization_success", ""))]
    relocalization_failure_reasons = Counter((r.get("relocalization_failure_reason", "").strip() or "none") for r in relocalization_rows)
    relocalization_top1_distances = [value(r, "relocalization_top1_distance") for r in relocalization_rows]
    relocalization_query_points = [value(r, "relocalization_query_points") for r in relocalization_rows]
    relocalization_topk_returned = [parse_int(r.get("relocalization_topk_returned", "")) for r in relocalization_rows]
    debug_verification_rows = [r for r in relocalization_rows if truthy(r.get("debug_verification_only", ""))]
    verify_raw_topk_rows = [r for r in rows if truthy(r.get("verify_raw_topk_used", ""))]
    verify_raw_topk_success = [r for r in verify_raw_topk_rows if truthy(r.get("verify_raw_topk_success", ""))]
    recovering_rows = [r for r in rows if state_after(r) == "RECOVERING" or state_before(r) == "RECOVERING"]
    recovering_to_tracking = [r for r in rows if state_before(r) == "RECOVERING" and state_after(r) == "TRACKING"]
    relocalizing_to_recovering = [r for r in rows if state_before(r) == "RELOCALIZING" and state_after(r) == "RECOVERING"]
    recovery_rejections = [r for r in recovering_rows if (r.get("reject_reason", "").strip())]
    recovering_stable_counts = [parse_int(r.get("recovering_stable_count", "")) for r in rows]

    threshold_rows = []
    for threshold in thresholds:
      frame_count = 0
      top1_accepted = 0
      candidate_counts = []
      submap_counter = Counter()
      for r in relocalization_rows:
          distances = topk_distances(r)
          if not distances:
              continue
          accepted_count = sum(1 for d in distances if math.isfinite(d) and d <= threshold)
          candidate_counts.append(accepted_count)
          if accepted_count > 0:
              frame_count += 1
          if distances and math.isfinite(distances[0]) and distances[0] <= threshold:
              top1_accepted += 1
          for submap, distance_value in zip(topk_submaps(r), distances):
              if math.isfinite(distance_value) and distance_value <= threshold:
                  submap_counter[submap] += 1
      threshold_rows.append({
          "threshold": fmt(threshold, 2),
          "frames_with_candidates": frame_count,
          "top1_accepted_frames": top1_accepted,
          "mean_candidate_count": fmt(mean(candidate_counts)) if candidate_counts else "nan",
          "max_candidate_count": max(candidate_counts) if candidate_counts else 0,
          "top_submaps": ";".join(f"{k}:{v}" for k, v in submap_counter.most_common(8)),
      })

    return {
        "states": states,
        "transitions": transitions,
        "reject_reasons": reject_reasons,
        "smoother_actions": smoother_actions,
        "target_actions": target_actions,
        "first_degraded": first_degraded,
        "first_lost": first_lost,
        "first_relocalizing": first_relocalizing,
        "lost_segments": lost_segments,
        "longest_lost": longest_lost,
        "delta_t": delta_t,
        "delta_r": delta_r,
        "scores": scores,
        "inliers": inliers,
        "inlier_fraction": inlier_fraction,
        "high_score_large_delta": high_score_large_delta,
        "rebuild_events": rebuild_events,
        "relocalization_rows": relocalization_rows,
        "relocalization_candidates": relocalization_candidates,
        "relocalization_success": relocalization_success,
        "relocalization_failure_reasons": relocalization_failure_reasons,
        "relocalization_top1_distances": relocalization_top1_distances,
        "relocalization_query_points": relocalization_query_points,
        "relocalization_topk_returned": relocalization_topk_returned,
        "debug_verification_rows": debug_verification_rows,
        "verify_raw_topk_rows": verify_raw_topk_rows,
        "verify_raw_topk_success": verify_raw_topk_success,
        "recovering_rows": recovering_rows,
        "recovering_to_tracking": recovering_to_tracking,
        "relocalizing_to_recovering": relocalizing_to_recovering,
        "recovery_rejections": recovery_rejections,
        "recovering_stable_counts": recovering_stable_counts,
        "thresholds": thresholds,
        "threshold_rows": threshold_rows,
    }


def stats_row(name: str, values: Sequence[float]) -> Dict[str, object]:
    vals = finite(values)
    return {
        "metric": name,
        "count": len(vals),
        "min": fmt(min(vals)) if vals else "nan",
        "max": fmt(max(vals)) if vals else "nan",
        "mean": fmt(mean(vals)) if vals else "nan",
        "p50": fmt(percentile(vals, 0.50)),
        "p90": fmt(percentile(vals, 0.90)),
        "p95": fmt(percentile(vals, 0.95)),
        "p99": fmt(percentile(vals, 0.99)),
    }


def write_outputs(rows: List[Dict[str, str]], analysis: Dict[str, object], out_dir: str, source_csv: str) -> None:
    os.makedirs(out_dir, exist_ok=True)

    write_csv(
        os.path.join(out_dir, "state_timeline.csv"),
        ["frame_id", "timestamp", "state_before", "state_after", "reject_reason", "smoother_guard_action", "target_rebuild_guard_action"],
        [
            {
                "frame_id": frame(r),
                "timestamp": r.get("timestamp", ""),
                "state_before": state_before(r),
                "state_after": state_after(r),
                "reject_reason": r.get("reject_reason", ""),
                "smoother_guard_action": r.get("smoother_guard_action", ""),
                "target_rebuild_guard_action": r.get("target_rebuild_guard_action", ""),
            }
            for r in rows
        ],
    )

    rebuild_fields = [
        "frame_id", "state_before", "state_after", "active_submaps_before", "active_submaps_after",
        "target_center_jump", "delta_t", "reject_reason",
        "delta_t_plus_1", "state_plus_1", "delta_t_plus_2", "state_plus_2",
        "delta_t_plus_3", "state_plus_3", "delta_t_plus_5", "state_plus_5",
    ]
    write_csv(os.path.join(out_dir, "target_rebuild_events.csv"), rebuild_fields, analysis["rebuild_events"])

    write_csv(
        os.path.join(out_dir, "correction_statistics.csv"),
        ["metric", "count", "min", "max", "mean", "p50", "p90", "p95", "p99"],
        [
            stats_row("score", analysis["scores"]),
            stats_row("delta_t", analysis["delta_t"]),
            stats_row("delta_r", analysis["delta_r"]),
            stats_row("inliers", analysis["inliers"]),
            stats_row("inlier_fraction", analysis["inlier_fraction"]),
        ],
    )

    write_csv(
        os.path.join(out_dir, "lost_segments.csv"),
        [
            "start_frame", "end_frame", "frames", "start_state_before", "end_state_after",
            "smoother_guard_actions", "relocalization_candidate_min", "relocalization_candidate_max",
        ],
        analysis["lost_segments"],
    )

    guard_rows = []
    for r in rows:
        if r.get("smoother_guard_action", "").strip() or r.get("target_rebuild_guard_action", "").strip():
            guard_rows.append({
                "frame_id": frame(r),
                "state_before": state_before(r),
                "state_after": state_after(r),
                "smoother_guard_action": r.get("smoother_guard_action", ""),
                "target_rebuild_guard_action": r.get("target_rebuild_guard_action", ""),
                "confirmation_window_remaining": r.get("confirmation_window_remaining", ""),
                "pending_correction_delta_t": r.get("pending_correction_delta_t", ""),
                "pending_correction_consistent": r.get("pending_correction_consistent", ""),
                "reject_reason": r.get("reject_reason", ""),
            })
    write_csv(
        os.path.join(out_dir, "guard_actions.csv"),
        [
            "frame_id", "state_before", "state_after", "smoother_guard_action", "target_rebuild_guard_action",
            "confirmation_window_remaining", "pending_correction_delta_t", "pending_correction_consistent", "reject_reason",
        ],
        guard_rows,
    )

    relocalization_fields = [
        "frame_id", "state_before", "state_after", "query_enable", "query_reason", "query_points",
        "descriptor_valid", "descriptor_nonempty_bins", "database_size", "topk_requested", "topk_returned",
        "candidates_before_filter", "candidates_after_filter", "filtered_by_descriptor", "filtered_by_translation",
        "filtered_by_other", "verification_attempted", "verification_success", "verification_best_submap",
        "verification_best_score", "verification_best_inliers", "verification_best_residual", "failure_reason",
        "debug_verification_only", "debug_verified_rejected_topk_count", "debug_verified_best_submap",
        "debug_verified_best_descriptor_distance", "debug_verified_best_score", "debug_verified_best_inliers",
        "debug_verified_best_residual", "debug_verified_success", "verify_raw_topk_used",
        "verify_raw_topk_candidate_count", "verify_raw_topk_best_submap", "verify_raw_topk_best_distance",
        "verify_raw_topk_best_score", "verify_raw_topk_best_inliers", "verify_raw_topk_best_residual",
        "verify_raw_topk_success", "recovery_state", "recovering_stable_count",
        "recovering_required_stable_frames", "recovering_transition_reason", "descriptor_passed_main_threshold",
        "descriptor_passed_raw_topk_experiment", "smoother_guard_action",
    ]
    relocalization_summary_rows = []
    for r in analysis["relocalization_rows"]:
        relocalization_summary_rows.append({
            "frame_id": frame(r),
            "state_before": state_before(r),
            "state_after": state_after(r),
            "query_enable": r.get("relocalization_query_enable", ""),
            "query_reason": r.get("relocalization_query_reason", ""),
            "query_points": r.get("relocalization_query_points", ""),
            "descriptor_valid": r.get("relocalization_descriptor_valid", ""),
            "descriptor_nonempty_bins": r.get("relocalization_descriptor_nonempty_bins", ""),
            "database_size": r.get("relocalization_database_size", ""),
            "topk_requested": r.get("relocalization_topk_requested", ""),
            "topk_returned": r.get("relocalization_topk_returned", ""),
            "candidates_before_filter": r.get("relocalization_candidates_before_filter", ""),
            "candidates_after_filter": r.get("relocalization_candidates_after_filter", r.get("relocalization_candidates", "")),
            "filtered_by_descriptor": r.get("relocalization_filtered_by_descriptor", ""),
            "filtered_by_translation": r.get("relocalization_filtered_by_translation", ""),
            "filtered_by_other": r.get("relocalization_filtered_by_other", ""),
            "verification_attempted": r.get("relocalization_verification_attempted", ""),
            "verification_success": r.get("relocalization_verification_success", ""),
            "verification_best_submap": r.get("relocalization_verification_best_submap", ""),
            "verification_best_score": r.get("relocalization_verification_best_score", ""),
            "verification_best_inliers": r.get("relocalization_verification_best_inliers", ""),
            "verification_best_residual": r.get("relocalization_verification_best_residual", ""),
            "failure_reason": r.get("relocalization_failure_reason", ""),
            "debug_verification_only": r.get("debug_verification_only", ""),
            "debug_verified_rejected_topk_count": r.get("debug_verified_rejected_topk_count", ""),
            "debug_verified_best_submap": r.get("debug_verified_best_submap", ""),
            "debug_verified_best_descriptor_distance": r.get("debug_verified_best_descriptor_distance", ""),
            "debug_verified_best_score": r.get("debug_verified_best_score", ""),
            "debug_verified_best_inliers": r.get("debug_verified_best_inliers", ""),
            "debug_verified_best_residual": r.get("debug_verified_best_residual", ""),
            "debug_verified_success": r.get("debug_verified_success", ""),
            "verify_raw_topk_used": r.get("verify_raw_topk_used", ""),
            "verify_raw_topk_candidate_count": r.get("verify_raw_topk_candidate_count", ""),
            "verify_raw_topk_best_submap": r.get("verify_raw_topk_best_submap", ""),
            "verify_raw_topk_best_distance": r.get("verify_raw_topk_best_distance", ""),
            "verify_raw_topk_best_score": r.get("verify_raw_topk_best_score", ""),
            "verify_raw_topk_best_inliers": r.get("verify_raw_topk_best_inliers", ""),
            "verify_raw_topk_best_residual": r.get("verify_raw_topk_best_residual", ""),
            "verify_raw_topk_success": r.get("verify_raw_topk_success", ""),
            "recovery_state": r.get("recovery_state", ""),
            "recovering_stable_count": r.get("recovering_stable_count", ""),
            "recovering_required_stable_frames": r.get("recovering_required_stable_frames", ""),
            "recovering_transition_reason": r.get("recovering_transition_reason", ""),
            "descriptor_passed_main_threshold": r.get("descriptor_passed_main_threshold", ""),
            "descriptor_passed_raw_topk_experiment": r.get("descriptor_passed_raw_topk_experiment", ""),
            "smoother_guard_action": r.get("smoother_guard_action", ""),
        })
    write_csv(os.path.join(out_dir, "relocalization_summary.csv"), relocalization_fields, relocalization_summary_rows)

    write_csv(
        os.path.join(out_dir, "relocalization_topk_summary.csv"),
        [
            "frame_id", "top1_submap", "top1_distance", "top1_yaw", "top2_submap", "top2_distance",
            "top3_submap", "top3_distance", "topk_submaps", "topk_distances", "topk_returned",
            "candidates_after_filter", "failure_reason",
        ],
        [
            {
                "frame_id": frame(r),
                "top1_submap": r.get("relocalization_top1_submap", ""),
                "top1_distance": r.get("relocalization_top1_distance", ""),
                "top1_yaw": r.get("relocalization_top1_yaw", ""),
                "top2_submap": r.get("relocalization_top2_submap", ""),
                "top2_distance": r.get("relocalization_top2_distance", ""),
                "top3_submap": r.get("relocalization_top3_submap", ""),
                "top3_distance": r.get("relocalization_top3_distance", ""),
                "topk_submaps": r.get("relocalization_topk_submaps", ""),
                "topk_distances": r.get("relocalization_topk_distances", ""),
                "topk_returned": r.get("relocalization_topk_returned", ""),
                "candidates_after_filter": r.get("relocalization_candidates_after_filter", r.get("relocalization_candidates", "")),
                "failure_reason": r.get("relocalization_failure_reason", ""),
            }
            for r in analysis["relocalization_rows"]
        ],
    )

    write_csv(
        os.path.join(out_dir, "relocalization_failure_reasons.csv"),
        ["failure_reason", "count"],
        [{"failure_reason": k, "count": v} for k, v in analysis["relocalization_failure_reasons"].most_common()],
    )

    write_csv(
        os.path.join(out_dir, "relocalization_threshold_sweep.csv"),
        ["threshold", "frames_with_candidates", "top1_accepted_frames", "mean_candidate_count", "max_candidate_count", "top_submaps"],
        analysis["threshold_rows"],
    )
    write_threshold_summary(os.path.join(out_dir, "relocalization_threshold_sweep.md"), rows, analysis, source_csv)

    write_summary(os.path.join(out_dir, "debug_csv_summary.md"), rows, analysis, source_csv)


def write_threshold_summary(path: str, rows: List[Dict[str, str]], analysis: Dict[str, object], source_csv: str) -> None:
    relocalization_rows = analysis["relocalization_rows"]
    all_top1 = finite([value(r, "relocalization_top1_distance") for r in relocalization_rows])
    debug_rows = analysis["debug_verification_rows"]
    debug_success = [r for r in debug_rows if truthy(r.get("debug_verified_success", ""))]
    with open(path, "w", encoding="utf-8") as f:
        f.write("# Relocalization Threshold Sweep\n\n")
        f.write(f"- source_csv: `{source_csv}`\n")
        f.write(f"- relocalization_rows: {len(relocalization_rows)}\n")
        if all_top1:
            f.write(
                f"- top1_distance_min/mean/median/p90/max: {fmt(min(all_top1))}/{fmt(mean(all_top1))}/"
                f"{fmt(percentile(all_top1, 0.50))}/{fmt(percentile(all_top1, 0.90))}/{fmt(max(all_top1))}\n"
            )
        f.write(f"- debug_verification_rows: {len(debug_rows)}\n")
        f.write(f"- debug_verification_success: {len(debug_success)}\n\n")
        f.write("| threshold | frames_with_candidates | top1_accepted_frames | mean_candidate_count | max_candidate_count | top_submaps |\n")
        f.write("| ---: | ---: | ---: | ---: | ---: | --- |\n")
        for row in analysis["threshold_rows"]:
            f.write(
                f"| {row['threshold']} | {row['frames_with_candidates']} | {row['top1_accepted_frames']} | "
                f"{row['mean_candidate_count']} | {row['max_candidate_count']} | {row['top_submaps']} |\n"
            )


def top_items(counter: Counter, limit: int = 8) -> str:
    if not counter:
        return "- none\n"
    return "".join(f"- `{k}`: {v}\n" for k, v in counter.most_common(limit))


def write_summary(path: str, rows: List[Dict[str, str]], analysis: Dict[str, object], source_csv: str) -> None:
    high = analysis["high_score_large_delta"]
    high_preview = "\n".join(
        f"- frame {frame(r)}: state {state_before(r)}->{state_after(r)}, score={fmt(value(r, 'score'))}, "
        f"delta_t={fmt(value(r, 'delta_t'))}, reject={r.get('reject_reason', '') or 'accepted'}"
        for r in high[:12]
    ) or "- none"

    candidates = analysis["relocalization_candidates"]
    no_candidates = sum(1 for c in candidates if c == 0)
    longest = analysis["longest_lost"]
    longest_text = (
        f"{longest['start_frame']}..{longest['end_frame']} ({longest['frames']} frames)"
        if longest else "none"
    )

    rebuilds = analysis["rebuild_events"]
    first_rebuild = rebuilds[0]["frame_id"] if rebuilds else "none"
    first_degraded = analysis["first_degraded"]
    first_lost = analysis["first_lost"]
    first_relocalizing = analysis["first_relocalizing"]

    chain = (
        f"target rebuild first seen at frame {first_rebuild}; first DEGRADED={first_degraded}, "
        f"first LOST={first_lost}, first RELOCALIZING={first_relocalizing}; "
        f"longest LOST/RELOCALIZING segment={longest_text}; "
        f"relocalization no-candidate frames={no_candidates}."
    )

    with open(path, "w", encoding="utf-8") as f:
        f.write("# Debug CSV Summary\n\n")
        f.write(f"- source_csv: `{source_csv}`\n")
        f.write(f"- frames: {len(rows)}\n")
        f.write(f"- frame_range: {frame(rows[0])}..{frame(rows[-1])}\n")
        f.write(f"- first_degraded: {first_degraded}\n")
        f.write(f"- first_lost: {first_lost}\n")
        f.write(f"- first_relocalizing: {first_relocalizing}\n")
        f.write(f"- longest_lost_relocalizing_segment: {longest_text}\n\n")

        f.write("## State Counts\n\n")
        f.write(top_items(analysis["states"]))
        f.write("\n## Transition Counts\n\n")
        f.write(top_items(analysis["transitions"]))
        f.write("\n## Reject Reasons\n\n")
        f.write(top_items(analysis["reject_reasons"]))
        f.write("\n## Smoother Guard Actions\n\n")
        f.write(top_items(analysis["smoother_actions"]))
        f.write("\n## Target Rebuild Guard Actions\n\n")
        f.write(top_items(analysis["target_actions"]))
        f.write("\n## High Score + Large Delta Frames\n\n")
        f.write(high_preview)
        f.write("\n\n## Relocalization\n\n")
        f.write(f"- requested_frames: {len(analysis['relocalization_rows'])}\n")
        f.write(f"- success_frames: {len(analysis['relocalization_success'])}\n")
        f.write(f"- no_candidate_frames: {no_candidates}\n")
        if candidates:
            f.write(f"- candidates_min/max/mean: {min(candidates)}/{max(candidates)}/{fmt(mean(candidates))}\n")
        topk_returned = [v for v in analysis["relocalization_topk_returned"] if v > 0]
        if topk_returned:
            f.write(f"- topk_returned_min/max/mean: {min(topk_returned)}/{max(topk_returned)}/{fmt(mean(topk_returned))}\n")
        top1 = finite(analysis["relocalization_top1_distances"])
        if top1:
            f.write(f"- top1_distance_min/mean/max: {fmt(min(top1))}/{fmt(mean(top1))}/{fmt(max(top1))}\n")
        query_points = finite(analysis["relocalization_query_points"])
        if query_points:
            f.write(f"- query_points_min/mean/max: {fmt(min(query_points), 0)}/{fmt(mean(query_points), 0)}/{fmt(max(query_points), 0)}\n")
        f.write("\n### Relocalization Failure Reasons\n\n")
        f.write(top_items(analysis["relocalization_failure_reasons"]))
        f.write("\n### Threshold Sweep\n\n")
        for row in analysis["threshold_rows"]:
            f.write(
                f"- threshold {row['threshold']}: frames_with_candidates={row['frames_with_candidates']}, "
                f"top1_accepted={row['top1_accepted_frames']}, mean_candidates={row['mean_candidate_count']}, "
                f"top_submaps={row['top_submaps'] or 'none'}\n"
            )
        debug_rows = analysis["debug_verification_rows"]
        if debug_rows:
            debug_success = [r for r in debug_rows if truthy(r.get("debug_verified_success", ""))]
            f.write("\n### Debug-Only Rejected Top-k Verification\n\n")
            f.write(f"- debug_verification_rows: {len(debug_rows)}\n")
            f.write(f"- debug_verification_success: {len(debug_success)}\n")
            if debug_success:
                f.write(
                    "- successful_frames: "
                    + ", ".join(
                        f"{frame(r)}(submap={r.get('debug_verified_best_submap','')},dist={r.get('debug_verified_best_descriptor_distance','')},score={r.get('debug_verified_best_score','')})"
                        for r in debug_success[:20]
                    )
                    + "\n"
                )
        f.write("\n### Verify Raw Top-k Recovery\n\n")
        f.write(f"- verify_raw_topk_used_frames: {len(analysis['verify_raw_topk_rows'])}\n")
        f.write(f"- verify_raw_topk_success_frames: {len(analysis['verify_raw_topk_success'])}\n")
        if analysis["verify_raw_topk_success"]:
            f.write(
                "- verify_raw_topk_success_detail: "
                + ", ".join(
                    f"{frame(r)}(submap={r.get('verify_raw_topk_best_submap','')},dist={r.get('verify_raw_topk_best_distance','')},score={r.get('verify_raw_topk_best_score','')})"
                    for r in analysis["verify_raw_topk_success"][:20]
                )
                + "\n"
            )
        f.write("\n### Recovery Stabilization\n\n")
        f.write(f"- relocalizing_to_recovering_frames: {len(analysis['relocalizing_to_recovering'])}\n")
        f.write(f"- recovering_to_tracking_frames: {len(analysis['recovering_to_tracking'])}\n")
        f.write(f"- recovery_rejection_frames: {len(analysis['recovery_rejections'])}\n")
        stable_counts = [v for v in analysis["recovering_stable_counts"] if v > 0]
        f.write(f"- recovering_stable_count_max: {max(stable_counts) if stable_counts else 0}\n")
        f.write("\n## Initial Judgment\n\n")
        f.write(f"Most likely failure chain: {chain}\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze glim_localization debug CSV")
    parser.add_argument("csv_path")
    parser.add_argument("--out", required=True, help="Output report directory")
    parser.add_argument("--high-score", type=float, default=0.8)
    parser.add_argument("--large-delta", type=float, default=2.4)
    parser.add_argument("--threshold-sweep", default="0.30,0.35,0.40,0.45,0.50,0.55,0.60,0.70")
    args = parser.parse_args()

    try:
        rows = read_rows(args.csv_path)
        analysis = analyze(rows, args.high_score, args.large_delta, parse_thresholds(args.threshold_sweep))
        write_outputs(rows, analysis, args.out, args.csv_path)
    except Exception as e:
        os.makedirs(args.out, exist_ok=True)
        error_path = os.path.join(args.out, "debug_csv_summary.md")
        with open(error_path, "w", encoding="utf-8") as f:
            f.write("# Debug CSV Summary\n\n")
            f.write(f"Failed to analyze `{args.csv_path}`: {e}\n")
        print(f"failed to analyze debug CSV: {e}")
        return 1

    print(f"wrote debug CSV analysis to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
