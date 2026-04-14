#!/usr/bin/env python3

import argparse
import math
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple


if "MPLCONFIGDIR" not in os.environ:
  os.environ["MPLCONFIGDIR"] = tempfile.mkdtemp(prefix="glim_localization_mpl_")

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


@dataclass
class TrajectorySample:
  stamp: float
  x: float
  y: float
  z: float
  qx: float
  qy: float
  qz: float
  qw: float
  status: int
  matching_score: float


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Plot glim_localization trajectory output")
  parser.add_argument("trajectory", help="Path to glim_localization_traj.txt")
  parser.add_argument(
    "-o",
    "--output",
    default=None,
    help="Output image path. If both 2D and 3D are enabled, this path is used as a prefix stem.",
  )
  parser.add_argument("--plot-2d", action="store_true", help="Generate a 2D top-view trajectory plot")
  parser.add_argument("--plot-3d", action="store_true", help="Generate a 3D trajectory plot")
  parser.add_argument("--show-arrows", action="store_true", help="Draw sparse orientation arrows")
  parser.add_argument("--time-color", action="store_true", help="Color trajectory points by timestamp")
  parser.add_argument(
    "--arrow-step",
    type=int,
    default=0,
    help="Arrow sampling interval. Default is auto-selected from trajectory length.",
  )
  return parser.parse_args()


def load_trajectory(path: Path) -> List[TrajectorySample]:
  samples: List[TrajectorySample] = []

  with path.open("r", encoding="utf-8") as f:
    for lineno, line in enumerate(f, start=1):
      stripped = line.strip()
      if not stripped or stripped.startswith("#"):
        continue

      fields = stripped.split()
      if len(fields) != 10:
        raise ValueError(
          f"{path}:{lineno}: expected 10 columns "
          "(stamp x y z qx qy qz qw status_int matching_score), got {len(fields)}"
        )

      try:
        samples.append(
          TrajectorySample(
            stamp=float(fields[0]),
            x=float(fields[1]),
            y=float(fields[2]),
            z=float(fields[3]),
            qx=float(fields[4]),
            qy=float(fields[5]),
            qz=float(fields[6]),
            qw=float(fields[7]),
            status=int(float(fields[8])),
            matching_score=float(fields[9]),
          )
        )
      except ValueError as exc:
        raise ValueError(f"{path}:{lineno}: failed to parse numeric fields: {exc}") from exc

  if not samples:
    raise ValueError(f"{path}: trajectory file is empty")

  return samples


def forward_vector(sample: TrajectorySample) -> Tuple[float, float, float]:
  qx = sample.qx
  qy = sample.qy
  qz = sample.qz
  qw = sample.qw

  fx = 1.0 - 2.0 * (qy * qy + qz * qz)
  fy = 2.0 * (qx * qy + qw * qz)
  fz = 2.0 * (qx * qz - qw * qy)
  return fx, fy, fz


def compute_total_distance(samples: Sequence[TrajectorySample]) -> float:
  total = 0.0
  for prev, cur in zip(samples[:-1], samples[1:]):
    dx = cur.x - prev.x
    dy = cur.y - prev.y
    dz = cur.z - prev.z
    total += math.sqrt(dx * dx + dy * dy + dz * dz)
  return total


def bounds(values: Iterable[float]) -> Tuple[float, float]:
  values = list(values)
  return min(values), max(values)


def print_stats(samples: Sequence[TrajectorySample]) -> None:
  xs = [s.x for s in samples]
  ys = [s.y for s in samples]
  zs = [s.z for s in samples]
  start = samples[0]
  end = samples[-1]

  print(f"frames: {len(samples)}")
  print(f"duration_sec: {end.stamp - start.stamp:.3f}")
  print(f"total_distance_m: {compute_total_distance(samples):.3f}")
  print(f"start_xyz: ({start.x:.3f}, {start.y:.3f}, {start.z:.3f})")
  print(f"end_xyz: ({end.x:.3f}, {end.y:.3f}, {end.z:.3f})")
  print(f"x_bounds: ({min(xs):.3f}, {max(xs):.3f})")
  print(f"y_bounds: ({min(ys):.3f}, {max(ys):.3f})")
  print(f"z_bounds: ({min(zs):.3f}, {max(zs):.3f})")


def resolve_output_paths(input_path: Path, output_arg: str, want_2d: bool, want_3d: bool) -> List[Tuple[str, Path]]:
  base_name = input_path.stem
  if output_arg is None:
    prefix = input_path.with_name(base_name)
    if want_2d and want_3d:
      return [("2d", prefix.with_name(prefix.name + "_2d.png")), ("3d", prefix.with_name(prefix.name + "_3d.png"))]
    if want_2d:
      return [("2d", prefix.with_name(prefix.name + "_2d.png"))]
    return [("3d", prefix.with_name(prefix.name + "_3d.png"))]

  output_path = Path(output_arg)
  if want_2d and want_3d:
    stem = output_path.with_suffix("") if output_path.suffix else output_path
    return [("2d", stem.with_name(stem.name + "_2d.png")), ("3d", stem.with_name(stem.name + "_3d.png"))]

  return [("2d" if want_2d else "3d", output_path)]


def maybe_add_time_colorbar(fig: plt.Figure, artist, enabled: bool) -> None:
  if enabled and artist is not None:
    cbar = fig.colorbar(artist, ax=fig.axes, shrink=0.8, pad=0.08)
    cbar.set_label("Time [s]")


def set_equal_3d_axes(ax, xs: Sequence[float], ys: Sequence[float], zs: Sequence[float]) -> None:
  x_min, x_max = bounds(xs)
  y_min, y_max = bounds(ys)
  z_min, z_max = bounds(zs)

  x_mid = 0.5 * (x_min + x_max)
  y_mid = 0.5 * (y_min + y_max)
  z_mid = 0.5 * (z_min + z_max)

  radius = max(x_max - x_min, y_max - y_min, z_max - z_min, 1e-3) * 0.5
  ax.set_xlim(x_mid - radius, x_mid + radius)
  ax.set_ylim(y_mid - radius, y_mid + radius)
  ax.set_zlim(z_mid - radius, z_mid + radius)


def choose_arrow_indices(num_samples: int, arrow_step_arg: int) -> List[int]:
  if num_samples <= 1:
    return []

  if arrow_step_arg > 0:
    step = arrow_step_arg
  else:
    step = max(1, num_samples // 30)

  return list(range(0, num_samples, step))


def plot_2d(samples: Sequence[TrajectorySample], output: Path, show_arrows: bool, time_color: bool, arrow_step_arg: int) -> None:
  xs = [s.x for s in samples]
  ys = [s.y for s in samples]
  ts = [s.stamp for s in samples]

  fig, ax = plt.subplots(figsize=(10, 8), constrained_layout=True)
  ax.plot(xs, ys, color="0.65", linewidth=1.5, label="trajectory")

  scatter = None
  if time_color:
    scatter = ax.scatter(xs, ys, c=ts, cmap="viridis", s=10, label="time-colored samples")
  else:
    ax.scatter(xs, ys, color="#1f77b4", s=10)

  start = samples[0]
  end = samples[-1]
  ax.scatter([start.x], [start.y], color="#2ca02c", s=80, marker="o", label="start", zorder=5)
  ax.scatter([end.x], [end.y], color="#d62728", s=100, marker="X", label="end", zorder=5)

  if show_arrows:
    diag = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
    arrow_length = max(diag * 0.03, 0.3)
    for idx in choose_arrow_indices(len(samples), arrow_step_arg):
      fx, fy, _ = forward_vector(samples[idx])
      norm = math.hypot(fx, fy)
      if norm < 1e-9:
        continue
      ax.arrow(
        samples[idx].x,
        samples[idx].y,
        arrow_length * fx / norm,
        arrow_length * fy / norm,
        width=arrow_length * 0.03,
        head_width=arrow_length * 0.18,
        head_length=arrow_length * 0.22,
        color="#ff7f0e",
        alpha=0.75,
        length_includes_head=True,
      )

  ax.set_title("glim_localization Trajectory (2D Top View)")
  ax.set_xlabel("X [m]")
  ax.set_ylabel("Y [m]")
  ax.grid(True, linestyle="--", alpha=0.35)
  ax.axis("equal")
  ax.legend(loc="best")
  maybe_add_time_colorbar(fig, scatter, time_color)

  output.parent.mkdir(parents=True, exist_ok=True)
  fig.savefig(output, dpi=200)
  plt.close(fig)


def plot_3d(samples: Sequence[TrajectorySample], output: Path, show_arrows: bool, time_color: bool, arrow_step_arg: int) -> None:
  xs = [s.x for s in samples]
  ys = [s.y for s in samples]
  zs = [s.z for s in samples]
  ts = [s.stamp for s in samples]

  fig = plt.figure(figsize=(11, 9), constrained_layout=True)
  ax = fig.add_subplot(111, projection="3d")
  ax.plot(xs, ys, zs, color="0.65", linewidth=1.2, label="trajectory")

  scatter = None
  if time_color:
    scatter = ax.scatter(xs, ys, zs, c=ts, cmap="viridis", s=10, depthshade=False)
  else:
    ax.scatter(xs, ys, zs, color="#1f77b4", s=10, depthshade=False)

  start = samples[0]
  end = samples[-1]
  ax.scatter([start.x], [start.y], [start.z], color="#2ca02c", s=80, marker="o", label="start", depthshade=False)
  ax.scatter([end.x], [end.y], [end.z], color="#d62728", s=100, marker="X", label="end", depthshade=False)

  if show_arrows:
    span = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 1.0)
    arrow_length = span * 0.06
    for idx in choose_arrow_indices(len(samples), arrow_step_arg):
      fx, fy, fz = forward_vector(samples[idx])
      norm = math.sqrt(fx * fx + fy * fy + fz * fz)
      if norm < 1e-9:
        continue
      ax.quiver(
        samples[idx].x,
        samples[idx].y,
        samples[idx].z,
        arrow_length * fx / norm,
        arrow_length * fy / norm,
        arrow_length * fz / norm,
        color="#ff7f0e",
        linewidth=1.0,
        arrow_length_ratio=0.25,
        alpha=0.75,
      )

  ax.set_title("glim_localization Trajectory (3D)")
  ax.set_xlabel("X [m]")
  ax.set_ylabel("Y [m]")
  ax.set_zlabel("Z [m]")
  set_equal_3d_axes(ax, xs, ys, zs)
  ax.legend(loc="best")
  maybe_add_time_colorbar(fig, scatter, time_color)

  output.parent.mkdir(parents=True, exist_ok=True)
  fig.savefig(output, dpi=200)
  plt.close(fig)


def main() -> int:
  args = parse_args()
  input_path = Path(args.trajectory)
  if not input_path.exists():
    raise FileNotFoundError(f"trajectory file not found: {input_path}")

  want_2d = args.plot_2d
  want_3d = args.plot_3d
  if not want_2d and not want_3d:
    want_2d = True
    want_3d = True

  samples = load_trajectory(input_path)
  print_stats(samples)

  outputs = resolve_output_paths(input_path, args.output, want_2d, want_3d)
  for plot_kind, output_path in outputs:
    if plot_kind == "2d":
      plot_2d(samples, output_path, args.show_arrows, args.time_color, args.arrow_step)
    elif plot_kind == "3d":
      plot_3d(samples, output_path, args.show_arrows, args.time_color, args.arrow_step)
    else:
      raise ValueError(f"unknown plot kind: {plot_kind}")

    print(f"saved_{plot_kind}: {output_path}")

  return 0


if __name__ == "__main__":
  raise SystemExit(main())
