#!/usr/bin/env python3

import argparse
from pathlib import Path

from usage_report_lib import build_analysis, compute_summary, generate_plots, write_summary_files


def main():
    parser = argparse.ArgumentParser(description="Generate resource plots from glim_localization monitoring logs")
    parser.add_argument("--input-dir", required=True, help="Monitoring log directory created by run_with_time.sh")
    parser.add_argument("--output-dir", required=True, help="Directory for generated plots and summary files")
    parser.add_argument("--title", default="glim_localization resource usage", help="Plot/report title")
    parser.add_argument("--dpi", type=int, default=150, help="PNG output DPI")
    parser.add_argument("--disable-gpu", action="store_true", help="Skip GPU plots even if GPU logs exist")
    parser.add_argument("--disable-io", action="store_true", help="Skip I/O throughput plot")
    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)

    analysis = build_analysis(input_dir)
    summary = compute_summary(analysis)
    artifacts = write_summary_files(summary, output_dir)
    plot_manifest = generate_plots(
        analysis,
        output_dir,
        title=args.title,
        enable_gpu=not args.disable_gpu,
        enable_io=not args.disable_io,
        dpi=args.dpi,
    )

    print(f"input_dir: {input_dir}")
    print(f"output_dir: {output_dir}")
    print(f"summary_csv: {output_dir / artifacts['summary_csv']}")
    print(f"summary_json: {output_dir / artifacts['summary_json']}")
    for key, filename in sorted(plot_manifest.items()):
        if filename:
            print(f"{key}: {output_dir / filename}")
        else:
            print(f"{key}: skipped")


if __name__ == "__main__":
    main()
