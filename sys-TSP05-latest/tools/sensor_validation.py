#!/usr/bin/env python3
"""TSP05: save ACTUAL ground-truth experiments and plot calibrated response.

Record (requires sourced ROS 2 Humble environment, running native bridge):
    python3 tools/sensor_validation.py record sharp 20 --seconds 4
    python3 tools/sensor_validation.py record tof 20 --seconds 4
    python3 tools/sensor_validation.py record pot 90 --seconds 4
    python3 tools/sensor_validation.py record flex 45 --seconds 4

Use a ruler/protractor to physically measure the input and type that truth.
Plot (requires python3-matplotlib, no ROS required):
    python3 tools/sensor_validation.py plot
"""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CSV = ROOT / 'calibration' / 'measurements.csv'
COLUMNS = ('sensor', 'truth', 'raw', 'filtered', 'reported', 'arduino_ms', 'utc')
FIELDS = {
    # sensor -> (raw electrical output ADC, converted engineering units, units)
    'sharp': ('sharp_raw', 'sharp_cm', 'cm'),
    'tof': ('tof_mm', 'tof_cm', 'cm'),
    'pot': ('pot_raw', 'pot_deg', 'degrees'),
    'flex': ('flex_raw', 'flex_deg', 'degrees'),
}


def record(args):
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as exc:
        raise SystemExit('Source /opt/ros/humble/setup.bash first: ' + str(exc))

    class Collector(Node):
        def __init__(self):
            super().__init__('tsp05_sensor_validation')
            self.records = []
            self.create_subscription(String, '/tsp05/state_json', self.on_msg, 10)

        def on_msg(self, msg):
            try:
                d = json.loads(msg.data)
                raw_name, reported_name, _ = FIELDS[args.sensor]
                if args.sensor == 'sharp' and d['sharp_st'] != 'VALID':
                    return
                if args.sensor == 'tof' and not d['tof_ok']:
                    return
                raw = float(d[raw_name])
                converted = (float(d['tof_mm']) / 10.0 if args.sensor == 'tof'
                             else float(d[reported_name]))
                if not (math.isfinite(raw) and math.isfinite(converted)):
                    return
                self.records.append(dict(sensor=args.sensor, truth=float(args.truth),
                                         raw=raw, filtered=(float(d['flex_adc'])
                                                          if args.sensor == 'flex' else ''),
                                         reported=converted,
                                         arduino_ms=int(d['ms']),
                                         utc=datetime.now(timezone.utc).isoformat()))
            except (ValueError, KeyError, TypeError):
                return

    rclpy.init()
    node = Collector()
    print(f'Keep {args.sensor} at MEASURED {args.truth:g} {FIELDS[args.sensor][2]} '
          f'for {args.seconds:g} s; capturing live sensor samples...')
    try:
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        records = node.records
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if len(records) < 5:
        raise SystemExit('Only %d valid samples; check /tsp05/connected, range and port.' % len(records))
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    is_new = not args.csv.exists() or args.csv.stat().st_size == 0
    with args.csv.open('a', newline='') as f:
        w = csv.DictWriter(f, COLUMNS)
        if is_new:
            w.writeheader()
        w.writerows(records)
    mean = statistics.mean(v['reported'] for v in records)
    raw_mean = statistics.mean(v['raw'] for v in records)
    sigma = statistics.stdev(v['raw'] for v in records)
    print(f'Captured {len(records)} real samples to {args.csv}')
    print(f'Converted mean={mean:.2f} {FIELDS[args.sensor][2]}; '
          f'error={mean - args.truth:+.2f}; electrical raw={raw_mean:.2f} +/-{sigma:.2f}')


def read_records(path):
    if not path.exists():
        raise ValueError(f'Missing {path}: first record actual measurements (see README).')
    with path.open(newline='') as f:
        rows = list(csv.DictReader(f))
    valid = []
    for r in rows:
        try:
            if r['sensor'] not in FIELDS:
                continue
            d = dict(sensor=r['sensor'], truth=float(r['truth']),
                     raw=float(r['raw']), reported=float(r['reported']))
            if all(math.isfinite(d[k]) for k in ('truth', 'raw', 'reported')):
                if r.get('filtered'):
                    try:
                        d['filtered'] = float(r['filtered'])
                    except ValueError:
                        pass
                valid.append(d)
        except (ValueError, KeyError, TypeError):
            continue
    return valid


def group_records(rows):
    groups = {}
    for r in rows:
        groups.setdefault((r['sensor'], r['truth']), []).append(r)
    return groups


def summarize(rows):
    """Compute per-ground-truth mean bias and noise; no invented calibration points."""
    grouped = group_records(rows)
    result = {}
    for sensor in FIELDS:
        level_stats = []
        for (name, truth), samples in sorted(grouped.items()):
            if name != sensor:
                continue
            m = statistics.mean(r['reported'] for r in samples)
            electrical = statistics.mean(r['raw'] for r in samples)
            sigma = statistics.stdev(r['raw'] for r in samples) if len(samples) > 1 else 0.0
            level_stats.append(dict(truth=truth, observed=m,
                                    error=m - truth, raw=electrical,
                                    raw_std=sigma, n=len(samples)))
        if not level_stats:
            continue
        errors = [x['error'] for x in level_stats]
        result[sensor] = dict(
            levels=level_stats, mae=sum(abs(x) for x in errors) / len(errors),
            rmse=math.sqrt(sum(x * x for x in errors) / len(errors)),
            max_abs=max(abs(x) for x in errors),
            count=len(level_stats),
            units=FIELDS[sensor][2],
        )
    return result


def plot(args):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit('Install python3-matplotlib: ' + str(exc))
    rows = read_records(args.csv)
    report = summarize(rows)
    if not report:
        raise SystemExit('CSV contains no valid recorded data.')
    output = args.csv.parent
    output.mkdir(parents=True, exist_ok=True)
    summary_lines = [
        '# TSP05 experimentally measured sensor conversion error', '',
        'These statistics were calculated from recorded physical ground truth, not from firmware expectations.',
        'MAE/RMSE/max error weight each distinct reference value equally.', '',
        '| Sensor | Physical reference points | MAE | RMSE | Max abs error |',
        '|---|---:|---:|---:|---:|',
    ]
    for sensor, data in report.items():
        summary_lines.append(f"| {sensor} | {data['count']} | {data['mae']:.2f} "
                             f"{data['units']} | {data['rmse']:.2f} {data['units']} "
                             f"| {data['max_abs']:.2f} {data['units']} |")
    summary_lines += ['', '## Physical reference points (one row per real measurement setting)', '',
                      '| Sensor | Reference | Reported mean | Error | Electrical mean | Samples |',
                      '|---|---:|---:|---:|---:|---:|']
    for sensor, data in report.items():
        for x in data['levels']:
            summary_lines.append(f"| {sensor} | {x['truth']:.1f} | {x['observed']:.2f} | "
                                 f"{x['error']:+.2f} | {x['raw']:.2f} | {x['n']} |")
    summary_lines += ['', 'Physical truth must be measured independently (ruler or protractor).',
                      'Flex degrees and potentiometer degrees are endpoint-based approximations until validated.',
                      'ToF output is already in millimeters; it is not suitable as the raw electrical transfer sensor.',
                      'The Sharp IR electrical response is captured as 10-bit ADC counts.', '']
    (output / 'error_report.md').write_text('\n'.join(summary_lines))
    print('Wrote:', output / 'error_report.md')

    # Extra evidence of filtering: raw and median+EMA values during the SAME
    # held physical flex angle. Capture one steady flex setting for >=20 s.
    flex_sets = [(key, values) for key, values in group_records(rows).items()
                 if key[0] == 'flex' and len(values) >= 5 and
                 all('filtered' in item for item in values)]
    if flex_sets:
        (name, truth), held = max(flex_sets, key=lambda kv: len(kv[1]))
        raw = [v['raw'] for v in held]
        filtered = [v['filtered'] for v in held]
        raw_sd = statistics.stdev(raw)
        filtered_sd = statistics.stdev(filtered)
        fig, ax = plt.subplots(figsize=(9, 4))
        ax.plot(range(len(held)), raw, label=f'Raw ADC (SD {raw_sd:.2f})', alpha=0.65)
        ax.plot(range(len(held)), filtered,
                label=f'Median+EMA filtered ADC (SD {filtered_sd:.2f})')
        ax.set(xlabel='Successive captured samples (~10 Hz)', ylabel='ADC counts',
               title=f'Flex held at physically measured {truth:g}°: raw vs filtered')
        ax.grid(True, alpha=0.2)
        ax.legend()
        fig.tight_layout()
        fig.savefig(output / 'flex_raw_filtered.png', dpi=180)
        plt.close(fig)
        summary_lines.extend([
            '', '## Sensor filtering at held flex angle',
            f'At independent reference {truth:g}°: raw ADC SD = {raw_sd:.3f}; '
            f'filtered ADC SD = {filtered_sd:.3f} (n={len(held)}).',
            'Compare raw and filtered over the same stationary time interval; '
            'if the flex moved during capture, repeat at a steady angle.',
        ])
        (output / 'error_report.md').write_text('\n'.join(summary_lines))
        print('Wrote:', output / 'flex_raw_filtered.png')

    if 'sharp' in report:
        levels = report['sharp']['levels']
        if len(levels) < 3:
            print('NOTE: collect >=3 distinct Sharp reference distances for a meaningful transfer plot; '
                  '7 points across 10-80 cm recommended.')
        x = [p['truth'] for p in levels]
        y = [p['raw'] for p in levels]
        e = [p['raw_std'] for p in levels]
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.errorbar(x, y, yerr=e, fmt='o-', capsize=3,
                    label='Measured ADC mean +/- sample SD')
        ax.set(xlabel='Ground-truth distance from ruler (cm) [physical input]',
               ylabel='Sharp GP2Y0A21 raw ADC count (0-1023) [electrical output]',
               title='Measured Sharp IR sensor transfer characteristic')
        ax.grid(True, alpha=0.25)
        ax.legend()
        fig.tight_layout()
        fig.savefig(output / 'sharp_transfer_function.png', dpi=180)
        plt.close(fig)
        print('Wrote:', output / 'sharp_transfer_function.png')

    fig, ax = plt.subplots(figsize=(8, 5))
    for sensor, data in report.items():
        levels = data['levels']
        ax.plot([v['truth'] for v in levels], [v['error'] for v in levels], 'o-',
                label=f'{sensor} ({data["units"]})')
    ax.axhline(0, linestyle='--', linewidth=1)
    ax.set(xlabel='Reference physical input (units differ by series)',
           ylabel='Reported mean minus reference (units differ by series)',
           title='Measured sensor-conversion errors (see report for per-sensor units)')
    ax.legend()
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(output / 'conversion_errors.png', dpi=180)
    plt.close(fig)
    print('Wrote:', output / 'conversion_errors.png')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)
    rec = sub.add_parser('record', help='record truth and a few seconds of LIVE data')
    rec.add_argument('sensor', choices=FIELDS)
    rec.add_argument('truth', type=float, help='independently measured physical value')
    rec.add_argument('--seconds', type=float, default=3.0)
    rec.add_argument('--csv', type=Path, default=DEFAULT_CSV)
    graph = sub.add_parser('plot', help='plot real measured sensor data')
    graph.add_argument('--csv', type=Path, default=DEFAULT_CSV)
    args = parser.parse_args()
    if args.cmd == 'record':
        if not math.isfinite(args.truth) or args.seconds < 1:
            parser.error('truth must be finite and --seconds >=1')
        record(args)
    elif args.cmd == 'plot':
        plot(args)


if __name__ == '__main__':
    main()
