#!/usr/bin/env python3
"""Score an experiment folder (from run_experiment.py) against Gazebo's ground truth.

    python3 tools/analyze_experiment.py results/<time>

Writes into that folder:
  summary.md            success rate, per-run table, per-waypoint and hold statistics
  waypoint_errors.csv   true error at each waypoint, every run
  hold_stats.csv        true range / lateral / height / bearing during each 5 s hold
  paths.png             top-down true paths of all runs, waypoints, marker, hold spot
  hold.png              true range, lateral offset and height error over the hold
  waypoint_errors.png   true position error per waypoint across runs

"True" values come from Gazebo (/ground_truth/drone), expressed in the mission frame (origin
at the takeoff spot). The drone's own estimate is shown alongside for comparison.
"""
import csv
import glob
import json
import math
import os
import statistics as st
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

CAM_OFFSET = (0.15, 0.0, -0.02)    # camera in body FLU (sim model)
MARKER_LOCAL = (0.0105, 0.0, 1.5)  # marker face centre in the marker model's frame


def rotate(q, v):
    """Rotate vector v by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    # v' = v + 2w(q x v) + 2 q x (q x v)
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return (vx + 2 * (w * cx + ccx), vy + 2 * (w * cy + ccy), vz + 2 * (w * cz + ccz))


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def load_run(run_dir):
    js = sorted(glob.glob(os.path.join(run_dir, 'mission_*.json')))
    if not js:
        return None
    summary = json.load(open(js[-1]))
    rows = list(csv.DictReader(open(js[-1][:-5] + '.csv')))
    return summary, rows


def true_state(row, origin):
    """True drone position (mission frame), quaternion, camera position (world)."""
    if not row['gt_qw']:
        return None
    p = (float(row['gt_x']), float(row['gt_y']), float(row['gt_z']))
    q = tuple(float(row[k]) for k in ('gt_qx', 'gt_qy', 'gt_qz', 'gt_qw'))
    rel = (p[0] - origin['x'], p[1] - origin['y'], p[2] - origin['z'])
    c = rotate(q, CAM_OFFSET)
    cam = (p[0] + c[0], p[1] + c[1], p[2] + c[2])
    return rel, q, cam


def marker_geometry(m):
    q = (0.0, 0.0, math.sin(m['yaw'] / 2), math.cos(m['yaw'] / 2))
    o = rotate(q, MARKER_LOCAL)
    centre = (m['x'] + o[0], m['y'] + o[1], m['z'] + o[2])
    normal = (math.cos(m['yaw']), math.sin(m['yaw']))   # marker faces its +x
    return centre, normal


def hold_errors(row, origin, centre, normal):
    s = true_state(row, origin)
    if s is None:
        return None
    _, q, cam = s
    d = (cam[0] - centre[0], cam[1] - centre[1], cam[2] - centre[2])
    rng = d[0] * normal[0] + d[1] * normal[1]
    lat = d[0] * -normal[1] + d[1] * normal[0]
    bearing = math.remainder(math.atan2(-d[1], -d[0]) - yaw_of(q), 2 * math.pi)
    return {'range': rng, 'lateral': lat, 'height': d[2], 'bearing_deg': math.degrees(bearing)}


def fmt(vals, unit='m', nd=3):
    if not vals:
        return 'n/a'
    if len(vals) == 1:
        return f'{vals[0]:.{nd}f} {unit}'
    return f'{st.mean(vals):.{nd}f} ± {st.stdev(vals):.{nd}f} {unit} (max {max(vals, key=abs):.{nd}f})'


def main():
    out = sys.argv[1]
    config = json.load(open(os.path.join(out, 'experiment.json')))
    conds = {c['run']: c for c in config['run_conditions']}
    runs = []
    for run_dir in sorted(glob.glob(os.path.join(out, 'run_*'))):
        n = int(run_dir.rsplit('_', 1)[1])
        loaded = load_run(run_dir)
        runs.append((n, run_dir, loaded))

    wp_rows, hold_rows, run_table = [], [], []
    plt.rcParams.update({'figure.dpi': 120, 'font.size': 9})
    fig_p, axp = plt.subplots(figsize=(6.5, 6.5))
    fig_h, axh = plt.subplots(3, 1, figsize=(7, 7), sharex=True)
    waypoints = None
    marker_drawn = False

    for n, run_dir, loaded in runs:
        cond = conds.get(n, {})
        if loaded is None:
            run_table.append({'run': n, 'success': False, 'note': 'no log'})
            continue
        summary, rows = loaded
        origin = summary.get('true_origin')
        marker = summary.get('true_marker')
        entry = {'run': n, 'success': summary.get('success', False),
                 'wind': f"{cond.get('wind_speed', 0):.1f} m/s @ {cond.get('wind_dir_deg', 0):.0f}°",
                 'duration_s': summary.get('duration_s'),
                 'search_deg': summary.get('search', {}).get('turned_deg'),
                 'note': summary.get('error', '')}
        if origin is None:
            entry['note'] = 'no ground truth'
            run_table.append(entry)
            continue

        # Path
        xs, ys = [], []
        for r in rows:
            s = true_state(r, origin)
            if s and r['phase'] not in ('arming',):
                xs.append(s[0][0])
                ys.append(s[0][1])
        axp.plot(xs, ys, lw=0.9, alpha=0.75, label=f'run {n}')

        # Waypoints: true error at the end of each waypoint's hold, and averaged over the hold
        wps = summary.get('waypoints', [])
        if wps:
            waypoints = [(w['x'], w['y'], w['z']) for w in wps]
        for w in wps:
            seg = [r for r in rows if r['phase'] == 'waypoints' and int(r['waypoint']) == w['waypoint']
                   and r['gt_qw']]
            t_end = w['t_end']
            hs = w.get('hold_s', 2.0)
            hold_seg = [r for r in seg if t_end - hs <= float(r['t']) <= t_end]
            arrive = [r for r in seg if float(r['t']) <= t_end - hs]
            def err(r):
                p = true_state(r, origin)[0]
                return math.dist(p, (w['x'], w['y'], w['z']))
            def est_err(r):
                return math.dist((float(r['x']), float(r['y']), float(r['z'])), (w['x'], w['y'], w['z']))
            wp_rows.append({
                'run': n, 'waypoint': w['waypoint'],
                'true_arrival_error_m': round(err(arrive[-1]), 3) if arrive else '',   # = hold start
                'settle_time_s': w.get('settle_time_s', ''),
                'true_hold_error_m': round(st.mean(err(r) for r in hold_seg), 3) if hold_seg else '',
                'true_end_error_m': round(err(hold_seg[-1]), 3) if hold_seg else '',
                'est_hold_error_m': round(st.mean(est_err(r) for r in hold_seg), 3) if hold_seg else '',
                'leg_time_s': w['leg_time_s']})

        # Hold (true geometry)
        if marker:
            centre, normal = marker_geometry(marker)
            if not marker_drawn:
                mx, my = centre[0] - origin['x'], centre[1] - origin['y']
                axp.plot([mx - 0.15 * normal[1], mx + 0.15 * normal[1]],
                         [my + 0.15 * normal[0], my - 0.15 * normal[0]], 'k-', lw=3)
                axp.annotate('marker', (mx, my), textcoords='offset points', xytext=(6, 4))
                tx, ty = mx + normal[0] * (1.0 + 0.15), my + normal[1] * (1.0 + 0.15)
                axp.plot(tx, ty, 'kx', ms=8)
                axp.annotate('hold spot\n(camera 1 m out)', (tx, ty), textcoords='offset points',
                             xytext=(6, -18), fontsize=8)
                marker_drawn = True
            # Score only a completed hold: the last continuous run of 'hold' rows in a run that
            # succeeded (failed runs only had partial holds that restarted).
            hold = []
            for r in rows:
                if r['phase'] == 'hold':
                    hold.append(r)
                elif r['phase'] in ('approach',):
                    hold = []
            if hold and summary.get('success'):
                errs = [e for e in (hold_errors(r, origin, centre, normal) for r in hold) if e]
                t0 = float(hold[0]['t'])
                ts = [float(r['t']) - t0 for r in hold if r['gt_qw']]
                for ax, key, ref in zip(axh, ('range', 'lateral', 'height'), (1.0, 0.0, 0.0)):
                    ax.plot(ts, [e[key] for e in errs], lw=0.9, alpha=0.8)
                h = summary.get('hold', {})
                rec = {'run': n,
                       'true_range_mean': st.mean(e['range'] for e in errs),
                       'true_range_worst': max((e['range'] for e in errs), key=lambda v: abs(v - 1.0)),
                       'true_lateral_mean': st.mean(e['lateral'] for e in errs),
                       'true_lateral_worst': max((e['lateral'] for e in errs), key=abs),
                       'true_height_mean': st.mean(e['height'] for e in errs),
                       'true_height_worst': max((e['height'] for e in errs), key=abs),
                       'true_bearing_mean_deg': st.mean(e['bearing_deg'] for e in errs),
                       'true_bearing_worst_deg': max((e['bearing_deg'] for e in errs), key=abs),
                       'est_range_mean': h.get('range_mean'), 'approach_s': h.get('approach_s'),
                       'restarts': h.get('restarts')}
                hold_rows.append({k: (round(v, 4) if isinstance(v, float) else v) for k, v in rec.items()})
                entry.update(approach_s=h.get('approach_s'),
                             true_range=f"{rec['true_range_mean']:.3f}",
                             true_lateral=f"{rec['true_lateral_mean']:+.3f}")
        run_table.append(entry)

    # --- plots ---
    if waypoints:
        wx, wy = [0.0] + [w[0] for w in waypoints], [0.0] + [w[1] for w in waypoints]
        axp.plot(wx, wy, 'k--', lw=0.8, alpha=0.6)
        for i, (x, y, _) in enumerate(waypoints, 1):
            axp.plot(x, y, 'o', mfc='none', mec='k', ms=14)
            axp.annotate(f'WP{i}', (x, y), textcoords='offset points', xytext=(8, 8))
        axp.plot(0, 0, 'k^', ms=8)
        axp.annotate('takeoff', (0, 0), textcoords='offset points', xytext=(8, -12))
    axp.set_aspect('equal')
    axp.set_xlabel('East (m)')
    axp.set_ylabel('North (m)')
    axp.set_title(f'True paths, {len(runs)} runs (Gazebo ground truth)')
    axp.grid(alpha=0.3)
    axp.legend(fontsize=7, loc='lower right')
    fig_p.tight_layout()
    fig_p.savefig(os.path.join(out, 'paths.png'))

    for ax, label, ref, tol in zip(axh, ('range to marker (m)', 'lateral offset (m)', 'height error (m)'),
                                   (1.0, 0.0, 0.0), (0.10, 0.15, 0.10)):
        ax.axhline(ref, color='k', lw=0.8)
        ax.axhspan(ref - tol, ref + tol, color='g', alpha=0.08)
        ax.set_ylabel(label)
        ax.grid(alpha=0.3)
    axh[0].set_title('True errors during the 5 s hold (green = tolerance)')
    axh[-1].set_xlabel('time since hold started (s)')
    fig_h.tight_layout()
    fig_h.savefig(os.path.join(out, 'hold.png'))

    if wp_rows:
        fig_w, axw = plt.subplots(figsize=(6, 3.5))
        ids = sorted({r['waypoint'] for r in wp_rows})
        data = [[r['true_hold_error_m'] for r in wp_rows if r['waypoint'] == i and r['true_hold_error_m'] != '']
                for i in ids]
        axw.boxplot(data, tick_labels=[f'WP{i}' for i in ids])
        for i, d in enumerate(data, 1):
            axw.plot([i] * len(d), d, 'k.', alpha=0.5)
        axw.set_ylabel('true position error while holding (m)')
        axw.set_title('Waypoint accuracy across runs (Gazebo ground truth)')
        axw.grid(alpha=0.3)
        fig_w.tight_layout()
        fig_w.savefig(os.path.join(out, 'waypoint_errors.png'))

    # --- tables ---
    if wp_rows:
        with open(os.path.join(out, 'waypoint_errors.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(wp_rows[0].keys()))
            w.writeheader()
            w.writerows(wp_rows)
    if hold_rows:
        with open(os.path.join(out, 'hold_stats.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(hold_rows[0].keys()))
            w.writeheader()
            w.writerows(hold_rows)

    ok = sum(1 for r in run_table if r.get('success'))
    L = [f'# Experiment {os.path.basename(os.path.normpath(out))}', '',
         f"{config['runs']} runs of the full mission (waypoints → search → approach → 5 s hold at 1 m → land), "
         f"each in a fresh headless sim. Wind {config['wind_min']}–{config['wind_max']} m/s random direction "
         f"plus the world's gusts; GPS/baro noise {'on' if config['noise'] else 'off'}. "
         f"{'Position from optical flow + rangefinder, no GPS (--gps-denied). ' if config.get('gps_denied') else ''}"
         "Hold statistics are over each successful run's completed 5 s hold. "
         'Errors are measured against Gazebo ground truth.', '',
         f'**Success: {ok}/{len(run_table)}**', '',
         '| Run | OK | Wind | Duration (s) | Search (°) | Settle (s) | True range (m) | True lateral (m) | Note |',
         '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for r in run_table:
        L.append(f"| {r['run']} | {'✓' if r.get('success') else '✗'} | {r.get('wind', '')} | "
                 f"{r.get('duration_s', '')} | {r.get('search_deg', '')} | {r.get('approach_s', '')} | "
                 f"{r.get('true_range', '')} | {r.get('true_lateral', '')} | {r.get('note', '')} |")
    if wp_rows:
        L += ['', '## Waypoints (true position error)', '',
              '| Waypoint | Target (m) | When the hold starts (settled) | Mean over the 2 s hold | End of hold | Drone\'s own estimate (mean over hold) |',
              '| --- | --- | --- | --- | --- | --- |']
        for i in sorted({r['waypoint'] for r in wp_rows}):
            rs = [r for r in wp_rows if r['waypoint'] == i]
            tgt = waypoints[i - 1] if waypoints else ('', '', '')
            L.append(f"| WP{i} | ({tgt[0]:g}, {tgt[1]:g}, {tgt[2]:g}) | "
                     f"{fmt([r['true_arrival_error_m'] for r in rs if r['true_arrival_error_m'] != ''])} | "
                     f"{fmt([r['true_hold_error_m'] for r in rs if r['true_hold_error_m'] != ''])} | "
                     f"{fmt([r['true_end_error_m'] for r in rs if r['true_end_error_m'] != ''])} | "
                     f"{fmt([r['est_hold_error_m'] for r in rs if r['est_hold_error_m'] != ''])} |")
    if hold_rows:
        col = lambda k: [r[k] for r in hold_rows if r[k] is not None]
        L += ['', '## Marker hold (true, camera to marker, mean over each run\'s 5 s hold)', '',
              '| Quantity | Tolerance | Mean ± std across runs (worst run) | Worst single sample |',
              '| --- | --- | --- | --- |',
              f"| Range (target 1.00 m) | ±0.10 m | {fmt(col('true_range_mean'))} | {fmt(col('true_range_worst'))} |",
              f"| Lateral offset | ±0.15 m | {fmt(col('true_lateral_mean'))} | {fmt(col('true_lateral_worst'))} |",
              f"| Height error | ±0.10 m | {fmt(col('true_height_mean'))} | {fmt(col('true_height_worst'))} |",
              f"| Bearing | ±5° | {fmt(col('true_bearing_mean_deg'), '°', 2)} | {fmt(col('true_bearing_worst_deg'), '°', 2)} |",
              '', f"Drone's own range estimate during the hold: {fmt(col('est_range_mean'))} "
              f"(vs true {fmt(col('true_range_mean'))}). Time from first sighting to settled: "
              f"{fmt(col('approach_s'), 's', 1)}. Hold restarts: {sum(col('restarts'))}."]
    L += ['', '![paths](paths.png)', '', '![hold](hold.png)', '', '![waypoints](waypoint_errors.png)', '']
    open(os.path.join(out, 'summary.md'), 'w').write('\n'.join(L))
    print('\n'.join(L[:12 + len(run_table)]))
    print(f'\nwrote {out}/summary.md, paths.png, hold.png, waypoint_errors.png, *.csv')


if __name__ == '__main__':
    main()
