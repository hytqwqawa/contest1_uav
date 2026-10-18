#!/usr/bin/env python3
"""Demo9 V2: offline/read-only flight-log diagnostic (ROS1 Noetic).

Does not launch ROS nodes, publish topics, call services, change parameters,
modify flight software, or act upon a live vehicle.

IMPORTANT: MAVROS/T265/range readings are sensor estimates. A threshold
crossing is NOT physical ground contact or an authorization to stop motors.
"""
import argparse
import bisect
import csv
import datetime
import math
import statistics
from pathlib import Path

NS = '/contest1'
STREAMS = {
    NS + '/ego_uav_interface/point_traj_status': 'status',
    NS + '/mavros/state': 'state',
    NS + '/mavros/extended_state': 'extended',
    NS + '/mavros/local_position/odom': 'odom',
    NS + '/ego_uav_interface/point_setpoint': 'point',
    NS + '/mavros/setpoint_raw/local': 'setpoint',
    NS + '/mavros/distance_sensor/hrlv_ez4_pub': 'range',
    NS + '/t265/odom/filtered': 't265',
    '/rosout_agg': 'rosout',
}


def median(values):
    return statistics.median(values) if values else None


def quantile(values, p):
    if not values:
        return None
    a = sorted(values)
    return a[int((len(a) - 1) * p)]


def printable(x, unit='m'):
    return 'N/A' if x is None or not math.isfinite(x) else ('%.3f%s' % (x, unit))


def nearest(rows, t, max_age=0.12):
    """Rows sorted by bag time, with t as first element."""
    if not rows:
        return None
    # Rows are tuples; tuple comparison keeps this O(log N), even for 30k samples.
    i = bisect.bisect_left(rows, (t,))
    choices = [rows[j] for j in (i - 1, i) if 0 <= j < len(rows)]
    best = min(choices, key=lambda q: abs(q[0] - t))
    return best if abs(best[0] - t) <= max_age else None


def last_before(rows, t):
    if not rows:
        return None
    i = bisect.bisect_right([r[0] for r in rows], t) - 1
    return rows[i] if i >= 0 else None


def stable_crossing(rows, lo, hi, index, threshold, above=True,
                    duration=0.18, max_interval=0.15):
    """Earliest sustained threshold crossing on data series (no single-sample triggers)."""
    if not rows or lo is None or hi is None or hi <= lo:
        return None
    for i, row in enumerate(rows):
        t = row[0]
        if t < lo or t >= hi:
            continue
        ok = row[index] >= threshold if above else row[index] <= threshold
        if not ok:
            continue
        previous = t
        enough = False
        for r in rows[i + 1:]:
            if r[0] > hi or r[0] - previous > max_interval:
                break
            ok_next = r[index] >= threshold if above else r[index] <= threshold
            if not ok_next:
                break
            previous = r[0]
            if r[0] - t >= duration:
                enough = True
                break
        if enough:
            return t
    return None


def partition(rows, lo, hi):
    if lo is None or hi is None or hi <= lo:
        return []
    return [r for r in rows if lo <= r[0] <= hi]


def odom_metrics(odom, point, t0, t1, gx, gy):
    selected = partition(odom, t0, t1)
    d0, errors, speeds = [], [], []
    for o in selected:
        t, x, y, z, vx, vy, vz = o
        d0.append(math.hypot(x - gx, y - gy))
        speeds.append(math.hypot(vx, vy))
        cmd = nearest(point, t, .09)
        if cmd is not None:
            errors.append(math.hypot(x - cmd[1], y - cmd[2]))
    return {
        'n': len(selected), 'xy_max': max(d0) if d0 else None,
        'xy_med': median(d0), 'xy_p95': quantile(d0, .95),
        'tracking_max': max(errors) if errors else None,
        'tracking_med': median(errors), 'tracking_p95': quantile(errors, .95),
        'xy_speed_med': median(speeds),
    }


def collect(bag):
    # Import only here so the analysis/math utilities can be tested without ROS.
    import rosbag
    sets = {k: [] for k in STREAMS.values()}
    with rosbag.Bag(str(bag), 'r') as reader:
        available = set(reader.get_type_and_topic_info().topics)
        for topic, msg, when in reader.read_messages(topics=list(STREAMS)):
            name = STREAMS[topic]
            t = when.to_sec()
            if name == 'status':
                sets[name].append((t, str(msg.data)))
            elif name == 'state':
                sets[name].append((t, bool(msg.armed), str(msg.mode)))
            elif name == 'extended':
                sets[name].append((t, int(msg.landed_state)))
            elif name in ('odom', 't265'):
                p = msg.pose.pose.position
                v = msg.twist.twist.linear
                vals = (p.x, p.y, p.z, v.x, v.y, v.z)
                if all(math.isfinite(x) for x in vals):
                    sets[name].append((t,) + vals)
            elif name == 'point':
                p = msg.position
                if all(math.isfinite(x) for x in (p.x, p.y, p.z)):
                    sets[name].append((t, p.x, p.y, p.z))
            elif name == 'setpoint':
                sets[name].append((t, msg.header.stamp.to_sec(), int(msg.header.seq)))
            elif name == 'range':
                r = float(msg.range)
                minimum = float(getattr(msg, 'min_range', 0.0))
                maximum = float(getattr(msg, 'max_range', 1e6))
                if math.isfinite(r) and 0 < r and minimum <= r <= maximum:
                    sets[name].append((t, r))
            elif name == 'rosout':
                desc = str(msg.msg)
                if any(word in desc for word in (
                        'TAKEOFF started', 'TAKEOFF trajectory completed',
                        'LAND started', 'LAND TERMINAL DESCENT', 'LAND aborted',
                        'LAND completed', 'LAND trajectory completed')):
                    sets[name].append((t, desc[:220]))
    for rows in sets.values():
        rows.sort(key=lambda r: r[0])
    return sets, available


def calculate(d):
    status, state, ext = d['status'], d['state'], d['extended']
    odom, point, ranges, t265 = d['odom'], d['point'], d['range'], d['t265']
    t_take = next((t for t, s in status if s == 'TAKEOFF_TRAJECTORY'), None)
    t_hold = next((t for t, s in status if t_take is not None and t > t_take
                   and s == 'AIRBORNE_HOLD'), None)
    t_land = next((t for t, s in status if t_hold is not None and t > t_hold
                   and s in ('LAND_TRAJECTORY', 'LAND_TERMINAL_DESCENT')), None)
    t_terminal = next((t for t, s in status if t_land is not None and t >= t_land
                       and s == 'LAND_TERMINAL_DESCENT'), None)
    t_disarm = next((t for t, armed, _ in state if t_land is not None and t > t_land
                     and not armed), None)
    t_landed = next((t for t, landed in ext if t_land is not None and t >= t_land
                     and landed == 1), None)
    if t_take is None:
        raise ValueError('No TAKEOFF_TRAJECTORY in POINT status; not a complete Demo9 bag')
    if len(odom) < 10:
        raise ValueError('Insufficient MAVROS odometry samples')

    # Ground baseline from ~1.5 s BEFORE the start, not from first 'HOLD' after it.
    ground = partition(odom, t_take - 1.5, t_take - .12)
    if len(ground) >= 4:
        gx = median([p[1] for p in ground])
        gy = median([p[2] for p in ground])
        gz = median([p[3] for p in ground])
        ground_quality = 'pre-takeoff median, %d samples' % len(ground)
    else:
        p = nearest(odom, t_take, .25)
        if p is None:
            raise ValueError('No nearby odometry for ground reference')
        gx, gy, gz = p[1:4]
        ground_quality = 'single-sample fallback (uncertain baseline)'

    relative = [(t, z - gz) for t, x, y, z, vx, vy, vz in odom]
    climb_limit = t_land if t_land is not None else odom[-1][0]
    t_z10 = stable_crossing(relative, t_take, climb_limit, 1, .10)
    t_z50 = stable_crossing(relative, t_take, climb_limit, 1, .50)
    t_z70 = stable_crossing(relative, t_take, climb_limit, 1, .70)
    hold_sample = nearest(odom, t_hold, .20) if t_hold is not None else None
    hold_z = hold_sample[3] - gz if hold_sample is not None else None

    # Commanded flight-phase and measured-height windows are intentionally separate.
    # Neither the POINT status nor the estimated height alone proves physical motion.
    odom_ascent_end = t_z70 if t_z70 is not None else t_z50
    odom_hover_start = t_z70 if t_z70 is not None else None
    landing_end = t_disarm if t_disarm is not None else (odom[-1][0] if t_land else None)
    commanded_take_metrics = odom_metrics(odom, point, t_take, t_hold, gx, gy)
    take_metrics = odom_metrics(odom, point, t_take, odom_ascent_end, gx, gy)
    measured_climb_metrics = odom_metrics(odom, point, t_z10, t_z70, gx, gy)
    deferred_climb_metrics = odom_metrics(odom, point, t_hold, t_z70, gx, gy) \
        if t_hold is not None and t_z70 is not None and t_z70 > t_hold else odom_metrics(odom, point, None, None, gx, gy)
    hover_metrics = odom_metrics(odom, point, odom_hover_start, t_land, gx, gy)
    hover_last = odom_metrics(odom, point,
                              max(odom_hover_start, t_land - 2.0)
                              if odom_hover_start is not None and t_land is not None else None,
                              t_land, gx, gy)
    land_metrics = odom_metrics(odom, point, t_land, landing_end, gx, gy)

    near = []
    for o in odom:
        t, x, y, z, vx, vy, vz = o
        if t_land is None or t < t_land or (landing_end and t > landing_end):
            continue
        r = nearest(ranges, t, .09)
        if r is None or r[1] > .15:
            continue
        c = nearest(point, t, .09)
        e = math.hypot(x - c[1], y - c[2]) if c else None
        near.append((t, x, y, z, vx, vy, vz, r[1], e))

    # T265 is in a potentially different frame: relative changes only.
    t265_summary = None
    if t265:
        start = nearest(t265, t_take, .20)
        end_at_hold = nearest(t265, t_hold, .20) if t_hold else None
        end_at_z70 = nearest(t265, t_z70, .20) if t_z70 else None
        if start:
            t265_summary = {
                'at_hold_z_delta': end_at_hold[3] - start[3] if end_at_hold else None,
                'at_hold_xy_delta': math.hypot(end_at_hold[1] - start[1], end_at_hold[2] - start[2])
                if end_at_hold else None,
                'at_z70_z_delta': end_at_z70[3] - start[3] if end_at_z70 else None,
                'at_z70_xy_delta': math.hypot(end_at_z70[1] - start[1], end_at_z70[2] - start[2])
                if end_at_z70 else None,
            }

    return locals()


def summarize(d, c, emit):
    t_take, t_hold, t_land = c['t_take'], c['t_hold'], c['t_land']
    gx, gy, gz = c['gx'], c['gy'], c['gz']
    emit('=== Demo9 V2: phase-verified read-only analysis ===')
    emit('[RECORD COUNTS] ' + ' '.join('%s=%d' % (k, len(d[k])) for k in
         ('status', 'odom', 'point', 't265', 'range', 'extended', 'setpoint')))
    emit('[GROUND REF] x=%.3fm y=%.3fm z=%.3fm (%s)' %
         (gx, gy, gz, c['ground_quality']))
    def elapsed(t):
        return 'N/A' if t is None else '%+.2fs' % (t - t_take)
    emit('[POINT STATUS] TAKEOFF=%s HOLD=%s LAND=%s TERMINAL=%s DISARM=%s LANDED_STATE=%s' %
         tuple(elapsed(c[key]) for key in
               ('t_take', 't_hold', 't_land', 't_terminal', 't_disarm', 't_landed')))
    emit('[ODOM HEIGHT] sustained +0.10m=%s  +0.50m=%s  +0.70m=%s' %
         tuple(elapsed(c[key]) for key in ('t_z10', 't_z50', 't_z70')))
    emit('[AT POINT HOLD] MAVROS relative Z=%s (target is ~0.80m)' %
         printable(c['hold_z']))
    if c['t_z10'] is not None and c['t_z70'] is not None:
        emit('[HEIGHT ESTIMATE] +0.10m -> +0.70m duration %.2fs (NOT external physical truth)' %
             (c['t_z70'] - c['t_z10']))

    warnings = []
    if c['t_hold'] is not None and c['hold_z'] is not None and c['hold_z'] < .50:
        warnings.append('POINT reports TAKEOFF complete, but MAVROS height at status HOLD is below +0.50m.')
    if c['t_hold'] is not None and c['t_z10'] is not None and c['t_z10'] > c['t_hold']:
        warnings.append('MAVROS sustained +0.10m occurs %.2fs after TAKEOFF -> HOLD; do NOT count the status-TAKEOFF interval as the full ascent.' % (c['t_z10'] - c['t_hold']))
    if c['t_z70'] is None:
        warnings.append('No sustained MAVROS +0.70m height detected before LAND: cannot identify measured hover phase.')
    elif c['t_hold'] is not None and c['t_z70'] > c['t_hold'] + 1:
        warnings.append('MAVROS reaches +0.70m %.2fs AFTER POINT declared takeoff complete; source timing/position estimate requires investigation.' % (c['t_z70'] - c['t_hold']))
    if not d['t265']:
        warnings.append('Filtered T265 stream missing from bag; independent estimator comparison unavailable.')
    if c['t_landed'] is None:
        warnings.append('No recorded PX4 landed_state=ON_GROUND during landing; do not infer physical touchdown from DISARM alone.')
    if c['t_land'] is None:
        warnings.append('No LAND_TRAJECTORY event recorded: landing analysis unavailable.')
    if 'fallback' in c['ground_quality']:
        warnings.append('Ground baseline is only one position sample; displacement estimates have higher uncertainty.')
    for w in warnings:
        emit('[WARN] ' + w)

    def metric_line(label, m):
        emit('[%s] samples=%d  from-ground XY(max/p95/median)=%s/%s/%s  '
             'XY tracking(max/p95)=%s/%s' %
             (label, m['n'], printable(m['xy_max']), printable(m['xy_p95']),
              printable(m['xy_med']), printable(m['tracking_max']),
              printable(m['tracking_p95'])))

    emit('')
    emit('--- COMMAND-STATUS window (shown for comparison only) ---')
    metric_line('POINT TAKEOFF -> AIRBORNE_HOLD', c['commanded_take_metrics'])
    emit('--- MAVROS-HEIGHT windows (relative to ground baseline, still estimated) ---')
    metric_line('COMMAND start -> measured +0.70m (+0.50m fallback)', c['take_metrics'])
    metric_line('MEASURED +0.10m -> +0.70m climb', c['measured_climb_metrics'])
    metric_line('POST-POINT-HOLD climb until +0.70m', c['deferred_climb_metrics'])
    metric_line('ALTITUDE-CONFIRMED HOVER (+0.70m until LAND)', c['hover_metrics'])
    metric_line('HOVER last 2s', c['hover_last'])
    metric_line('LAND from POINT command to DISARM', c['land_metrics'])

    near = c['near']
    if near:
        n0 = near[0]
        drift = [math.hypot(q[1] - n0[1], q[2] - n0[2]) for q in near]
        errs = [q[8] for q in near if q[8] is not None]
        descent = [-q[6] for q in near if q[6] < 0]
        emit('[NEAR-GROUND range<=0.15m] n=%d range(first/last)=%.3f/%.3fm  '
             'relative XY drift(max/end)=%s/%s  XY tracking p95=%s  '
             'measured descent vz median=%s' %
             (len(near), near[0][7], near[-1][7], printable(max(drift)),
              printable(drift[-1]), printable(quantile(errs, .95)),
              printable(median(descent), 'm/s')))
        near10 = [q for q in near if q[7] <= .10]
        emit('[NEAR-GROUND range<=0.10m] n=%d (range is not verified physical clearance)' % len(near10))
    else:
        emit('[NEAR-GROUND] no matching fresh range+ODOM samples in landing interval')

    if c['t265_summary'] is not None:
        t = c['t265_summary']
        emit('[T265 filtered, relative within its own frame] at POINT HOLD: XY=%s Z=%s; '
             'at MAVROS +0.70m: XY=%s Z=%s' %
             (printable(t['at_hold_xy_delta']), printable(t['at_hold_z_delta']),
              printable(t['at_z70_xy_delta']), printable(t['at_z70_z_delta'])))
    if len(d['setpoint']) >= 2:
        gaps = [(b[1] - a[1], b[0] - a[0]) for a, b in zip(d['setpoint'], d['setpoint'][1:])]
        nonnegative = [g for g in gaps if g[0] >= 0]
        if nonnegative:
            emit('[MAVROS SETPOINT] max header gap=%.3fs max recording gap=%.3fs  '
                 'header gaps>=0.25s=%d' %
                 (max(g[0] for g in nonnegative), max(g[1] for g in nonnegative),
                  sum(g[0] >= .25 for g in nonnegative)))

    if d['rosout']:
        emit('--- ROS POINT events (up to 12) ---')
        for t, msg in d['rosout'][:12]:
            emit('  %+.2fs %s' % (t - t_take, msg))

    # Compact time series to expose reporting delay and z-frame corrections.
    emit('--- MAVROS ODOM altitude timeline (relative to takeoff, every 2s) ---')
    finish = c['t_disarm'] or c['t_land'] or d['odom'][-1][0]
    if finish > t_take:
        step = 2
        for sec in range(0, min(150, int(finish - t_take) + 1), step):
            s = nearest(d['odom'], t_take + sec, .22)
            if s is not None:
                emit('  t=%+4ds dz=%+.3fm dXY=%.3fm vz=%+.3fm/s' %
                     (sec, s[3] - gz, math.hypot(s[1] - gx, s[2] - gy), s[6]))

    emit('[LIMITATION] Sensor estimates and flight status can disagree; do not label these as externally measured physical motion.')
    emit('[SAFETY] No change to landing parameters. Sensor range 0.10m is NOT a motor-stop condition.')


def export_csv(path, d, c):
    path.parent.mkdir(parents=True, exist_ok=True)
    status, odom, point, ranges, t265, ext = [d[k] for k in
                                               ('status', 'odom', 'point', 'range', 't265', 'extended')]
    if not odom:
        return
    start = max(odom[0][0], c['t_take'] - 1.5)
    end = min(odom[-1][0], (c['t_disarm'] or odom[-1][0]) + 1)
    with path.open('w', newline='', encoding='utf-8') as f:
        wr = csv.writer(f)
        wr.writerow(('t_unix_s', 'from_takeoff_s', 'status', 'odom_x_m', 'odom_y_m',
                     'odom_dz_m', 'odom_dxy_ground_m', 'odom_vz_m_s', 'point_x_m',
                     'point_y_m', 'point_z_m', 'point_tracking_xy_m', 'range_m',
                     't265_dx_from_takeoff_m', 't265_dy_from_takeoff_m',
                     't265_dz_from_takeoff_m', 'px4_landed_state'))
        prev_status = ''
        k_status = 0
        anchor_t265 = nearest(t265, c['t_take'], .20)
        for j in range(int((end - start) / .1) + 1):
            t = start + j * .1
            while k_status < len(status) and status[k_status][0] <= t:
                prev_status = status[k_status][1]
                k_status += 1
            o = nearest(odom, t, .09)
            pc = nearest(point, t, .09)
            rg = nearest(ranges, t, .12)
            ts = nearest(t265, t, .12)
            landed = last_before(ext, t)
            if o is None:
                continue
            xyerr = math.hypot(o[1] - pc[1], o[2] - pc[2]) if pc else None
            d265 = ([ts[i] - anchor_t265[i] for i in (1, 2, 3)]
                    if ts and anchor_t265 else [None] * 3)
            wr.writerow((round(t, 3), round(t - c['t_take'], 3), prev_status,
                         o[1], o[2], o[3] - c['gz'],
                         math.hypot(o[1] - c['gx'], o[2] - c['gy']), o[6],
                         pc[1] if pc else None, pc[2] if pc else None,
                         pc[3] if pc else None, xyerr,
                         rg[1] if rg else None, *d265,
                         landed[1] if landed and t - landed[0] <= 1 else None))


def main():
    parser = argparse.ArgumentParser(description='Offline read-only Demo9 V2 analysis')
    parser.add_argument('bag', nargs='?', help='Explicit .bag path (default newest finalized Demo9 bag)')
    parser.add_argument('--export-dir', help='Optional directory for report + synchronized CSV')
    args = parser.parse_args()
    folder = Path.home() / 'contest1_uav/data/bags/demo9_takeoff_hover_land_v1'
    if args.bag:
        bag = Path(args.bag).expanduser()
    else:
        files = sorted(folder.glob('demo9_*.bag'), key=lambda p: p.stat().st_mtime, reverse=True)
        if not files:
            raise SystemExit('[ABORT] No finalized Demo9 bag found')
        bag = files[0]
    if not bag.is_file() or bag.suffix != '.bag':
        raise SystemExit('[ABORT] Need an existing finalized .bag file; never analyze .bag.active')
    print('[BAG] %s' % bag)
    print('[BAG SIZE] %.1f MiB' % (bag.stat().st_size / 1048576))
    data, available = collect(bag)
    important = ('status', 'odom', 'point', 'range', 't265', 'extended')
    for name in important:
        topic = next(k for k, v in STREAMS.items() if v == name)
        if topic not in available:
            print('[MISSING TOPIC] %s' % topic)
    try:
        calc = calculate(data)
    except ValueError as exc:
        raise SystemExit('[ABORT] ' + str(exc))
    lines = []
    def emit(line):
        print(line)
        lines.append(line)
    summarize(data, calc, emit)
    if args.export_dir:
        root = Path(args.export_dir).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        report = root / (bag.stem + '_v2_report.txt')
        csvname = root / (bag.stem + '_v2_timeline.csv')
        report.write_text('[BAG] %s\n' % bag + '\n'.join(lines) + '\n', encoding='utf-8')
        export_csv(csvname, data, calc)
        print('[EXPORT] %s' % report)
        print('[EXPORT] %s' % csvname)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
