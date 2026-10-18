#!/usr/bin/env python3
"""Read-only Demo9 V3 bag summary for TAKEOFF/HOLD/PILOT-LAND.

All spatial values are estimates (not independent physical measurement).
The range sensor's stated min/max are respected; never infer safe motor-stop.
"""
import argparse
import math
import statistics
from pathlib import Path

import rosbag


def first_after(rows, pred, lower):
    return next((t for t, msg in rows if t >= lower and pred(msg)), None)


def tlabel(v, zero):
    return ('N/A' if v is None else '%+.2fs' % (v-zero))


def describe(name, rows, origin, t0, t1):
    selected = [x for x in rows if t0 <= x[0] <= t1]
    if not selected:
        print(name, 'NO MATCHING ODOM'); return
    disp = [math.hypot(p[0]-origin[0],p[1]-origin[1]) for _,p in selected]
    z = [p[2]-origin[2] for _,p in selected]
    print('[%s] n=%d maxXY=%.3fm medianXY=%.3fm Zrange=[%.3f,%.3f]m' %
          (name,len(selected),max(disp),statistics.median(disp),min(z),max(z)))


def run(path):
    topics={
        'odom':'/contest1/mavros/local_position/odom',
        'state':'/contest1/mavros/state',
        'extended':'/contest1/mavros/extended_state',
        'point':'/contest1/ego_uav_interface/point_traj_status',
        'range':'/contest1/mavros/distance_sensor/hrlv_ez4_pub',
        'sp':'/contest1/mavros/setpoint_raw/local',
    }
    rows={k:[] for k in topics}
    names={v:k for k,v in topics.items()}
    with rosbag.Bag(str(path), 'r') as b:
        for topic,msg,stamp in b.read_messages(topics=list(names)):
            rows[names[topic]].append((stamp.to_sec(),msg))
    if len(rows['odom'])<20 or len(rows['state'])<4 or not rows['point']:
        raise SystemExit('[STOP] Not enough V3 flight data in this bag')
    t0=next((t for t,s in rows['point'] if s.data=='TAKEOFF_TRAJECTORY'),None)
    if t0 is None:
        raise SystemExit('[STOP] No POINT TAKEOFF phase; no flight metrics')
    odom=[(t,(m.pose.pose.position.x,m.pose.pose.position.y,
              m.pose.pose.position.z,m.twist.twist.linear.z)) for t,m in rows['odom']]
    before=[p for t,p in odom if t0-1.5<=t<=t0]
    if len(before)<5:
        before=[p for _,p in odom[:20]]
    ground=tuple(statistics.median(p[i] for p in before) for i in range(3))
    th05=next((t for t,p in odom if t>=t0 and p[2]-ground[2]>=.50),None)
    th07=next((t for t,p in odom if t>=t0 and p[2]-ground[2]>=.70),None)
    point_hold=first_after(rows['point'],lambda m: m.data=='AIRBORNE_HOLD',t0)
    takeover=first_after(rows['state'],lambda m: m.armed and m.mode=='POSCTL',t0)
    if takeover is None:
        # Could have disarmed without entering POSCTL; report but never infer a landing.
        takeover=first_after(rows['state'],lambda m: not m.armed,t0)
    ext_ground=first_after(rows['extended'],lambda m: m.landed_state==1,
                           takeover if takeover is not None else t0)
    disarm=first_after(rows['state'],lambda m: not m.armed,t0)

    print('[READ ONLY] Bag=',path)
    print('[COUNTS]', ' '.join('%s=%d'%(k,len(v)) for k,v in rows.items()))
    print('[GROUND REF] x=%.3f y=%.3f z=%.3f (odometry estimate)' % ground)
    print('[PHASE] TAKEOFF=%s POINT_HOLD=%s +0.50m=%s +0.70m=%s' %
          (tlabel(t0,t0),tlabel(point_hold,t0),tlabel(th05,t0),tlabel(th07,t0)))
    print('[PHASE] PILOT_POSCTL=%s PX4_GROUND=%s DISARM=%s' %
          (tlabel(takeover,t0),tlabel(ext_ground,t0),tlabel(disarm,t0)))
    if th07 is not None:
        describe('TAKEOFF to +0.70m',odom,ground,t0,th07)
    if th07 is not None and takeover is not None and takeover>th07:
        describe('VERIFIED-HEIGHT/HOVER to pilot takeover',odom,ground,th07,takeover)
    if takeover is not None:
        end=ext_ground or disarm or odom[-1][0]
        describe('PILOT descent to PX4 ground/last',odom,ground,takeover,end)
        near=[(t,p) for t,p in odom if takeover<=t<=end and
              -0.05<=p[2]-ground[2]<=.15]
        if near:
            disp=[math.hypot(p[0]-ground[0],p[1]-ground[1]) for _,p in near]
            print('[PILOT near-ground Z estimate] n=%d maxXY=%.3fm (Z estimate is not clearance)' %
                  (len(near),max(disp)))
    bad, valid=0,0
    for t,r in rows['range']:
        if takeover is None or t<takeover:
            continue
        if math.isfinite(r.range) and r.min_range>0 and r.max_range>r.min_range and r.min_range<=r.range<=r.max_range:
            valid+=1
        else:
            bad+=1
    print('[RANGE after PILOT takeover] IN_SPEC=%d OUTSIDE_SPEC_OR_INVALID=%d' % (valid,bad))
    header=[m.header.stamp.to_sec() for t,m in rows['sp'] if t>=t0 and
            (takeover is None or t<=takeover) and m.header.stamp.to_sec()>0]
    if len(header)>1:
        gaps=[b-a for a,b in zip(header,header[1:])]
        print('[SETPOINT before pilot takeover] max_gap=%.3fs gaps>=0.25s=%d' %
              (max(gaps),sum(g>=.25 for g in gaps)))
    print('[CAUTION] All positions are estimates. No range-derived motor stop or physical-contact claim.')


if __name__=='__main__':
    a=argparse.ArgumentParser()
    a.add_argument('bag',help='Path to a finalized Demo9 V3 bag')
    args=a.parse_args()
    p=Path(args.bag).expanduser()
    if not p.is_file():
        raise SystemExit('[STOP] Bag does not exist (or recording not finalized): '+str(p))
    run(p)
