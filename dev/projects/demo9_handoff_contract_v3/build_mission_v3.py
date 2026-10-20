from pathlib import Path
p=Path(__file__).resolve().parent/'candidates/uav2_demo5_waypoints.py'
s=p.read_text()

def change(old,new,count=1):
    global s
    actual=s.count(old)
    if actual!=count:
        raise RuntimeError('Expected %d instances, found %d: %r' % (count,actual,old[:140]))
    s=s.replace(old,new,count)

change('''        self.mission_allowed = False
''','''        self.mission_allowed = False
        # Arrival times for the recovery-only live safety gate.
        self.state_rx_time = None
        self.health_rx_time = None
        self.permission_rx_time = None
''')
change('''    def state_cb(self, msg):
        self.state = msg
''','''    def state_cb(self, msg):
        self.state = msg
        self.state_rx_time = rospy.Time.now()
''')
change('''    def health_cb(self, msg):
        self.flight_healthy = msg.data
''','''    def health_cb(self, msg):
        self.flight_healthy = msg.data
        self.health_rx_time = rospy.Time.now()
''')
change('''    def mission_cb(self, msg):
        self.mission_allowed = msg.data
''','''    def mission_cb(self, msg):
        self.mission_allowed = msg.data
        self.permission_rx_time = rospy.Time.now()
''')
change('''        # First ensure the downstream interface is in HOLD. It continues
        # publishing its independent 50 Hz HOLD only while odom/health work.
        self.call_service(disable)
''','''        # Disable upstream trajectory and wait for a *fresh actual output*
        # from the downstream 50 Hz HOLD. MUX will use the SAME output as
        # its switch continuity anchor.
        disable_at = rospy.Time.now()
        self.call_service(disable)
''')
change('''            now = rospy.Time.now()
            if self.odom is None or self.odom_time is None:
''','''            now = rospy.Time.now()
            for name, stamp, timeout in (
                    ('PX4 state', self.state_rx_time, 2.50),
                    ('flight health', self.health_rx_time, 0.30),
                    ('RC permission', self.permission_rx_time, 1.50)):
                if stamp is None or not 0.0 <= (now - stamp).to_sec() < timeout:
                    raise RuntimeError('POINT rejoin: %s unavailable/stale' % name)
            if self.odom is None or self.odom_time is None:
''')
change('''                    self.point_status != 'AIRBORNE_HOLD' or
                    'HOLD' not in self.if_status):
                return False, None
''','''                    self.point_status != 'AIRBORNE_HOLD' or
                    self.if_status != 'HOLD_ACTIVE'):
                return False, None
            downstream, downstream_time = self.mavros_sp, self.mavros_sp_time
            if (downstream is None or downstream_time is None or
                    not downstream_time > disable_at or
                    not 0.0 <= (rospy.Time.now() - downstream_time).to_sec() < 0.15 or
                    downstream.coordinate_frame != PositionTarget.FRAME_LOCAL_NED):
                return False, None
            active_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                           PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_YAW)
            if downstream.type_mask & active_mask:
                return False, None
''')
change('''            values = (p.x, p.y, p.z, cmd.position.x, cmd.position.y,
                      cmd.position.z, cmd.yaw, v.x, v.y, v.z)
''','''            values = (p.x, p.y, p.z, cmd.position.x, cmd.position.y,
                      cmd.position.z, cmd.yaw, v.x, v.y, v.z,
                      downstream.position.x, downstream.position.y,
                      downstream.position.z, downstream.yaw)
''')
change('''            speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
            return (dist <= 0.12 and yaw_error <= math.radians(12.0)
                    and speed <= 0.20), stamp
''','''            speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
            hold_dist = math.sqrt(
                (cmd.position.x-downstream.position.x)**2 +
                (cmd.position.y-downstream.position.y)**2 +
                (cmd.position.z-downstream.position.z)**2)
            hold_yaw = abs(wrap(cmd.yaw-downstream.yaw))
            return (dist <= 0.12 and yaw_error <= math.radians(12.0)
                    and speed <= 0.20 and hold_dist <= 0.12 and
                    hold_yaw <= math.radians(12.0)), stamp
''')
change("rospy.logwarn('[HANDOFF_V2] Begin EGO -> POINT HOLD rejoin')", "rospy.logwarn('[HANDOFF_V3] Begin EGO -> POINT HOLD rejoin')")
s=s.replace('[HANDOFF_V2]', '[HANDOFF_V3]')
s=s.replace('DEMO9_HANDOFF_REVIEW_V2 - guarded', 'DEMO9_HANDOFF_REVIEW_V3 - guarded')
compile(s,str(p),'exec')
p.write_text(s)
print('[OK]',p, len(s))
