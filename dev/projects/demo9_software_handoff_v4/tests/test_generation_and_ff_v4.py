"""Targeted generation-time and feedforward checks; actual DEV sources only."""
import math
import unittest
from test_mux_and_interface_v3 import Mux, Stamp, String, cmd
import test_restore_v3 as restore_v3

class V4MuxGuards(unittest.TestCase):
    def setUp(self):
        Stamp.clock=100.0
        self.m=Mux()
        self.m.source='EGO'
        self.m.ego_cb(cmd(1,1,1,0))
        self.m.timer_cb(None)
        self.m.interface_cb(String('HOLD_ACTIVE'))
        self.m.downstream_cb(cmd(1,1,1,0))
        self.m.point_status_cb(String('AIRBORNE_HOLD'))

    def test_header_older_than_receive_time_is_stale(self):
        m=cmd(1,1,1,0);m.header.stamp=Stamp(99.70)
        self.m.point_cb(m)
        self.assertFalse(self.m.use_point(None).success)

    def test_status_must_be_fresh_airborne_hold(self):
        self.m.point_status_cb(String('POINT_TRAJECTORY'))
        self.m.point_cb(cmd(1,1,1,0))
        self.assertIn('AIRBORNE_HOLD',self.m.use_point(None).message)

    def test_nonzero_velocity_refused(self):
        m=cmd(1,1,1,0);m.velocity.x=.30
        self.m.point_cb(m)
        verdict=self.m.use_point(None)
        self.assertFalse(verdict.success)
        self.assertIn('nonzero_velocity',verdict.message)

    def test_nonzero_acceleration_refused(self):
        m=cmd(1,1,1,0);m.acceleration_or_force.y=.30
        self.m.point_cb(m)
        verdict=self.m.use_point(None)
        self.assertFalse(verdict.success)
        self.assertIn('nonzero_acceleration',verdict.message)

    def test_nan_yaw_rate_refused(self):
        m=cmd(1,1,1,0);m.yaw_rate=float('nan')
        self.m.point_cb(m)
        verdict=self.m.use_point(None)
        self.assertFalse(verdict.success)
        self.assertIn('nonfinite_yaw_rate',verdict.message)

    def test_out_of_order_point_sample_does_not_override_new(self):
        current=cmd(1,1,1,0)
        self.m.point_cb(current)
        Stamp.clock+=.02
        old=cmd(1,0,1,-90);old.header.stamp=Stamp(99.5)
        self.m.point_cb(old)
        self.assertAlmostEqual(self.m.point_msg.position.y,1)
        self.assertTrue(self.m.use_point(None).success)

    def test_nonzero_feedforward_after_handoff_is_blocked(self):
        self.m.point_cb(cmd(1,1,1,0));self.assertTrue(self.m.use_point(None).success)
        self.m.timer_cb(None)
        before=len(self.m.pub.messages)
        bad=cmd(1,1,1,0);bad.velocity.x=.3
        self.m.point_cb(bad);self.m.timer_cb(None)
        self.assertEqual(len(self.m.pub.messages),before)
        self.assertEqual(self.m.status_pub.messages[-1].data,'POINT_GUARD_REJECTED')

class V4MissionGuards(unittest.TestCase):
    def make(self,scenario):
        return restore_v3.RestoreActualTests().make(scenario)
    def test_generated_before_abort_but_received_after_is_rejected(self):
        t,fn=self.make('old_generation')
        with self.assertRaisesRegex(RuntimeError,'fresh current HOLD'):fn()
        self.assertNotIn('use_point',t.services)
    def test_same_generated_point_retransmitted_three_times_does_not_qualify(self):
        t,fn=self.make('duplicate_generation')
        with self.assertRaisesRegex(RuntimeError,'fresh current HOLD'):fn()
        self.assertNotIn('use_point',t.services)
    def test_nonzero_point_hold_velocity_never_reenables(self):
        t,fn=self.make('point_nonzero_velocity')
        with self.assertRaisesRegex(RuntimeError,'fresh current HOLD'):fn()
        self.assertNotIn('enable_trajectory',t.services)
    def test_no_new_point_status_never_reenables(self):
        t,fn=self.make('stale_point_status')
        with self.assertRaisesRegex(RuntimeError,'fresh current HOLD'):fn()
        self.assertNotIn('enable_trajectory',t.services)
    def test_mavros_echo_cannot_mask_dead_point_stream_after_enable(self):
        t,fn=self.make('no_fresh_point_after_enable')
        with self.assertRaisesRegex(RuntimeError,'MAVROS POINT output not verified'):fn()
        self.assertEqual(t.services[-1],'disable_trajectory')

if __name__=='__main__':unittest.main()
