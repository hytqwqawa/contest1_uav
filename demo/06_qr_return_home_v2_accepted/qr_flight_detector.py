#!/usr/bin/env python3
"""Flight-stage QR-only RGB subscriber: decode <=5Hz, report exactly 1Hz.

Started only AFTER the proven POINT arrival/HOLD; no latched result.
Does not send any flight command or subscribe to depth images.
"""
import threading
import time
import rospy
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode, ZBarSymbol
from sensor_msgs.msg import Image
from std_msgs.msg import String
from qr_stability import QrStability

TOPIC = '/contest1/demo6/qr/live_result'
STATUS = '/contest1/demo6/qr/status'


class Detector:
    def __init__(self):
        self.rate = float(rospy.get_param('~process_rate', 5.0))
        self.required = int(rospy.get_param('~stable_count', 3))
        if not (0.5 <= self.rate <= 5.0 and 3 <= self.required <= 10):
            raise ValueError('Invalid QR detector parameters')
        self.bridge = CvBridge()
        self.qr = QrStability(self.required)
        self.lock = threading.Lock()
        self.last_process = 0.0
        self.out = rospy.Publisher(TOPIC, String, queue_size=1, latch=False)
        self.status = rospy.Publisher(STATUS, String, queue_size=1, latch=False)
        rospy.Subscriber('/contest1/camera/color/image_raw', Image,
                         self.on_image, queue_size=1, buff_size=4*1024*1024)
        rospy.Timer(rospy.Duration(1.0), self.on_second)
        rospy.loginfo('[DEMO6 QR] RGB subscribe, %.1fHz decode, %d-frame stability, 1Hz output',
                      self.rate, self.required)

    def on_image(self, msg):
        now = time.monotonic()
        with self.lock:
            if now - self.last_process < 1.0 / self.rate:
                return
            self.last_process = now
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            objects = decode(frame, symbols=[ZBarSymbol.QRCODE])
            values = []
            for obj in objects:
                try:
                    value = obj.data.decode('utf-8', errors='strict')
                except UnicodeError:
                    continue
                if value and len(value) <= 512:
                    values.append(value)
            with self.lock:
                self.qr.process(values, time.monotonic())
        except Exception as exc:
            rospy.logerr_throttle(5.0, '[DEMO6 QR] image/decode error: %s', str(exc))
            with self.lock:
                self.qr.process([], time.monotonic())

    def on_second(self, _event):
        with self.lock:
            value = self.qr.current(time.monotonic())
            status = self.qr.status(time.monotonic())
        self.out.publish(String(data=value))
        self.status.publish(String(data=status))
        if value:
            rospy.logwarn('[DEMO6 QR] result=%r status=DETECTED', value)
        else:
            rospy.logwarn('[DEMO6 QR] result=NOT_FOUND status=%s', status)


if __name__ == '__main__':
    rospy.init_node('demo6_qr_flight_detector', anonymous=False)
    Detector()
    rospy.spin()
