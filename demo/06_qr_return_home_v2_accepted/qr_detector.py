#!/usr/bin/env python3

import rospy
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
from pyzbar.pyzbar import decode


class QRDetector:

    def __init__(self):

        self.bridge = CvBridge()

        # -------------------------
        # Parameters
        # -------------------------

        self.process_rate = rospy.get_param(
            "~process_rate",
            5.0
        )

        self.stable_count_required = rospy.get_param(
            "~stable_count",
            3
        )


        # -------------------------
        # State
        # -------------------------

        self.last_process_time = rospy.Time(0)

        self.last_candidate = ""

        self.stable_count = 0

        self.detected_text = ""


        # -------------------------
        # ROS
        # -------------------------

        self.result_pub = rospy.Publisher(
            "/contest1/qr_detection/result",
            String,
            queue_size=1,
            latch=True
        )

        self.status_pub = rospy.Publisher(
            "/contest1/qr_detection/status",
            String,
            queue_size=1,
            latch=True
        )


        rospy.Subscriber(
            "/contest1/camera/color/image_raw",
            Image,
            self.image_callback,
            queue_size=1
        )


        self.status_pub.publish(
            String("SEARCHING")
        )


        rospy.loginfo(
            "DEMO6 QR detector phase2 started"
        )

        rospy.loginfo(
            "process_rate=%.1f Hz stable_count=%d",
            self.process_rate,
            self.stable_count_required
        )


    def image_callback(self, msg):

        now = rospy.Time.now()


        # limit QR processing rate
        if (
            now - self.last_process_time
        ).to_sec() < (
            1.0 / self.process_rate
        ):
            return


        self.last_process_time = now


        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                "bgr8"
            )

        except Exception as e:

            rospy.logerr_throttle(
                5,
                "cv_bridge error: %s",
                e
            )

            return


        results = decode(frame)


        if not results:

            return


        for r in results:

            try:

                text = r.data.decode(
                    "utf-8"
                )

            except Exception:

                continue


            if not text:

                continue


            # -------------------------
            # stability check
            # -------------------------

            if text == self.last_candidate:

                self.stable_count += 1

            else:

                self.last_candidate = text
                self.stable_count = 1


            if (
                self.stable_count
                >= self.stable_count_required
            ):

                if text != self.detected_text:

                    self.detected_text = text


                    self.result_pub.publish(
                        String(text)
                    )


                    rospy.logwarn(
                        "[QR] detected: %s",
                        text
                    )


                self.status_pub.publish(
                    String("DETECTED")
                )

                return


        self.status_pub.publish(
            String("SEARCHING")
        )


if __name__ == "__main__":

    rospy.init_node(
        "demo6_qr_detector"
    )

    QRDetector()

    rospy.spin()
