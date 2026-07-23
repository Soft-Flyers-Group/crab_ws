#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy
)

from camera_info_manager import CameraInfoManager

import gi
import numpy as np
import os
import yaml
import copy

gi.require_version("Gst", "1.0")
from gi.repository import Gst

class StellarCameraNode(Node):
    def __init__(self):
        super().__init__("stellar_camera_node")

        # Initialize Gstreamer
        Gst.init(None)

        # Calibration
        self.calibration_url = ("package://crab_control/config/stellar_hd.yaml")

        # Calibration written here by cameracalibrator
        self.calibration_save_path = os.path.expanduser("~/crab_ws/src/crab_control/config/stellar_hd.yaml")

        # Camera info manager to open the calibration
        self.camera_info_manager = CameraInfoManager(self, cname="stellar_hd", url=self.calibration_url)
        self.camera_info_manager.loadCameraInfo()

        try:
            if self.camera_info_manager.isCalibrated():
                self.get_logger().info("Calibration loaded from share")
            else:
                self.get_logger().warn("No calibration loaded")
        except Exception:
            self.get_logger().warn("No calibration loaded")

        # QoS: for a live camera feed you want the newest frame, not a
        # reliable backlog, so use BEST_EFFORT + depth=1.
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )

        # Publishers
        self.image_pub = self.create_publisher(Image, "/camera/image_raw", camera_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", camera_qos)

        # Camera calibration service
        self.camera_info_service = self.create_service(SetCameraInfo, "/camera/set_camera_info", self.set_camera_info_callback)

        # GStreamer
        camera = "/dev/video0"
        width = 1280
        height = 720
        fps = 30

        # HW-accelerated path on Jetson:
        #  - nvv4l2decoder does MJPEG decode on the NVJPG engine instead of CPU jpegdec
        #  - nvvidconv does the colorspace convert on the VIC engine instead of CPU videoconvert
        # Output is BGRx (4 channels) because that's the cheap conversion target for nvvidconv;
        # we drop the alpha channel in Python before publishing.
        pipeline = f"""
        v4l2src device={camera} io-mode=2 !
        image/jpeg,width={width},height={height},framerate={fps}/1 !
        tee name=t
        t. ! queue max-size-buffers=1 leaky=downstream !
        nvv4l2decoder mjpeg=1 enable-max-performance=1 !
        nvvidconv !
        video/x-raw,format=BGRx !
        appsink name=ros_sink emit-signals=true max-buffers=1 drop=true sync=false
        t. ! queue max-size-buffers=1 leaky=downstream !
        rtpjpegpay !
        udpsink host=10.42.0.1 port=5600 sync=false async=false
        """
        # Create the GST Pipeline
        self.pipeline = Gst.parse_launch(pipeline)
        self.appsink = self.pipeline.get_by_name("ros_sink")
        self.appsink.connect("new-sample", self.new_frame)
        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info("StellarHD camera started (HW decode path)")
        self.get_logger().info(f"Loading calibration: {self.calibration_url}")
        self.get_logger().info(f"Saving calibration: {self.calibration_save_path}")

    def set_camera_info_callback(self, request, response):
        ci = request.camera_info
        data = {
            "image_width": int(ci.width),
            "image_height": int(ci.height),
            "camera_name": "stellar_hd",
            "distortion_model": ci.distortion_model,
            "distortion_coefficients": {
                "rows": 1,
                "cols": len(ci.d),
                "data": [float(x) for x in ci.d]
            },
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": [float(x) for x in ci.k]
            },
            "rectification_matrix": {
                "rows": 3,
                "cols": 3,
                "data": [float(x) for x in ci.r]
            },
            "projection_matrix": {
                "rows": 3,
                "cols": 4,
                "data": [float(x) for x in ci.p]
            }
        }
        os.makedirs(os.path.dirname(self.calibration_save_path), exist_ok=True)
        with open(self.calibration_save_path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)

        self.camera_info_manager.camera_info = ci
        self.get_logger().info("Calibration saved to src config")

        response.success = True
        response.status_message = "Calibration saved"
        return response

    def new_frame(self, sink):
        sample = sink.emit("pull-sample")

        if sample is None:
            return Gst.FlowReturn.ERROR

        buffer = sample.get_buffer()
        caps = sample.get_caps()
        width = caps.get_structure(0).get_value("width")
        height = caps.get_structure(0).get_value("height")
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR
        try:
            # nvvidconv gives us BGRx (4 bytes/px). Build the Image msg
            # directly from the mapped buffer instead of going through
            # cv_bridge, which avoids an extra full-frame copy.
            frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape(height, width, 4)
            bgr = np.ascontiguousarray(frame[:, :, :3])  # drop alpha, one copy (needed for row-major bgr8)

            stamp = self.get_clock().now().to_msg()

            image_msg = Image()
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = "camera_link"
            image_msg.height = height
            image_msg.width = width
            image_msg.encoding = "bgr8"
            image_msg.is_bigendian = 0
            image_msg.step = width * 3
            image_msg.data = bgr.tobytes()

            self.image_pub.publish(image_msg)

            try:
                camera_info = copy.deepcopy(self.camera_info_manager.getCameraInfo())
            except Exception:
                camera_info = CameraInfo()

            camera_info.header.stamp = stamp
            camera_info.header.frame_id = "camera_link"
            camera_info.width = width
            camera_info.height = height

            self.camera_info_pub.publish(camera_info)

        finally:
            buffer.unmap(map_info)

        return Gst.FlowReturn.OK

    def destroy_node(self):
        self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StellarCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()