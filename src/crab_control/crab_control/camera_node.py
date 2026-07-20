#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import gi
import numpy as np
import cv2

gi.require_version('Gst', '1.0')
from gi.repository import Gst


class StellarCameraNode(Node):

    def __init__(self):

        super().__init__(
            'stellar_camera_node'
        )

        Gst.init(None)

        self.bridge = CvBridge()

        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        # Change this if needed
        camera = "/dev/video0"

        width = 1280
        height = 720
        fps = 30

        #
        # Single GStreamer pipeline
        #
        pipeline = f"""
        v4l2src device={camera} io-mode=2 !
        image/jpeg,width={width},height={height},framerate={fps}/1 !
        tee name=t

        t. !
        queue !
        jpegdec !
        videoconvert !
        video/x-raw,format=BGR !
        appsink name=ros_sink
        emit-signals=true
        max-buffers=1
        drop=true

        t. !
        queue !
        rtpjpegpay !
        udpsink host=10.42.0.1 port=5600
        """


        self.pipeline = Gst.parse_launch(
            pipeline
        )


        self.appsink = (
            self.pipeline
            .get_by_name("ros_sink")
        )


        self.appsink.connect(
            "new-sample",
            self.new_frame
        )


        self.pipeline.set_state(
            Gst.State.PLAYING
        )


        self.get_logger().info(
            "StellarHD camera started"
        )

        self.get_logger().info(
            "Streaming RTP JPEG to 10.42.0.1:5600"
        )


    def new_frame(self, sink):

        sample = sink.emit(
            "pull-sample"
        )

        if sample is None:
            return Gst.FlowReturn.ERROR


        buffer = sample.get_buffer()

        caps = sample.get_caps()

        width = caps.get_structure(0).get_value(
            "width"
        )

        height = caps.get_structure(0).get_value(
            "height"
        )


        success, map_info = buffer.map(
            Gst.MapFlags.READ
        )

        if not success:
            return Gst.FlowReturn.ERROR


        try:

            data = np.frombuffer(
                map_info.data,
                dtype=np.uint8
            )


            frame = data.reshape(
                (height, width, 3)
            )


            msg = self.bridge.cv2_to_imgmsg(
                frame,
                encoding="bgr8"
            )


            msg.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            msg.header.frame_id = (
                "camera_link"
            )


            self.image_pub.publish(
                msg
            )


        finally:

            buffer.unmap(
                map_info
            )


        return Gst.FlowReturn.OK



    def destroy_node(self):

        self.pipeline.set_state(
            Gst.State.NULL
        )

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
        rclpy.shutdown()



if __name__ == "__main__":
    main()
