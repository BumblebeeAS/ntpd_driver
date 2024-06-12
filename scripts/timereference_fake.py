#!/usr/bin/env python3

"""Copyright (c) 2014-2021 Vladimir Ermakov. All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

       * Redistributions of source code must retain the above copyright
         notice, this list of conditions and the following disclaimer.

       * Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in the
         documentation and/or other materials provided with the distribution.

       * Neither the name of the copyright holder nor the names of its
         contributors may be used to endorse or promote products derived from
         this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
"""
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import TimeReference


class FakeTimePublisher(Node):
    def __init__(self):
        super().__init__("fake_time_publisher")

        # Publisher to publish the fake time
        self.publisher_ = self.create_publisher(TimeReference, "fake_time", 10)
        self.declare_parameter("init_offset_time", 60.0)

        # Initialize start time
        self.current_time = (
            time.time() + self.get_parameter("init_offset_time").value
        )

        # Timer to call the publish_time method every second
        self.timer = self.create_timer(1.0, self.publish_time)

    def publish_time(self):
        # Increment the current time by 1 second
        self.current_time += 1

        # Create a message and publish it
        msg = TimeReference()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "fake_time_publisher"
        msg.time_ref.sec = int(self.current_time)
        msg.time_ref.nanosec = int((
            self.current_time - int(self.current_time)) * 1e9)
        self.publisher_.publish(msg)

        # Log the published time
        self.get_logger().info(f'Publishing: {msg}')


def main(args=None):
    rclpy.init(args=args)

    # Create the node and spin it to keep it alive
    fake_time_publisher = FakeTimePublisher()

    try:
        rclpy.spin(fake_time_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown the ROS 2 node gracefully
        fake_time_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
