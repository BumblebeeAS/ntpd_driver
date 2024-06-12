#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import TimeReference
import time


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

        # Convert the current time to a human-readable format
        readable_time = time.strftime(
            "%Y-%m-%d %H:%M:%S", time.localtime(self.current_time)
        )

        # Create a message and publish it
        msg = TimeReference()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "fake_time_publisher"
        msg.time_ref.sec = int(self.current_time)
        msg.time_ref.nanosec = int((self.current_time - int(self.current_time)) * 1e9)
        self.publisher_.publish(msg)

        # Log the published time
        self.get_logger().info('Publishing: "%s"' % msg)


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
