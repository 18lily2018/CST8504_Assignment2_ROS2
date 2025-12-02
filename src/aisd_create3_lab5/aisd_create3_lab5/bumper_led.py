#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Some systems (like your home VM) may not have the Bumper message.
# This try/except lets the node still start by falling back to another type.
try:
    from irobot_create_msgs.msg import Bumper, LightringLeds
except ImportError:
    # Fallback: use HazardDetectionVector as a stand-in for Bumper
    from irobot_create_msgs.msg import HazardDetectionVector as Bumper, LightringLeds


class BumperLedNode(Node):
    def __init__(self):
        super().__init__('bumper_led_node')

        # Subscribe to bumper (or hazard) events from Create3
        self.bumper_sub = self.create_subscription(
            Bumper,
            '/mobile_base/events/bumper',
            self.bumper_callback,
            10
        )

        # Publisher to control light ring LEDs
        self.led_pub = self.create_publisher(
            LightringLeds,
            '/cmd_lightring',
            10
        )

        self.get_logger().info('Lab5 bumper→LED node running, waiting for bumper press...')

    def bumper_callback(self, msg: Bumper):
        leds = LightringLeds()

        # Simplest behaviour: when we get ANY bumper/hazard message,
        # turn the lights GREEN; otherwise turn them off.
        # On the lab machine you can refine this to check right bumper specifically.
        leds.leds = [LightringLeds.LED_GREEN] * 6
        self.get_logger().info('Received bumper/hazard event → LEDs set to GREEN')

        self.led_pub.publish(leds)


def main(args=None):
    rclpy.init(args=args)
    node = BumperLedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
