"""
Dance command publisher for Create 3 robot
"""

from rclpy.node import Node
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import LightringLeds
from .choreographer import DanceChoreographer, Move, Lights, FinishedDance


class DanceCommandPublisher(Node):
    def __init__(
        self,
        dance_choreographer: DanceChoreographer,
        node_name: str = "dance_command_publisher",
    ):
        super().__init__(node_name)
        self.dance_choreographer = dance_choreographer
        self.lights_publisher = self.create_publisher(
            LightringLeds, "cmd_lightring", 10
        )
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.last_twist = Twist()
        self.last_lightring = LightringLeds()
        self.last_lightring.override_system = False
        self.ready = False
        self.last_wait_subscriber_printout = None
        self.finished = False

    def timer_callback(self):
        if self.finished:
            return
        current_time = self.get_clock().now()

        # Wait for subscribers before starting dance
        if not self.ready:
            # Check if subscribers are ready
            if (
                self.vel_publisher.get_subscription_count() > 0
                and self.lights_publisher.get_subscription_count() > 0
            ):
                # Subscribers are connected, we can start the dance
                self.ready = True
                self.get_logger().info(
                    f"Starting dance at time {current_time.nanoseconds / 1e9}"
                )
                self.dance_choreographer.start_dance(current_time)
                return

            # Periodic logging while waiting for subscribers
            elif (
                not self.last_wait_subscriber_printout
                or (
                    (current_time - self.last_wait_subscriber_printout).nanoseconds
                    / 1e9
                )
                > 5.0
            ):
                self.last_wait_subscriber_printout = current_time
                self.get_logger().info(
                    "Waiting for publishers to connect to subscribers"
                )
                return
            else:
                return
        next_actions = self.dance_choreographer.get_next_actions(current_time)
        twist = self.last_twist
        lightring = self.last_lightring

        for next_action in next_actions:
            if isinstance(next_action, Move):
                twist = Twist()
                twist.linear.x = next_action.x
                twist.angular.z = next_action.theta
                self.last_twist = twist
                self.get_logger().debug(
                    f"Time {current_time.nanoseconds / float(1e9)} New move action: {twist.linear.x}, {twist.angular.z}"
                )

            elif isinstance(next_action, Lights):
                lightring = LightringLeds()
                lightring.override_system = True
                lightring.leds = next_action.led_colors
                self.last_lightring = lightring
                self.get_logger().debug(
                    f"Time {current_time.nanoseconds / float(1e9)} New lights action, first led ({lightring.leds[0].red},{lightring.leds[0].green},{lightring.leds[0].blue})"
                )

            else:  # FinishedDance
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.last_twist = twist
                lightring = LightringLeds()
                lightring.override_system = False
                self.last_lightring = lightring
                self.finished = True
                self.get_logger().info(
                    f"Time {current_time.nanoseconds / float(1e9)} Finished Dance Sequence"
                )
                raise FinishedDance

        lightring.header.stamp = current_time.to_msg()
        self.vel_publisher.publish(twist)
        self.lights_publisher.publish(lightring)
