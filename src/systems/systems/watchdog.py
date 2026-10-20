import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu # Standard ROS 2 message for IMU data

# Latched e-stop: late subscribers (e.g. the state machine starting after us) still get the last value
E_STOP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class WatchdogNode(Node):
    def __init__(self):
        super().__init__('systems_watchdog')

        # Max silence allowed on a sensor topic once it has started publishing
        self.declare_parameter('timeout_sec', 0.2)
        # Time every sensor gets after boot to send its first message
        self.declare_parameter('startup_grace_sec', 5.0)
        self.timeout_duration = rclpy.time.Duration(
            seconds=self.get_parameter('timeout_sec').value)
        self.startup_grace = rclpy.time.Duration(
            seconds=self.get_parameter('startup_grace_sec').value)

        # Publishers: Sending the kill signals.
        # /cmd_vel_safety is the highest-priority input to twist_mux, so these zeros always win over Nav2.
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_safety', 10)
        self.e_stop_pub = self.create_publisher(Bool, '/e_stop', E_STOP_QOS)

        # Subscribers: Listening to the hardware "heartbeats"
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Bool, '/pico/heartbeat', self.pico_callback, 10)

        # None until the first message arrives, so a sensor that is still booting isn't treated as dead
        self.start_time = self.get_clock().now()
        self.last_seen = {'IMU': None, 'Pico': None}

        # The Watchdog Timer: This loop runs 20 times a second (every 50ms) to check the clocks
        self.timer = self.create_timer(0.05, self.check_timeouts)
        self.e_stop_active = False
        self.publish_e_stop(False)

        self.get_logger().info("Safety Watchdog initialized. Monitoring sensors...")

    def imu_callback(self, msg):
        self.last_seen['IMU'] = self.get_clock().now()

    def pico_callback(self, msg):
        self.last_seen['Pico'] = self.get_clock().now()

    def publish_e_stop(self, active):
        msg = Bool()
        msg.data = active
        self.e_stop_pub.publish(msg)

    def publish_zero_velocity(self):
        self.cmd_vel_pub.publish(Twist())

    def check_timeouts(self):
        """The main security loop that checks for dead connections."""
        if self.e_stop_active:
            # Keep re-asserting both signals so nothing downstream can drift out of the halt
            self.publish_e_stop(True)
            self.publish_zero_velocity()
            return

        now = self.get_clock().now()
        for name, last in self.last_seen.items():
            if last is None:
                if now - self.start_time > self.startup_grace:
                    self.trigger_e_stop(f"{name} never came online")
                    return
            elif now - last > self.timeout_duration:
                self.trigger_e_stop(f"Lost {name} connection")
                return

    def trigger_e_stop(self, reason):
        """The emergency protocol."""
        self.e_stop_active = True
        self.get_logger().fatal(f"WATCHDOG TRIGGERED: {reason}!")

        # 1. Alert the State Machine (and lock twist_mux) so it switches to the FAULT state
        self.publish_e_stop(True)

        # 2. Hard override: Slam the hardware brakes immediately
        self.publish_zero_velocity()
        self.get_logger().fatal("Zero-velocity command published. Robot halted.")

def main(args=None):
    rclpy.init(args=args)
    node = WatchdogNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
