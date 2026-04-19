import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu # Standard ROS 2 message for IMU data

class WatchdogNode(Node):
    def __init__(self):
        super().__init__('systems_watchdog')
        
        # Publishers: Sending the kill signals
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.e_stop_pub = self.create_publisher(Bool, '/e_stop', 10)
        
        # Subscribers: Listening to the hardware "heartbeats"
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Bool, '/pico/heartbeat', self.pico_callback, 10)
        
        # Timestamps to track the last time we heard from the sensors
        self.last_imu_time = self.get_clock().now()
        self.last_pico_time = self.get_clock().now()
        
        # The strict 200ms limit set by your team lead
        self.timeout_duration = rclpy.time.Duration(seconds=0.2)
        
        # The Watchdog Timer: This loop runs 20 times a second (every 50ms) to check the clocks
        self.timer = self.create_timer(0.05, self.check_timeouts)
        self.e_stop_active = False
        
        self.get_logger().info("Safety Watchdog initialized. Monitoring sensors...")

    def imu_callback(self, msg):
        """Every time Branden's LIDAR/IMU node publishes data, reset the IMU clock."""
        self.last_imu_time = self.get_clock().now()

    def pico_callback(self, msg):
        """Every time Aylin's Pico sends a pulse, reset the Pico clock."""
        self.last_pico_time = self.get_clock().now()

    def check_timeouts(self):
        """The main security loop that checks for dead connections."""
        if self.e_stop_active:
            # CONTINUOUSLY slam the brakes so Nav2 cannot override the halt command
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_msg)
            return 

        now = self.get_clock().now()
        imu_silence = now - self.last_imu_time
        pico_silence = now - self.last_pico_time

        if imu_silence > self.timeout_duration or pico_silence > self.timeout_duration:
            self.trigger_e_stop(imu_silence > self.timeout_duration)

    def trigger_e_stop(self, imu_failed):
        """The emergency protocol."""
        self.e_stop_active = True
        
        if imu_failed:
            self.get_logger().fatal("WATCHDOG TRIGGERED: Lost IMU connection!")
        else:
            self.get_logger().fatal("WATCHDOG TRIGGERED: Lost Pico serial connection!")

        # 1. Alert the State Machine so it switches to the FAULT state
        e_stop_msg = Bool()
        e_stop_msg.data = True
        self.e_stop_pub.publish(e_stop_msg)

        # 2. Hard override: Slam the hardware brakes immediately
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().fatal("Zero-velocity command published. Robot halted.")

def main(args=None):
    rclpy.init(args=args)
    node = WatchdogNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()