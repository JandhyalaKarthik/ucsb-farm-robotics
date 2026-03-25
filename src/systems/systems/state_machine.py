import rclpy
from rclpy.node import Node
import smach
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time

# ==========================================
# 1. The ROS 2 Nervous System (Pub/Sub)
# ==========================================
class FarmBotSystemsNode(Node):
    def __init__(self):
        super().__init__('farm_bot_state_machine')
        
        # Publishers: Sending commands OUT
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers: Listening to other teams IN
        # Listening to Emma's CV Node
        self.create_subscription(Bool, '/spray_cmd', self.cv_callback, 10)
        # Listening to Branden's Navigation Node (assuming a custom topic for end-of-row)
        self.create_subscription(Bool, '/end_of_row', self.nav_callback, 10)
        # Listening to the Watchdog (which we will build next)
        self.create_subscription(Bool, '/e_stop', self.fault_callback, 10)

        # State Flags (These change when messages arrive)
        self.start_system = True # Defaulting to True to auto-start for testing
        self.pest_detected = False
        self.reached_end_of_row = False
        self.e_stop_triggered = False

    def cv_callback(self, msg):
        if msg.data:
            self.pest_detected = True

    def nav_callback(self, msg):
        if msg.data:
            self.reached_end_of_row = True

    def fault_callback(self, msg):
        if msg.data:
            self.e_stop_triggered = True

    def halt_robot(self):
        """Helper function to slam the brakes"""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)


# ==========================================
# 2. Define the Robot's States
# ==========================================

class Idle(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['start_cmd_received'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("[STATE: IDLE] Waiting for start command...")
        while not self.node.start_system:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return 'start_cmd_received'


class NavigatingRow(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['pest_detected', 'reached_end_of_row', 'fatal_error'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("[STATE: NAVIGATING] Moving through the row...")
        
        # Keep spinning to listen for incoming ROS 2 messages
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if self.node.e_stop_triggered:
                return 'fatal_error'
            elif self.node.pest_detected:
                return 'pest_detected'
            elif self.node.reached_end_of_row:
                return 'reached_end_of_row'


class Spraying(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['spray_complete'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("[STATE: SPRAYING] Pest detected! Halting motors to spray.")
        
        # 1. Stop the robot so the pesticide hits the target
        self.node.halt_robot()
        
        # 2. Wait for the spray to finish (Aylin's Pico handles the physical valve)
        time.sleep(2.0) # Simulating a 2-second spray burst
        
        # 3. Reset the flag and go back to driving
        self.node.pest_detected = False
        return 'spray_complete'


class Turning(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['turn_complete', 'fatal_error'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("[STATE: TURNING] End of row reached. Executing U-Turn.")
        self.node.halt_robot()
        
        time.sleep(3.0) # Simulating the time it takes Branden's code to turn the robot
        
        self.node.reached_end_of_row = False
        return 'turn_complete'


class Fault(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=[]) # No outcomes, this is a dead end
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().error("[STATE: FAULT] E-STOP TRIGGERED! Killing all motors.")
        # Spam the stop command just to be safe
        for _ in range(5):
            self.node.halt_robot()
            time.sleep(0.1)
            
        # Trap the state machine here permanently until physically reset
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=1.0)


# ==========================================
# 3. Build the State Machine
# ==========================================

def main():
    rclpy.init()
    farm_node = FarmBotSystemsNode()

    farm_bot_sm = smach.StateMachine(outcomes=['e_stop_triggered'])

    # Pass the ROS node into each state so they can read the sensors
    with farm_bot_sm:
        smach.StateMachine.add('IDLE', Idle(farm_node), 
                               transitions={'start_cmd_received': 'NAVIGATING_ROW'})
        
        smach.StateMachine.add('NAVIGATING_ROW', NavigatingRow(farm_node), 
                               transitions={'pest_detected': 'SPRAYING',
                                            'reached_end_of_row': 'TURNING',
                                            'fatal_error': 'e_stop_triggered'})
        
        smach.StateMachine.add('SPRAYING', Spraying(farm_node), 
                               transitions={'spray_complete': 'NAVIGATING_ROW'})
                               
        smach.StateMachine.add('TURNING', Turning(farm_node), 
                               transitions={'turn_complete': 'NAVIGATING_ROW',
                                            'fatal_error': 'e_stop_triggered'})

    # Start the state machine
    farm_bot_sm.execute()
    
    farm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()