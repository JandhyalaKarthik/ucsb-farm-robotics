import rclpy
from rclpy.node import Node
import smach
from std_msgs.msg import Bool, String, Int32
from geometry_msgs.msg import Twist
import time
from enum import Enum

class SprayType(Enum):
    REGULAR = "regular"
    PESTICIDE = "pesticide"

# ==========================================
# 1. Enhanced ROS 2 Node with Spray Control
# ==========================================
class EnhancedFarmBotNode(Node):
    def __init__(self):
        super().__init__('enhanced_farm_bot_state_machine')
        
        # Publishers: Sending commands OUT
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.spray_control_pub = self.create_publisher(String, '/spray_control', 10)
        self.spray_type_pub = self.create_publisher(String, '/spray_type', 10)
        
        # Subscribers: Listening to other nodes
        self.create_subscription(Bool, '/pesticide_detected', self.cv_pesticide_callback, 10)
        self.create_subscription(Bool, '/end_of_row', self.nav_callback, 10)
        self.create_subscription(Bool, '/row_detected', self.row_detection_callback, 10)
        self.create_subscription(Bool, '/e_stop', self.fault_callback, 10)
        self.create_subscription(Bool, '/start_system', self.start_callback, 10)

        # State Flags
        self.system_started = False
        self.pesticide_detected = False
        self.reached_end_of_row = False
        self.row_detected_after_turn = False
        self.e_stop_triggered = False
        
        # Timing configurations
        self.idle_time = 5.0  # seconds to wait in idle before starting
        self.end_time_limit = 30.0  # seconds to wait for new row after turn
        self.pesticide_spray_duration = 2.0  # seconds to spray pesticide
        
        # Current spray state
        self.current_spray_type = SprayType.REGULAR
        self.is_spraying = False

    def cv_pesticide_callback(self, msg):
        """Computer vision detected pesticides"""
        if msg.data:
            self.pesticide_detected = True
            self.get_logger().info("Pesticide detected by computer vision!")

    def nav_callback(self, msg):
        """Navigation system detected end of row"""
        if msg.data:
            self.reached_end_of_row = True

    def row_detection_callback(self, msg):
        """Row detection after turning"""
        if msg.data:
            self.row_detected_after_turn = True
        else:
            self.row_detected_after_turn = False

    def fault_callback(self, msg):
        """Emergency stop triggered"""
        if msg.data:
            self.e_stop_triggered = True

    def start_callback(self, msg):
        """System start command"""
        if msg.data:
            self.system_started = True

    def halt_robot(self):
        """Stop all robot movement"""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)

    def start_continuous_spray(self, spray_type=SprayType.REGULAR):
        """Start continuous spraying"""
        if not self.is_spraying or self.current_spray_type != spray_type:
            self.current_spray_type = spray_type
            self.is_spraying = True
            
            # Send spray commands
            spray_msg = String()
            spray_msg.data = "start"
            self.spray_control_pub.publish(spray_msg)
            
            type_msg = String()
            type_msg.data = spray_type.value
            self.spray_type_pub.publish(type_msg)
            
            self.get_logger().info(f"Started continuous {spray_type.value} spraying")

    def stop_spray(self):
        """Stop all spraying"""
        if self.is_spraying:
            self.is_spraying = False
            spray_msg = String()
            spray_msg.data = "stop"
            self.spray_control_pub.publish(spray_msg)
            self.get_logger().info("Stopped spraying")

    def switch_spray_type(self, spray_type):
        """Switch to different spray type"""
        if self.current_spray_type != spray_type:
            self.current_spray_type = spray_type
            type_msg = String()
            type_msg.data = spray_type.value
            self.spray_type_pub.publish(type_msg)
            self.get_logger().info(f"Switched to {spray_type.value} spray")


# ==========================================
# 2. Enhanced State Definitions
# ==========================================

class IdleWithTimer(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['start_farming'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info(f"[STATE: IDLE] Waiting {self.node.idle_time} seconds before starting...")
        
        # Stop any existing spray
        self.node.stop_spray()
        
        start_time = time.time()
        while time.time() - start_time < self.node.idle_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.node.e_stop_triggered:
                return 'start_farming'  # Will be caught by emergency handling
        
        self.node.get_logger().info("[STATE: IDLE] Idle time complete, starting farming operations")
        return 'start_farming'


class NavigatingWithContinuousSpray(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['pesticide_detected', 'reached_end_of_row', 'fatal_error'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("[STATE: NAVIGATING] Moving through row with continuous spraying...")
        
        # Start continuous regular spraying
        self.node.start_continuous_spray(SprayType.REGULAR)
        
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if self.node.e_stop_triggered:
                self.node.stop_spray()
                return 'fatal_error'
            elif self.node.pesticide_detected:
                return 'pesticide_detected'
            elif self.node.reached_end_of_row:
                # Stop spraying before turning
                self.node.stop_spray()
                return 'reached_end_of_row'


class PesticideTreatment(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['treatment_complete', 'fatal_error'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("[STATE: PESTICIDE_TREATMENT] Treating detected pesticides...")
        
        # Step 1: Stop the robot
        self.node.halt_robot()
        
        # Step 2: Switch to pesticide spray and spray for specified duration
        self.node.switch_spray_type(SprayType.PESTICIDE)
        self.node.start_continuous_spray(SprayType.PESTICIDE)
        
        start_time = time.time()
        while time.time() - start_time < self.node.pesticide_spray_duration:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.node.e_stop_triggered:
                self.node.stop_spray()
                return 'fatal_error'
        
        # Step 3: Switch back to regular spray
        self.node.switch_spray_type(SprayType.REGULAR)
        
        # Step 4: Reset flags and continue
        self.node.pesticide_detected = False
        
        self.node.get_logger().info("[STATE: PESTICIDE_TREATMENT] Treatment complete, resuming navigation")
        return 'treatment_complete'


class TurningWithRowCheck(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['turn_complete_row_found', 'end_time_reached', 'fatal_error'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("[STATE: TURNING] Executing turn and checking for next row...")
        
        # Ensure spraying is stopped during turn
        self.node.stop_spray()
        self.node.halt_robot()
        
        # Simulate turning process
        self.node.get_logger().info("[STATE: TURNING] Performing U-turn...")
        time.sleep(3.0)  # Simulating turn time
        
        # Reset end-of-row flag
        self.node.reached_end_of_row = False
        
        # Wait for row detection with timeout
        self.node.get_logger().info(f"[STATE: TURNING] Waiting up to {self.node.end_time_limit} seconds for new row...")
        start_time = time.time()
        
        while time.time() - start_time < self.node.end_time_limit:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if self.node.e_stop_triggered:
                return 'fatal_error'
            
            if self.node.row_detected_after_turn:
                self.node.get_logger().info("[STATE: TURNING] New row detected! Continuing farming.")
                return 'turn_complete_row_found'
        
        # Timeout reached without finding new row
        self.node.get_logger().info(f"[STATE: TURNING] No new row found within {self.node.end_time_limit} seconds. Ending farming operation.")
        return 'end_time_reached'


class EndOfField(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=[])  # Terminal state
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("[STATE: END_OF_FIELD] Farming operation complete!")
        
        # Stop all systems
        self.node.stop_spray()
        self.node.halt_robot()
        
        # Stay in this state
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=1.0)
            if self.node.e_stop_triggered:
                break


class Fault(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=[])  # Terminal state
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().error("[STATE: FAULT] EMERGENCY STOP TRIGGERED!")
        
        # Stop everything immediately
        self.node.stop_spray()
        for _ in range(5):
            self.node.halt_robot()
            time.sleep(0.1)
        
        # Stay in fault state
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=1.0)


# ==========================================
# 3. Build the Enhanced State Machine
# ==========================================

def main():
    rclpy.init()
    farm_node = EnhancedFarmBotNode()

    farm_bot_sm = smach.StateMachine(outcomes=['field_complete', 'emergency_stop'])

    with farm_bot_sm:
        smach.StateMachine.add('IDLE', IdleWithTimer(farm_node),
                               transitions={'start_farming': 'NAVIGATING_ROW'})
        
        smach.StateMachine.add('NAVIGATING_ROW', NavigatingWithContinuousSpray(farm_node),
                               transitions={'pesticide_detected': 'PESTICIDE_TREATMENT',
                                            'reached_end_of_row': 'TURNING',
                                            'fatal_error': 'FAULT'})
        
        smach.StateMachine.add('PESTICIDE_TREATMENT', PesticideTreatment(farm_node),
                               transitions={'treatment_complete': 'NAVIGATING_ROW',
                                            'fatal_error': 'FAULT'})
        
        smach.StateMachine.add('TURNING', TurningWithRowCheck(farm_node),
                               transitions={'turn_complete_row_found': 'NAVIGATING_ROW',
                                            'end_time_reached': 'END_OF_FIELD',
                                            'fatal_error': 'FAULT'})
        
        smach.StateMachine.add('END_OF_FIELD', EndOfField(farm_node),
                               transitions={})
        
        smach.StateMachine.add('FAULT', Fault(farm_node),
                               transitions={})

    # Execute the state machine
    outcome = farm_bot_sm.execute()
    
    farm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
