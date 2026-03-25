import rclpy
from rclpy.node import Node
import smach

# ==========================================
# 1. Define the Robot's States
# ==========================================

class Idle(smach.State):
    def __init__(self):
        # 'outcomes' are the exact strings this state is allowed to return
        smach.State.__init__(self, outcomes=['start_cmd_received'])

    def execute(self, userdata):
        print("[STATE: IDLE] Robot is standing by. Waiting for command...")
        # TODO: Add logic to wait for a physical button press or a ROS 2 start topic
        
        # For testing, we just automatically transition to navigating
        return 'start_cmd_received'


class NavigatingRow(smach.State):
    def __init__(self):
        # This is the most complex state. It can branch off into 3 different actions.
        smach.State.__init__(self, outcomes=['pest_detected', 'reached_end_of_row', 'fatal_error'])

    def execute(self, userdata):
        print("[STATE: NAVIGATING] Driving forward, staying centered using LIDAR...")
        # TODO: Subscribe to Branden's Navigation node here.
        
        # Simulated logic for testing the flow in your terminal:
        action = input("Simulate event (type 'pest' or 'end'): ")
        if action == 'pest':
            return 'pest_detected'
        elif action == 'end':
            return 'reached_end_of_row'
        else:
            return 'fatal_error'


class Spraying(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['spray_complete'])

    def execute(self, userdata):
        print("[STATE: SPRAYING] Target locked. Pausing motors and firing payload!")
        # TODO: Publish a 'True' boolean to the /spray_cmd topic for Aylin's Pico node.
        # TODO: Wait for Emma's CV node to confirm the bounding box has passed.
        
        return 'spray_complete'


# ==========================================
# 2. Build the State Machine
# ==========================================

def main():
    # Initialize the ROS 2 Python environment
    rclpy.init()

    # Create a top-level state machine
    # 'e_stop_triggered' is the absolute final exit code for the whole program
    farm_bot_sm = smach.StateMachine(outcomes=['e_stop_triggered'])

    # Open the state machine container to add our states
    with farm_bot_sm:
        
        # Add the IDLE state. If it returns 'start_cmd_received', go to NAVIGATING_ROW.
        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'start_cmd_received': 'NAVIGATING_ROW'})
        
        # Add the NAVIGATING_ROW state. Define where its 3 outcomes lead.
        smach.StateMachine.add('NAVIGATING_ROW', NavigatingRow(), 
                               transitions={'pest_detected': 'SPRAYING',
                                            'reached_end_of_row': 'IDLE', # Placeholder: Loop back to idle for now
                                            'fatal_error': 'e_stop_triggered'})
        
        # Add the SPRAYING state. When done, it must go back to navigating.
        smach.StateMachine.add('SPRAYING', Spraying(), 
                               transitions={'spray_complete': 'NAVIGATING_ROW'})

    # Start the state machine!
    print("--- Farm Robot Systems Initialized ---")
    outcome = farm_bot_sm.execute()
    
    print(f"--- System Shut Down with outcome: {outcome} ---")

if __name__ == '__main__':
    main()