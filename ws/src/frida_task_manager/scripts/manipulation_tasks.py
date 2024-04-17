#!/usr/bin/env python3

"""
This script manages the implementation of each of the manipulation tasks
"""

### Import libraries
import rospy
import actionlib

### ROS messages
from std_msgs.msg import String, Int32
from frida_hri_interfaces.msg import Command, CommandList
from frida_manipulation_interfaces.msg import manipulationPickAndPlaceAction, manipulationPickAndPlaceGoal, manipulationPickAndPlaceResult, manipulationPickAndPlaceFeedback
from frida_manipulation_interfaces.msg import MoveJointAction, MoveJointGoal, MoveJointResult, MoveJointFeedback
from frida_manipulation_interfaces.srv import Gripper

MANIPULATION_SERVER = "/manipulationServer"
ARM_SERVER = "/arm_as"
PLACE_TARGET = -5
POUR_TARGET = -10

class TasksManipulation:
    """Manager for the manipulation area tasks"""
    STATE = {
        "TERMINAL_ERROR": -1,
        "EXECUTION_ERROR": 0,
        "EXECUTION_SUCCESS": 1
    }

    AREA_TASKS = ["pick", "place", "grab", "give", "open", "close"]

    OBJECTS_DICT = {
        "zucaritas": "cocacola",
        "cookies": "galletas",
        "cookie": "galletas",
        "snack": 5,
        "snacks": 5
    }

    def __init__(self, fake = False) -> None:
        
        self.FAKE_TASKS = fake
        if not self.FAKE_TASKS:
            rospy.loginfo("[INFO] Waiting for manipulation server")
            self.manipulation_client = actionlib.SimpleActionClient(MANIPULATION_SERVER, manipulationPickAndPlaceAction)
            self.move_arm_client = actionlib.SimpleActionClient(ARM_SERVER, MoveJointAction)
            self.gripper_service = rospy.ServiceProxy('/gripper_service', Gripper)
            
            rospy.loginfo("[INFO] Connecting to manipulation_server")
            if not self.manipulation_client.wait_for_server(timeout=rospy.Duration(10.0)):
                rospy.logerr("[SUCCESS] Manipulation server not initialized")
            rospy.loginfo("[INFO] Connecting to arm_server")
            if not self.move_arm_client.wait_for_server(timeout=rospy.Duration(10.0)):
                rospy.logerr("[SUCCESS] Arm server not initialized")
            rospy.loginfo("[INFO] Connecting to gripper_service")
            if not self.gripper_service.wait_for_service(timeout=rospy.Duration(10.0)):
                rospy.logerr("[SUCCESS] Gripper service not initialized")
            

        rospy.loginfo("[SUCCESS] Manipulation Task Manager initialized")

    def execute_command(self, command: str, target: str, info: str) -> int:
        """Method to execute each command"""
        rospy.loginfo("[INFO] Manipulation Command")
        if command == "pick":
            return self.execute_pick_and_place( target )
        if command in ("place", "pour"):
            return self.execute_pick_and_place( command )
        if command in ("give"):
            return self.execute_give()
        if command in ("observe"):
            return self.execute_observe()
        return -1

    def execute_pick(self, target: str) -> int:
        """Method to execute the pick action"""
        if target not in TasksManipulation.OBJECTS_DICT:
            rospy.logerr("Object not found")
            return TasksManipulation.STATE["EXECUTION_ERROR"]
        if not self.FAKE_TASKS:
            return self.execute_pick_and_place( TasksManipulation.OBJECTS_DICT[target] )
        else:
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]

    def execute_pick_and_place(self, target: int) -> int:
        """Method to call the pick and place action server"""

        def manipulation_goal_feedback(feedback_msg):
            pass
        
        
        rospy.loginfo(f"[INFO] Manipulation Target: {target}")
        
        if not self.FAKE_TASKS:
            self.manipulation_client.send_goal(
                        manipulationPickAndPlaceGoal(object_name = target),
                        feedback_cb=manipulation_goal_feedback,
                )

            self.manipulation_client.wait_for_result()
            result = self.manipulation_client.get_result()
            return TasksManipulation.STATE["EXECUTION_SUCCESS"] if result.result else TasksManipulation.STATE["EXECUTION_ERROR"]
        else:
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
    
    def execute_give(self) -> int:
        rospy.loginfo(f"[INFO] Giving")
        if not self.FAKE_TASKS:
            # execute give
            rospy.loginfo("[INFO] Giving...")
            self.move_arm_client.send_goal(
                MoveJointGoal(predefined_position="give"),
            )
            self.move_arm_client.wait_for_result()
            rospy.sleep(1)
            rospy.loginfo("[INFO] Opening gripper")
            self.gripper_service(True)
            rospy.sleep(3)
            rospy.loginfo("[INFO] Closing gripper")
            self.gripper_service(False)
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
        else:
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
    
    def execute_observe(self) -> int:
        rospy.loginfo(f"[INFO] Observing")
        if not self.FAKE_TASKS:
            # execute observe
            rospy.loginfo("[INFO] Moving to observe position")
            self.move_arm_client.send_goal(
                MoveJointGoal(predefined_position="observe"),
            )
            self.move_arm_client.wait_for_result()
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
        else:
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        if not self.FAKE_TASKS:
            self.manipulation_client.cancel_all_goals()
            rospy.loginfo("Command canceled Manipulation")
        else:
            rospy.loginfo("Command canceled Manipulation")

if __name__ == "__main__":
    try:
        TasksManipulation()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
