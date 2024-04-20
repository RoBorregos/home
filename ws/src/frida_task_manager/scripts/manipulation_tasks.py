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
from frida_manipulation_interfaces.msg import MoveJointAction, MoveJointFeedback, MoveJointGoal, MoveJointResult

MANIPULATION_SERVER = "/manipulationServer"
ARM_JOINTS_SERVER = "/arm_joints_as"
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

    def __init__(self) -> None:
        self.manipulation_client = actionlib.SimpleActionClient(MANIPULATION_SERVER, manipulationPickAndPlaceAction)
        self.arm_joints_client = actionlib.SimpleActionClient(ARM_JOINTS_SERVER, MoveJointAction)

        if not self.manipulation_client.wait_for_server(timeout=rospy.Duration(10.0)):
            rospy.logerr("Manipulation server not initialized")
        if not self.arm_joints_client.wait_for_server(timeout=rospy.Duration(10.0)):
            rospy.logerr("Arm joints server not initialized")

        rospy.loginfo("Manipulation Task Manager initialized")

    def execute_command(self, command: str, target: str, info: str) -> int:
        """Method to execute each command"""
        rospy.loginfo("Manipulation Command")
        if command == "pick":
            return self.execute_pick_and_place( target )
        if command in ("place", "pour"):
            return self.execute_pick_and_place( command )

        return -1

    def execute_pick(self, target: str) -> int:
        """Method to execute the pick action"""
        if target not in TasksManipulation.OBJECTS_DICT:
            rospy.logerr("Object not found")
            return TasksManipulation.STATE["EXECUTION_ERROR"]
        return self.execute_pick_and_place( TasksManipulation.OBJECTS_DICT[target] )

    def execute_pick_and_place(self, target: int) -> int:
        """Method to call the pick and place action server"""

        def manipulation_goal_feedback(feedback_msg):
            pass

        self.manipulation_client.send_goal(
                    manipulationPickAndPlaceGoal(object_name = target),
                    feedback_cb=manipulation_goal_feedback,
            )

        rospy.loginfo(f"Target: {target}")
        self.manipulation_client.wait_for_result()
        result = self.manipulation_client.get_result()
        return TasksManipulation.STATE["EXECUTION_SUCCESS"] if result.result else TasksManipulation.STATE["EXECUTION_ERROR"]
    
    def move_arm_joints(self, target_x: int, target_y: int, position: str = "") -> int:
        """Method to move the arm joints"""
        if position != "":
            self.arm_joints_client.send_goal(
                MoveJointGoal(predefined_position = position)
            )
        else:
            self.arm_joints_client.send_goal(
                MoveJointGoal(target_delta_x = target_x, target_delta_y = target_y)
            )
        self.arm_joints_client.wait_for_result(rospy.Duration(5))
        result = self.arm_joints_client.get_result()
        return TasksManipulation.STATE["EXECUTION_SUCCESS"] if result.success else TasksManipulation.STATE["EXECUTION_ERROR"]

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        self.manipulation_client.cancel_all_goals()
        rospy.loginfo("Command canceled Manipulation")

if __name__ == "__main__":
    try:
        TasksManipulation()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
