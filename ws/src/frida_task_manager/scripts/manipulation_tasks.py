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
from frida_manipulation_interfaces.srv import Gripper

MANIPULATION_SERVER = "/manipulationServer"
ARM_JOINTS_SERVER = "/arm_joints_as"
PLACE_TARGET = -5
POUR_TARGET = -10

MANIPULATION_SERVER_ACTIVE = True
ARM_JOINTS_SERVER_ACTIVE = True

class TasksManipulation:
    """Manager for the manipulation area tasks"""
    STATE = {
        "TERMINAL_ERROR": -1,
        "EXECUTION_SUCCESS": 3,
        "EXECUTION_FAILED": 4
    }

    AREA_TASKS = ["pick", "place", "grab", "give", "open", "close"]

    OBJECTS_DICT = {
        "zucaritas": "cocacola",
        "cookies": "galletas",
        "cookie": "galletas",
        "snack": 5,
        "snacks": 5
    }

    def __init__(self, enabled: bool = True) -> None:
        self.enabled = enabled
        self.manipulation_server_active = MANIPULATION_SERVER_ACTIVE
        self.arm_joints_active = ARM_JOINTS_SERVER_ACTIVE


        if enabled and self.manipulation_server_active:
            self.manipulation_client = actionlib.SimpleActionClient(MANIPULATION_SERVER, manipulationPickAndPlaceAction)
            if not self.manipulation_client.wait_for_server(timeout=rospy.Duration(10.0)):
                self.manipulation_server_active = False
                rospy.logerr("Manipulation server not initialized")

        if enabled and self.arm_joints_active:
            self.arm_joints_client = actionlib.SimpleActionClient(ARM_JOINTS_SERVER, MoveJointAction)
            if not self.arm_joints_client.wait_for_server(timeout=rospy.Duration(10.0)):
                self.arm_joints_active = False
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
            return TasksManipulation.STATE["EXECUTION_FAILED"]
        return self.execute_pick_and_place( TasksManipulation.OBJECTS_DICT[target] )

    def execute_pick_and_place(self, target: int) -> int:
        """Method to call the pick and place action server"""
        if not self.enabled or not self.manipulation_server_active:
            rospy.logerr("Manipulation server not available")
            return TasksManipulation.STATE["EXECUTION_FAILED"]

        def manipulation_goal_feedback(feedback_msg):
            pass

        self.manipulation_client.send_goal(
                    manipulationPickAndPlaceGoal(object_name = target),
                    feedback_cb=manipulation_goal_feedback,
            )

        rospy.loginfo(f"Target: {target}")
        self.manipulation_client.wait_for_result()
        result = self.manipulation_client.get_result()
        return TasksManipulation.STATE["EXECUTION_SUCCESS"] if result.result else TasksManipulation.STATE["EXECUTION_FAILED"]
    
    def move_arm_joints(self, target_x: int, target_y: int, position: str = "") -> int:
        """Method to move the arm joints"""
        if not self.enabled or not self.arm_joints_active:
            rospy.logerr("Arm joints server not available")
            return TasksManipulation.STATE["EXECUTION_FAILED"]

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
        return TasksManipulation.STATE["EXECUTION_SUCCESS"] if result.success else TasksManipulation.STATE["EXECUTION_FAILED"]
    
    def set_gripper_state(self, open_gripper: bool) -> int:
        """Method to open or close the gripper"""
        if not self.enabled or not self.arm_joints_active:
            rospy.logerr("Arm joints server not available")
            return TasksManipulation.STATE["EXECUTION_FAILED"]
        rospy.wait_for_service("/gripper_service")
        try:
            gripper = rospy.ServiceProxy("/gripper_service", Gripper)
            response = gripper("open" if open_gripper else "close")
            return TasksManipulation.STATE["EXECUTION_SUCCESS"] if response.success else TasksManipulation.STATE["EXECUTION_FAILED"]
        except rospy.ServiceException:
            rospy.logerr("Service call failed")
        return TasksManipulation.STATE["EXECUTION_FAILED"] 

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        self.manipulation_client.cancel_all_goals()
        rospy.loginfo("Command canceled Manipulation")

if __name__ == "__main__":
    try:
        TasksManipulation()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
