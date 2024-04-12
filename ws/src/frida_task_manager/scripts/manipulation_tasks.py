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

PICK_AND_PLACE_SERVER = "/manipulationServer"
PLACE_TARGET = -5

class TasksManipulation:

    STATE = {
        "TERMINAL_ERROR": -1,
        "EXECUTION_ERROR": 0,
        "EXECUTION_SUCCESS": 1
    }

    AREA_TASKS = ["pick", "place", "grab", "give", "open", "close"]

    OBJECTS_DICT = {
        "zucaritas": 1,
        "cookies": 11,
        "cookie": 11,
        "snack": 5,
        "snacks": 5
    }

    def __init__(self) -> None:
        self.pick_client = actionlib.SimpleActionClient(PICK_AND_PLACE_SERVER, manipulationPickAndPlaceAction)
        self.pick_client.wait_for_server()
        rospy.loginfo("Manipulation Task Manager initialized")

    def execute_command(self, command: str, target: str, info: str) -> int:
        """Method to execute each command"""
        rospy.loginfo("Manipulation Command")
        if command == "pick":
            return self.execute_pick( target )
        if command == "place":
            return self.execute_pick_and_place(PLACE_TARGET)

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

        rospy.loginfo("Sending Manipulation Goal")

        self.pick_client.send_goal(
                    manipulationPickAndPlaceGoal(object_id = target),
                    feedback_cb=manipulation_goal_feedback,
            )

        rospy.loginfo(f"Object ID: {target}")
        self.pick_client.wait_for_result()
        result = self.pick_client.get_result()
        return TasksManipulation.STATE["EXECUTION_SUCCESS"] if result.result else TasksManipulation.STATE["EXECUTION_ERROR"]

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        self.pick_client.cancel_all_goals()
        rospy.loginfo("Command canceled Manipulation")

if __name__ == "__main__":
    try:
        TasksManipulation()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
