#!/usr/bin/env python3

"""
This script manages the implementation of each of the manipulation tasks
"""

### Import libraries
import rospy
import actionlib

### ROS messages
from std_msgs.msg import String
from frida_hri_interfaces.msg import Command, CommandList
from frida_manipulation_interfaces.msg import manipulationPickAndPlaceAction, manipulationPickAndPlaceGoal, manipulationPickAndPlaceResult, manipulationPickAndPlaceFeedback

PICK_AND_PLACE_SERVER = "/cartesianManipulationServer"

class TasksManipulation:

    AREA_TASKS = ["pick", "place", "grab", "give", "open", "close"]

    OBJECTS_DICT = {
        "zucaritas": 1
    }

    def __init__(self) -> None:
        self.pick_client = actionlib.SimpleActionClient(PICK_AND_PLACE_SERVER, manipulationPickAndPlaceAction)
        rospy.loginfo("Manipulation Task Manager initialized")

    def execute_command(self, command: str, goal: str, info: str) -> int:
        """Method to execute each command"""
        rospy.loginfo("Manipulation Command")

        goal = manipulationPickAndPlaceGoal()
        goal.object_id = TasksManipulation.OBJECTS_DICT[goal]
        #goal.wait = 0 if command == "feedback" else 1

        self.pick_client.send_goal(goal)
        self.pick_client.wait_for_result()
        result = self.pick_client.get_result()
        #rospy.loginfo(f"Result: {result.success}")
        return result.success
        
    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        self.pick_client.cancel_all_goals()
        rospy.loginfo("Command canceled Manipulation")

if __name__ == "__main__":
    try:
        TasksManipulation()
    except rospy.ROSInterruptException as e:
        rospy.logerr("Error: {}".format(e))
        pass