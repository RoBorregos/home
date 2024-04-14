#!/usr/bin/env python3

"""
This script manages the implementation of each Nav tasks
"""

### Import libraries
import rospy
import actionlib

### ROS messages
from std_msgs.msg import String
from std_srvs.srv import GetBool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
from frida_navigation_interfaces.msg import navServAction, navServFeedback, navServGoal, navServResult

STORE_FACE_SERVICE = "/new_name"

class TasksVision:
    """Class to manage the navigation tasks"""
    STATE = {
        "TERMINAL_ERROR": -1,
        "EXECUTION_ERROR": 0,
        "EXECUTION_SUCCESS": 1
    }

    AREA_TASKS = ["wait", "save"]

    def __init__(self) -> None:
        """Initialize the ROS node""" 
        self.save_name_service = rospy.ServiceProxy(STORE_FACE_SERVICE, GetBool)

        if not self.save_name_service.wait_for_service(timeout=rospy.Duration(10.0)):
            rospy.logerr("Save name service not initialized")

    def execute_command(self, command: str, target: str, info: str) -> int:
        """Method to execute each command"""
        rospy.loginfo("Nav Command")

        if command == "save":



        return TasksVision.STATE["EXECUTION_ERROR"]
    
    def save_face_name(self, name: str) -> int:
        """Method to save the face name"""
        rospy.loginfo("Save face name")
        try:
            response = self.save_name_service(

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        rospy.loginfo("Command canceled Nav")

if __name__ == "__main__":
    try:
        TasksVision()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
