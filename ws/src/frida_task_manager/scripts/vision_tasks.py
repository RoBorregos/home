#!/usr/bin/env python3

"""
This script manages the implementation of each Nav tasks
"""

### Import libraries
import rospy
import actionlib

### ROS messages
from std_msgs.msg import String
from frida_vision_interfaces.srv import NewHost, NewHostResponse

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
        self.save_name_call = rospy.ServiceProxy(STORE_FACE_SERVICE, NewHost)

        self.save_name_call.wait_for_service(timeout=rospy.Duration(10.0))
        
        rospy.loginfo("Vision Task Manager initialized")

    def execute_command(self, command: str, target: str, info: str) -> int:
        """Method to execute each command"""
        rospy.loginfo("Nav Command")

        return TasksVision.STATE["EXECUTION_ERROR"]
    
    def save_face_name(self, name: str) -> int:
        """Method to save the face name"""
        rospy.loginfo("Save face name")
        try:
            response = self.save_name_call( name )
            if response.success:
                return TasksVision.STATE["EXECUTION_SUCCESS"]
        except rospy.ServiceException:
            rospy.logerr("Service call name failed")

        return TasksVision.STATE["EXECUTION_ERROR"]

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        rospy.loginfo("Command canceled Nav")

if __name__ == "__main__":
    try:
        TasksVision()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
