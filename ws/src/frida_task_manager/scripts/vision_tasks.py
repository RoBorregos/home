#!/usr/bin/env python3

"""
This script manages the implementation of each Nav tasks
"""

### Import libraries
import rospy
import actionlib

### ROS messages
from std_msgs.msg import String
from std_srvs.srv import SetBool
from frida_vision_interfaces.srv import NewHost, NewHostResponse, FindSeat, PersonCount 

STORE_FACE_SERVICE = "/new_name"
CHECK_PERSON = "/check_person"
FIND_TOPIC = "/find_seat"
PERSON_COUNT_START_TOPIC = "/start_counting"
PERSON_COUNT_END_TOPIC = "/end_counting"

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

    def check_person(self) -> bool:
        """Method to check if a person is detected calling PersonDetection.py"""
        try:
            rospy.wait_for_service(CHECK_PERSON, timeout=5.0)
            check_person = rospy.ServiceProxy(CHECK_PERSON, SetBool)
            response = check_person(True)
            return response.success
        except rospy.ServiceException:
            rospy.logerr("Service call check_person failed")
            return False

    def find_seat(self) -> int:
        """Method to find the angle the robot should turn to point the free seat"""
        try:
            rospy.wait_for_service(FIND_TOPIC, timeout=5.0)
            find_seat = rospy.ServiceProxy(FIND_TOPIC, FindSeat)
            response = find_seat(True)
            if int(response.angle) == -100:
                return 300
            return int(response.angle)
        except rospy.ServiceException:
            rospy.logerr("Service call find_seat failed")
            return 300
        
    def count_persons(self, requirements: str) -> str:
        """Method to count the number of persons in the room"""
        try:
            rospy.wait_for_service(PERSON_COUNT_START_TOPIC, timeout=5.0)
            start_count = rospy.ServiceProxy(PERSON_COUNT_START_TOPIC, SetBool)
            response = start_count(True)
            if response.success:
                rospy.wait_for_service(PERSON_COUNT_END_TOPIC, timeout=5.0)
                goal = PersonCount()
                end_count = rospy.ServiceProxy(PERSON_COUNT_END_TOPIC, PersonCount)
                goal.data = requirements
                response = end_count(goal)
                return response.count
        except rospy.ServiceException:
            rospy.logerr("Service call count_persons failed")
            return -1
        

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        rospy.loginfo("Command canceled Nav")

if __name__ == "__main__":
    try:
        TasksVision()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
