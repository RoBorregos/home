#!/usr/bin/env python3

"""
This script manages the implementation of each Nav tasks
"""

### Import libraries
import rospy
import actionlib

### ROS messages
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
from frida_navigation_interfaces.msg import navServAction, navServFeedback, navServGoal, navServResult

# Topics and servers
NAV_SERVER = "/navServer"
MOVE_BASE_SERVER = "/move_base"
LOCATION_TOPIC = "/robot_pose"

# Active functions
NAV_SERVER_ACTIVE = True
MOVE_BASE_ACTIVE = True

class TasksNav:
    """Class to manage the navigation tasks"""
    STATE = {
        "TERMINAL_ERROR": -1,
        "EXECUTION_SUCCESS": 3,
        "EXECUTION_FAILED": 4
    }

    AREA_TASKS = ["go", "follow", "stop", "approach", "remember"]

    def __init__(self, enabled: bool = True) -> None:
        """Intialize the class"""
        self.enabled = enabled
        self.nav_server_active = NAV_SERVER_ACTIVE
        self.move_base_active = MOVE_BASE_ACTIVE

        if enabled and self.nav_server_active:
            self.nav_client = actionlib.SimpleActionClient(NAV_SERVER, navServAction)
            if not self.nav_client.wait_for_server(timeout=rospy.Duration(5.0)):
                self.nav_server_active = False
                rospy.logerr("Nav server not initialized")

        if enabled and self.move_base_active:
            self.move_base_client = actionlib.SimpleActionClient(MOVE_BASE_SERVER, MoveBaseAction)
            if not self.move_base_client.wait_for_server(timeout=rospy.Duration(5.0)):
                self.move_base_active = False
                rospy.logerr("Move Base server not initialized")
        self.past_location = None


    def execute_command(self, command: str, target: str, info: str) -> int:
        """Method to execute each command"""
        rospy.loginfo("Nav Command")

        if command == "go":
            return self.go_place(target)
        if command == "remember":
            return self.store_current_location()

        return TasksNav.STATE["EXECUTION_ERROR"]

    def go_place(self, target: str) -> int:
        """Action to move the robot to a specific location"""
        if target == "past location":
            if self.move_base_active and self.enabled:
                rospy.logerr("Nav move base not available")
                return TasksNav.STATE["EXECUTION_FAILED"]
            if self.past_location is None:
                rospy.logerr("No past location stored")
                return TasksNav.STATE["EXECUTION_FAILED"]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.past_location

            self.move_base_client.send_goal(goal)
            self.move_base_client.wait_for_result()
            rospy.loginfo("Arrived at past location")
            return TasksNav.STATE["EXECUTION_SUCCESS"]
        # Move to room location
        if self.nav_server_active and self.enabled:
            rospy.logerr("Nav server not available")
            return TasksNav.STATE["EXECUTION_FAILED"]
        try:
            goal = navServGoal()
            goal.target_location = target
            self.nav_client.send_goal(goal)
            self.nav_client.wait_for_result()
            return TasksNav.STATE["EXECUTION_SUCCESS"]
        except rospy.ROSException:
            rospy.logerr("Location not found")
            return TasksNav.STATE["EXECUTION_FAILED"]

    def store_current_location(self) -> int:
        """Method to retrieve the current location of the robot"""
        try:
            self.past_location = rospy.wait_for_message(LOCATION_TOPIC, Pose, timeout=3.0)
            rospy.loginfo("Current location stored")
            return TasksNav.STATE["EXECUTION_SUCCESS"]
        except rospy.ROSException:
            rospy.logerr("Unable to store current location")
            return TasksNav.STATE["EXECUTION_FAILED"]

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        self.nav_client.cancel_all_goals()
        rospy.loginfo("Command canceled Nav")

if __name__ == "__main__":
    try:
        TasksNav()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
