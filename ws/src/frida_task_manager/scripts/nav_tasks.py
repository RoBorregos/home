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
from geometry_msgs.msg import Pose, PoseStamped
from frida_navigation_interfaces.msg import navServAction, navServFeedback, navServGoal, navServResult

NAV_SERVER = "/navServer"
MOVE_BASE_SERVER = "/move_base"
LOCATION_TOPIC = "/robot_pose"

FAKE_TASKS = True

class TasksNav:
    """Class to manage the navigation tasks"""
    STATE = {
        "TERMINAL_ERROR": -1,
        "EXECUTION_ERROR": 0,
        "EXECUTION_SUCCESS": 1
    }

    AREA_TASKS = ["go", "follow", "stop", "approach", "remember"]

    def __init__(self) -> None:
        if not FAKE_TASKS:
            self.nav_client = actionlib.SimpleActionClient(NAV_SERVER, navServAction)
            self.move_base_client = actionlib.SimpleActionClient(MOVE_BASE_SERVER, MoveBaseAction)
            rospy.loginfo("[INFO] Waiting for nav server")
            if not self.nav_client.wait_for_server(timeout=rospy.Duration(5.0)):
                rospy.logerr("Nav server not initialized")
            rospy.loginfo("[INFO] Waiting for move base server")
            if not self.move_base_client.wait_for_server(timeout=rospy.Duration(5.0)):
                rospy.logerr("[INFO] Move Base server not initialized")
        else:
            rospy.loginfo("[INFO] Fake Nav Task Manager initialized")
        
        rospy.loginfo("[SUCCESS] Nav Task Manager initialized")
        self.past_location = None

    def execute_command(self, command: str, target: str, info: str) -> int:
        """Method to execute each command"""
        rospy.loginfo("[INFO] Nav Command")

        if command == "go":
            return self.go_place(target)
        if command == "remember":
            return self.store_current_location()

        return TasksNav.STATE["EXECUTION_ERROR"]

    def go_place(self, target: str) -> int:
        """Action to move the robot to a specific location"""
        if not FAKE_TASKS:
            if target == "past location":
                if self.past_location is None:
                    rospy.logerr("No past location stored")
                    return TasksNav.STATE["EXECUTION_ERROR"]
                
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = self.past_location

                self.move_base_client.send_goal(goal)
                self.move_base_client.wait_for_result()
                rospy.loginfo("[SUCCESS] Arrived at past location")
                return TasksNav.STATE["EXECUTION_SUCCESS"]
            # Move to room location
            try:
                goal = navServGoal()
                goal.target_location = target

                self.nav_client.send_goal(goal)
                self.nav_client.wait_for_result()
                return TasksNav.STATE["EXECUTION_SUCCESS"]
            except rospy.ROSException:
                rospy.logerr("Location not found")
                return TasksNav.STATE["EXECUTION_ERROR"]
        else:
            rospy.loginfo("[INFO] Going to past location" if target == "past location" else f"Going to {target}")
            rospy.loginfo("[SUCCESS] Arrived at past location" if target == "past location" else f"Arrived at {target}")
            return TasksNav.STATE["EXECUTION_SUCCESS"]

    def go_pose(self, target: PoseStamped) -> int:
        """Action to move the robot to a specific location"""
        if not FAKE_TASKS:
            goal = MoveBaseGoal()
            if goal.target_pose.header.frame_id != "map":
                # Transform pose to the map frame
                rospy.loginfo("[INFO] Transforming pose")
                pass
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = target.pose

            self.move_base_client.send_goal(goal)
            self.move_base_client.wait_for_result()
            return TasksNav.STATE["EXECUTION_SUCCESS"]
        else:
            rospy.loginfo("[INFO] Going to pose")
            return TasksNav.STATE["EXECUTION_SUCCESS"]
    
    def store_current_location(self) -> int:
        """Method to retrieve the current location of the robot"""
        if not FAKE_TASKS:
            try:
                self.past_location = rospy.wait_for_message(LOCATION_TOPIC, Pose, timeout=3.0)
                rospy.loginfo("[SUCCESS] Current location stored")
                return TasksNav.STATE["EXECUTION_SUCCESS"]
            except rospy.ROSException:
                rospy.logerr("[SUCCESS] Unable to store current location")
                return TasksNav.STATE["EXECUTION_ERROR"]
        else:
            rospy.loginfo("[SUCCESS]] Current location stored")
            return TasksNav.STATE["EXECUTION_SUCCESS"]

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        if not FAKE_TASKS:
            self.move_base_client.cancel_all_goals()
        rospy.loginfo("[INFO] Command canceled Nav")

if __name__ == "__main__":
    try:
        TasksNav()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'[ERROR] Error: {e}')
