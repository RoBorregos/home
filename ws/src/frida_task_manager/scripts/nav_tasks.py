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

NAV_SERVER = "/move_base"

LOCATIONS = {
    "office" : {
        "table" : [
            -0.7937252521514893,
            1.0474066734313965,
            0.0,
            0.0,
            0.0,
            0.02197488636860073,
            0.9997585230289798
        ]
    }
}

class TasksNav:
    """Class to manage the navigation tasks"""
    STATE_ENUM = {
        "IDLE": 0,
        "RECEIVE_COMMANDS": 1,
        "EXECUTING_COMMANDS": 2,
        "STOPPING": 3,
        "ERROR": 4,
        "SHUTDOWN": 5
    }

    AREA_TASKS = ["go", "follow", "stop", "approach", "remember"]

    def __init__(self) -> None:
        self.nav_client = actionlib.SimpleActionClient(NAV_SERVER, MoveBaseAction)
        self.past_location = None

        init_result = self.nav_client.wait_for_server(timeout=rospy.Duration(5.0))
        if init_result:
            rospy.loginfo("Nav Task Manager initialized")
        else:
            rospy.logerr("Go action not initialized")

    def execute_command(self, command: str, target: str, info: str) -> int:
        """Method to execute each command"""
        rospy.loginfo("Nav Command")

        if command == "go":
            return self.go_place(target)

        return -1

    def go_place(self, target: str) -> int:
        """Action to move the robot to a specific location"""
        if target == "past location":
            return 1
        
        split_target = target.split(" ")
        try:
            location_pose = None
            for sublocation in split_target:
                if location_pose is None:
                    location_pose = LOCATIONS[sublocation]
                else:
                    location_pose = location_pose[sublocation]
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = location_pose[0]
            goal.target_pose.pose.position.y = location_pose[1]
            goal.target_pose.pose.position.z = location_pose[2]
            goal.target_pose.pose.orientation.x = location_pose[3]
            goal.target_pose.pose.orientation.y = location_pose[4]
            goal.target_pose.pose.orientation.z = location_pose[5]
            goal.target_pose.pose.orientation.w = location_pose[6]

            self.nav_client.send_goal(goal)
            self.nav_client.wait_for_result()

            return 1
            
        except KeyError:
            rospy.logerr("Location not found")
            return -1
        
    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        self.nav_client.cancel_all_goals()
        rospy.loginfo("Command canceled Nav")

if __name__ == "__main__":
    try:
        TasksNav()
    except rospy.ROSInterruptException as e:
        rospy.logerr("Error: {}".format(e))
        pass