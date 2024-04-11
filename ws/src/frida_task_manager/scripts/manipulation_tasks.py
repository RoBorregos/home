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

    AREA_TASKS = ["pick", "place", "grab", "give", "open", "close"]

    OBJECTS_DICT = {
        "zucaritas": 1,
        "cookies": 9,
        "cookie": 9
    }

    def __init__(self) -> None:
        self.pick_client = actionlib.SimpleActionClient(PICK_AND_PLACE_SERVER, manipulationPickAndPlaceAction)
        self.pick_client.wait_for_server()
        rospy.loginfo("Manipulation Task Manager initialized")

    def execute_command(self, command: str, target: str, info: str) -> int:
        """Method to execute each command"""
        rospy.loginfo("Manipulation Command")
        if command == "pick":
            return self.execute_pick_and_place( TasksManipulation.OBJECTS_DICT[target] )
        if command == "place":
            return self.execute_pick_and_place(PLACE_TARGET)

        return -1
    
    def execute_pick_and_place(self, target: int) -> int:
        """Method to call the pick and place action server"""
        class ManipulationGoalScope:
            object_ = target
            result = False
            result_received = False
        
        def manipulation_goal_feedback(feedback_msg):
            pass
        
        def get_result_callback(state, result):
            ManipulationGoalScope.result = result.result
            ManipulationGoalScope.result_received = True

        rospy.loginfo("Sending Manipulation Goal")

        self.pick_client.send_goal(
                    manipulationPickAndPlaceGoal(object_id = ManipulationGoalScope.object_),
                    feedback_cb=manipulation_goal_feedback,
                    done_cb=get_result_callback)
        
        rospy.loginfo(f"Object ID: {target}")
        
        while not ManipulationGoalScope.result_received and not rospy.is_shutdown():
            pass
        
        return 1 if ManipulationGoalScope.result else -1
        
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