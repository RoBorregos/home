#!/usr/bin/env python3

"""
Task manager for the Storing task of RoboCup @Home 2024
"""

### Import libraries
import rospy
import actionlib
import copy
import math
### ROS messages
from std_msgs.msg import String, Bool
from frida_hri_interfaces.msg import Command, CommandList
from frida_hri_interfaces.msg import ConversateAction, ConversateFeedback, ConversateGoal, ConversateResult
from frida_navigation_interfaces.msg import navServAction, navServFeedback, navServGoal, navServResult


### Python submodules
from hri_tasks import TasksHRI
from manipulation_tasks import TasksManipulation
from nav_tasks import TasksNav
from vision_tasks import TasksVision

NAV_ENABLED = True
MANIPULATION_ENABLED = True
CONVERSATION_ENABLED = True
VISION_ENABLED = True

FAKE_NAV = False
FAKE_MANIPULATION = False
FAKE_HRI = False
FAKE_VISION = False

SHELF_SIZE = 0.35
TARGET_SHELVE_APPROACH = 0.4

# For Storing Groceries
# Constants
ARM_MAX_HEIGHT = 1.3
ARM_MIN_HEIGHT = 1.0
ARM_MAX_ANGLE = 45
ARM_MIN_ANGLE = -45
DISTANCE_TO_SHELF = TARGET_SHELVE_APPROACH

AREAS = ["nav", "manipulation", "hri", "vision"]
VISION_AVAILABLE_MODES = ["fast_execution", "robust"]

AREA_ENABLED = {
    "nav": NAV_ENABLED,
    "manipulation": MANIPULATION_ENABLED,
    "hri": CONVERSATION_ENABLED,
    "vision": VISION_ENABLED
}

class TaskManagerServer:
    """Class to manage different tasks divided by categories"""
    
    TASK_STATES = {
        "WAITING_DOOR" : 0,
        "NAV_TO_INSPECTION_POINT" : 1,
        "WAIT_FOR_CONVERSATION_SIGNAL" : 2,
        "RETURN_ENTRANCE" : 3,
    }
    
    STATE_ENUM = {
        "IDLE": 0,
        "RECEIVE_COMMANDS": 1,
        "EXECUTING_COMMANDS": 2,
        "STOPPING": 3,
        "ERROR": 4,
        "SHUTDOWN": 5
    }

    COMMANDS_CATEGORY = {
        "nav" : ["go", "follow", "stop", "approach", "remember", "go_pose", "stop_follow", "deproach", "door_signal"],
        "manipulation" : ["pick", "place", "grasp", "give", "open", "close", "pour", "observe", "place_shelf", "move_arm"],
        "hri" : ["ask", "interact", "feedback", "analyze_objects"],
        "vision" : ["find", "identify", "count", "get_bag", "get_shelves"]
    }

    def __init__(self) -> None:
        self._node = rospy.init_node("task_manager_server")
        self._rate = rospy.Rate(200)
        # Creates an empty dictionary to store the subtask manager of each area
        self.subtask_manager = dict.fromkeys(AREAS, None)
        
        if MANIPULATION_ENABLED:
            self.subtask_manager["manipulation"] = TasksManipulation(fake=FAKE_MANIPULATION)
        if NAV_ENABLED:
            self.subtask_manager["nav"] = TasksNav(fake=FAKE_NAV)
        if VISION_ENABLED:
            self.subtask_manager["vision"] = TasksVision(fake=FAKE_VISION)
            self.vision_mode = "robust"
            if self.vision_mode not in VISION_AVAILABLE_MODES:
                rospy.logerr("[ERROR] Invalid vision mode")
                return
        if CONVERSATION_ENABLED:
            self.subtask_manager["hri"] = TasksHRI(fake=FAKE_HRI)
            self.subtask_manager["hri"].speak("Hi, my name is Frida. I'm here to help you with your domestic tasks")   
        
        # self.current_state = TaskManagerServer.TASK_STATES["PRE_TABLE_POSITION"]
        self.current_state = TaskManagerServer.TASK_STATES["WAITING_DOOR"]
        self.current_command = None
        self.perceived_information = ""

        self.run()

    def execute_command(self, command: Command) -> int:
        """Method for executing a single command inside its area submodule"""

        rospy.loginfo(f"[INFO] Executing command: {command.action} -> {command.complement}")

        task_result = 0
        for area in AREAS:
            if command.action in TaskManagerServer.COMMANDS_CATEGORY[area] and AREA_ENABLED[area]:
                task_result = self.subtask_manager[area].execute_command(
                    command.action, command.complement, self.perceived_information
                )

        if task_result == -1 or task_result == 0:
            rospy.logerr("[ERROR] Error in task execution")
            return TaskManagerServer.STATE_ENUM["ERROR"]

        self.perceived_information += f"{command.action} {command.complement} {task_result} "
        return TaskManagerServer.STATE_ENUM["EXECUTING_COMMANDS"]

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        for area in AREAS:
            if self.current_command in TaskManagerServer.COMMANDS_CATEGORY[area]:
                self.subtask_manager[area].cancel_command()

    def run(self) -> None:
        """Main loop for the task manager"""


        while not rospy.is_shutdown():
            if self.current_state == TaskManagerServer.TASK_STATES["WAITING_DOOR"]:
                result = self.execute_command(Command(action="door_signal"))
                if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                    rospy.logerr("[ERROR] Error in task execution")
                    return
                result = self.execute_command(Command(action="remember"))
                self.current_state = TaskManagerServer.TASK_STATES["NAV_TO_INSPECTION_POINT"]
            elif self.current_state == TaskManagerServer.TASK_STATES["NAV_TO_INSPECTION_POINT"]:
                result = self.execute_command(Command(action="go", complement="inspection_point"))
                if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                    rospy.logerr("[ERROR] Error in task execution")
                    return
                self.current_state = TaskManagerServer.TASK_STATES["WAIT_FOR_CONVERSATION_SIGNAL"]
                self.subtask_manager["hri"].speak("Hi. Please say my name for me to return to the entrance")   
            elif self.current_state == TaskManagerServer.TASK_STATES["WAIT_FOR_CONVERSATION_SIGNAL"]:
                if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                    rospy.logerr("[ERROR] Error in task execution")
                    return
                if self.subtask_manager["hri"].keyword_detected:
                    self.current_state = TaskManagerServer.TASK_STATES["RETURN_ENTRANCE"]
            elif self.current_state == TaskManagerServer.TASK_STATES["RETURN_ENTRANCE"]:
                result = self.execute_command(Command(action="go", complement="past location"))
                if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                    rospy.logerr("[ERROR] Error in task execution")
                    return
                break
            self._rate.sleep()
            
        rospy.loginfo("[SUCCESS] Task Finished")

if __name__ == "__main__":
    try:
        TaskManagerServer()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
