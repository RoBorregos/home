#!/usr/bin/env python3

"""
Task manager for the Breakfast task of RoboCup @Home 2024
"""

### Import libraries
import rospy
import actionlib

### ROS messages
from std_msgs.msg import String, Bool
from frida_hri_interfaces.msg import Command, CommandList
from frida_hri_interfaces.msg import ConversateAction, ConversateFeedback, ConversateGoal, ConversateResult

### Python submodules
from hri_tasks import TasksHRI
from manipulation_tasks import TasksManipulation
from nav_tasks import TasksNav
from vision_tasks import TasksVision

NAV_ENABLED = True
MANIPULATION_ENABLED = True
CONVERSATION_ENABLED = True
VISION_ENABLED = True

FAKE_NAV = True
FAKE_MANIPULATION = True
FAKE_HRI = True
FAKE_VISION = True

AREAS = ["nav", "manipulation", "hri", "vision"]

AREA_ENABLED = {
    "nav": NAV_ENABLED,
    "manipulation": MANIPULATION_ENABLED,
    "hri": CONVERSATION_ENABLED,
    "vision": VISION_ENABLED
}

class TaskManagerServer:
    """Class to manage different tasks divided by categories"""
    
    TASK_STATES = {
        "PRE_TABLE_POSITION": 0,
        "APPROACH_TABLE": 1,
        "PICK_OBJECT": 2,
        "PRE_SHELVE_POSITION": 4,
        "ANALYZE_SHELVE": 5,
        "APPROACH_SHELVE": 6,
        "SELECT_OBJECT_SHELVE": 7,
        "PLACE": 8,
        "SHUTDOWN": 9
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
        "nav" : ["go", "follow", "stop", "approach", "remember", "go_pose", "stop_follow"],
        "manipulation" : ["pick", "place", "grasp", "give", "open", "close", "pour", "observe"],
        "hri" : ["ask", "interact", "feedback"],
        "vision" : ["find", "identify", "count", "get_bag"]
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
        if CONVERSATION_ENABLED:
            self.subtask_manager["hri"] = TasksHRI(fake=FAKE_HRI)
            self.subtask_manager["hri"].speak("Hi, my name is Frida. I'm here to help you with your domestic tasks")   
        
        self.current_state = TaskManagerServer.TASK_STATES["PRE_TABLE_POSITION"]
        self.current_command = None

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
            if self.current_state == TaskManagerServer.TASK_STATES["PRE_TABLE_POSITION"]:
                pass
            elif self.current_state == TaskManagerServer.TASK_STATES["APPROACH_TABLE"]:
                pass
            elif self.current_state == TaskManagerServer.TASK_STATES["PICK_OBJECT"]:
                pass
            elif self.current_state == TaskManagerServer.TASK_STATES["PRE_SHELVE_POSITION"]:
                pass
            elif self.current_state == TaskManagerServer.TASK_STATES["ANALYZE_SHELVE"]:
                pass
            elif self.current_state == TaskManagerServer.TASK_STATES["APPROACH_SHELVE"]:
                pass
            elif self.current_state == TaskManagerServer.TASK_STATES["SELECT_OBJECT_SHELVE"]:
                pass
            elif self.current_state == TaskManagerServer.TASK_STATES["PLACE"]:
                pass
            
            if self.current_state == TaskManagerServer.TASK_STATES["SHUTDOWN"]:
                rospy.loginfo("[INFO] Returning to initial position...")
                self.subtask_manager["hri"].speak("I have finished my tasks, I'm going to rest now")
                break
            self._rate.sleep()
        rospy.loginfo("[SUCCESS] Task Finished")

if __name__ == "__main__":
    try:
        TaskManagerServer()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
