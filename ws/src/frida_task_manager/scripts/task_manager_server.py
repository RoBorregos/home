#!/usr/bin/env python3

"""
This script is the base/template for the development of multiple task managers for every task in RoboCup@Home
"""

### Import libraries
import rospy
import actionlib
from threading import current_thread
import uuid

### ROS messages
from std_msgs.msg import String
from frida_hri_interfaces.msg import Command, CommandList
from frida_hri_interfaces.msg import ConversateAction, ConversateFeedback, ConversateGoal, ConversateResult

### Python submodules
from hri_tasks import TasksHRI
from manipulation_tasks import TasksManipulation
from nav_tasks import TasksNav
from vision_tasks import TasksVision

COMMANDS_TOPIC = "/task_manager/commands"
SPEAK_TOPIC = "/speech/speak"
CONVERSATION_SERVER = "/conversation_as"

NAV_ENABLED = True
MANIPULATION_ENABLED = True
CONVERSATION_ENABLED = True
VISION_ENABLED = False

AREAS = ["nav", "manipulation", "hri", "vision"]

AREA_ENABLED = {
    "nav": NAV_ENABLED,
    "manipulation": MANIPULATION_ENABLED,
    "hri": CONVERSATION_ENABLED,
    "vision": VISION_ENABLED
}

STATES = {
    "IDLE": 0,
    "COMMANDS_IN_QUEUE": 1,
    "EXECUTING_COMMANDS": 2,
    "EXECUTION_SUCCESS": 3,
    "EXECUTION_FAILED": 4,
    "STOPPING": 5,
    "ERROR": 6,
    "SHUTDOWN": 7
}

class TaskManagerServer:
    """Class to manage different tasks divided by categories"""

    COMMANDS_CATEGORY = {
        "nav" : ["go", "follow", "stop", "approach", "remember"],
        "manipulation" : ["pick", "place", "grasp", "give", "open", "close"],
        "hri" : ["ask", "interact", "feedback"],
        "vision" : ["find", "identify", "count"]
    }

    def __init__(self) -> None:
        self._node = rospy.init_node("task_manager_server")
        self._rate = rospy.Rate(200)
        self._sub = rospy.Subscriber(COMMANDS_TOPIC, CommandList, self.commands_callback)

        # Creates an empty dictionary to store the subtask manager of each area
        self.subtask_manager = dict.fromkeys(AREAS, None)
        self.subtask_manager["manipulation"] = TasksManipulation(MANIPULATION_ENABLED)
        self.subtask_manager["nav"] = TasksNav(NAV_ENABLED)
        if VISION_ENABLED:
            self.subtask_manager["vision"] = TasksVision()
        if CONVERSATION_ENABLED:
            self.subtask_manager["hri"] = TasksHRI()
            self.subtask_manager["hri"].speak("Hi, my name is Frida. I'm here to help you with your domestic tasks")

        self.current_state = STATES["IDLE"]
        self.past_state = None
        self.current_command = None
        self.current_queue = []
        self.perceived_information = ""

        self.run()

    def commands_callback(self, commands_input: CommandList) -> None:
        """Receive processed commands from the interpreter and call executions from the queue"""
        rospy.loginfo("Received commands")

        if not commands_input:
            self.current_state = STATES["ERROR"]
            return

        # Check if there are commands pending in the queue and cancel them
        # TODO: Validate with the user if it wants to cancel all the command queue
        if self.current_state != STATES["IDLE"]:
            rospy.logerr("Cancelling current commands and executing new received ")
            self.current_queue = []
            self.cancel_command()
            self.current_state = STATES["IDLE"]

        self.current_state = STATES["COMMANDS_IN_QUEUE"]

        self.subtask_manager["hri"].speak("I've understood your request, I'll execute the following commands:")
        for command in commands_input.commands:
            rospy.loginfo(f"Command: {command.action} -> {command.complement} : {command.characteristic}")
            command_clean =  command.characteristic + " " + command.complement.replace("_", " ")
            if command.action == 'interact':
                self.subtask_manager["hri"].speak(f"I'll interact with you based on this request: {command_clean}", wait=True)
            else:
                self.subtask_manager["hri"].speak(f"I'll {command.action} {command_clean}", wait=True)
        self.subtask_manager["hri"].speak("Let's start")
        
        self.current_queue = commands_input.commands
        self.past_state = self.current_state

    def execute_command(self, command: Command) -> int:
        """Method for executing a single command inside its area submodule"""

        rospy.loginfo(f"Executing command: {command.action} -> {command.complement} : {command.characteristic}")
        command_clean =  command.characteristic + " " + command.complement.replace("_", " ")
        self.subtask_manager["hri"].speak(f"I'll {command.action} {command_clean}", wait=True)

        area_target = ""
        task_result = ""
        for area in AREAS:
            if command.action in TaskManagerServer.COMMANDS_CATEGORY[area]:
                area_target = area
                if not AREA_ENABLED[area]:
                    rospy.logerr(f"Area {area} not enabled")
                    self.subtask_manager["hri"].speak(f"I'm sorry, I can't {command.action} {command_clean}")
                    return STATES["EXECUTION_FAILED"]

        if command.action in ("interact", "ask"):
            task_result = self.subtask_manager["hri"].execute_command(
                command.action, command.complement, self.perceived_information
            )

        elif command.action == "remember":
            self.subtask_manager["nav"].store_current_location()

        elif command.action == "go":
            if self.subtask_manager["nav"].go_place(command.complement) == STATES["EXECUTION_SUCCESS"]:
                self.perceived_information += f"Arrived at {command.complement} "
            else:
                self.subtask_manager["hri"].speak("I couldn't reach the goal")

        elif command.action == "pick":
            if self.subtask_manager["manipulation"].execute_pick(command.complement) == STATES["EXECUTION_SUCCESS"]:
                self.perceived_information += f"Picked {command.complement} "
            else:
                self.subtask_manager["hri"].speak("I'm sorry, I couldn't pick the object")
        
        elif command.action in ("place", "pour"):
            if self.subtask_manager["manipulation"].execute_pick_and_place(command.action) == STATES["EXECUTION_SUCCESS"]:
                self.perceived_information += f"{command.action} {command.complement} "
            else:
                self.subtask_manager["hri"].speak(f"I'm sorry, I couldn't {command.action} the object")

        else:
            self.subtask_manager["hri"].speak("I'm sorry, I can't perform this action")
            
        self.perceived_information += f"{command.action} {command.complement} {task_result} "
        return STATES["EXECUTING_COMMANDS"]

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        for area in AREAS:
            if self.current_command in TaskManagerServer.COMMANDS_CATEGORY[area]:
                self.subtask_manager[area].cancel_command()

    def run(self) -> None:
        """Main loop for the task manager"""
        while not rospy.is_shutdown():

            if len(self.current_queue) > 0: #and self.current_thread == current_thread:
                self.current_command = self.current_queue.pop(0)
                self.status = self.execute_command(self.current_command)
                if self.current_queue == []:
                    self.subtask_manager["hri"].speak("I have finished my tasks, I'm ready to receive a new request")
                    
                if self.status == STATES["ERROR"]:
                    rospy.logerr("Error in command manager")
                    self.current_state = STATES["ERROR"]
                    self.cancel_command()
                    self.current_queue = []
                    self.state = STATES["IDLE"]
                    self.past_state = self.status
                    continue
            elif self.current_state != STATES["IDLE"]:
                self.current_thread = None
                self.current_queue = []
                self.current_state = STATES["IDLE"]
                #self.subtask_manager["hri"].speak("I have finished my tasks, I'm ready to receive a new request")
            self._rate.sleep()


if __name__ == "__main__":
    try:
        TaskManagerServer()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
