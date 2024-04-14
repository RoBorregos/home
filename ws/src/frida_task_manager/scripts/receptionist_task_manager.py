#!/usr/bin/env python3

"""
Task manager for the Breakfast task of RoboCup @Home 2024
"""

### Import libraries
import rospy
import actionlib

### ROS messages
from std_msgs.msg import String
from frida_hri_interfaces.msg import Command, CommandList
from frida_hri_interfaces.msg import ConversateAction, ConversateFeedback, ConversateGoal, ConversateResult

### Python submodules
from hri_tasks import TasksHRI
from manipulation_tasks import TasksManipulation
from nav_tasks import TasksNav

SPEAK_TOPIC = "/speech/speak"
CONVERSATION_SERVER = "/conversation_as"
FACE_LOCATIONS_TOPIC = "/

NAV_ENABLED = True
MANIPULATION_ENABLED = True
CONVERSATION_ENABLED = False
VISION_ENABLED = False

AREAS = ["nav", "manipulation", "hri", "vision"]

AREA_ENABLED = {
    "nav": NAV_ENABLED,
    "manipulation": MANIPULATION_ENABLED,
    "hri": CONVERSATION_ENABLED,
    "vision": VISION_ENABLED
}

class TaskManagerServer:
    """Class to manage different tasks divided by categories"""
    STATE_ENUM = {
        "IDLE": 0,
        "RECEIVE_COMMANDS": 1,
        "EXECUTING_COMMANDS": 2,
        "STOPPING": 3,
        "ERROR": 4,
        "SHUTDOWN": 5
    }

    COMMANDS_CATEGORY = {
        "nav" : ["go", "follow", "stop", "approach", "remember"],
        "manipulation" : ["pick", "place", "grasp", "give", "open", "close", "pour"],
        "hri" : ["ask", "interact", "feedback"],
        "vision" : ["find", "identify", "count", "wait"]
    }

    def __init__(self) -> None:
        self._node = rospy.init_node("task_manager_server")
        self._rate = rospy.Rate(200)
        self._sub = rospy.Subscriber(COMMANDS_TOPIC, CommandList, self.commands_callback)

        # Creates an empty dictionary to store the subtask manager of each area
        self.subtask_manager = dict.fromkeys(AREAS, None)

        if CONVERSATION_ENABLED:
            self.subtask_manager["hri"] = TasksHRI()
            self.subtask_manager["hri"].speak("Hi, my name is Frida. I'm here to help you with your domestic tasks")
        if MANIPULATION_ENABLED:
            self.subtask_manager["manipulation"] = TasksManipulation()
        if NAV_ENABLED:
            self.subtask_manager["nav"] = TasksNav()
        #if VISION_ENABLED:
            #self.subtask_manager["vision"] = TasksVision()

        self.current_state = TaskManagerServer.STATE_ENUM["IDLE"]
        self.current_past_state = None
        self.current_command = None
        self.current_queue = []
        self.perceived_information = ""

        self.current_queue = [
            Command(action="wait", complement="person"),
            Command(action="analyze", complement="person"),
            Command(action="interact", complement="introduce yourself"),
            Command(action="ask", complement="user name and favorite drink"),
            Command(action="save", complement="name"),
            Command(action="interact", complement="ask to be followed to the living room"),
            Command(action="go", complement="living room"),
            Command(action="look", complement="seat"),
            Command(action="interact", complement="introduce host"),
            Command(action="look", complement="guest 1"),
            Command(action="interact", complement="announce a place to sit will be offered"),
            Command(action="look", complement="seat"),
            Command(action="find", complement="free seat"),
            Command(action="look", complement="free seat"),
            Command(action="interact", complement="indicate the user can sit in the direction is gazing"),
            Command(action="interact", complement="announce robot will return to entrance"),
            Command(action="go", complement="entrance"),

            Command(action="wait", complement="person"),
            Command(action="analyze", complement="person"),
            Command(action="interact", complement="introduce yourself"),
            Command(action="ask", complement="user name and favorite drink"),
            Command(action="save", complement="name"),
            Command(action="interact", complement="ask to be followed to the living room"),
            Command(action="go", complement="living room"),
            Command(action="look", complement="seat"),
            Command(action="interact", complement="introduce host"),
            Command(action="interact", complement="introduce guest 1"),
            Command(action="look", complement="guest 2"),
            Command(action="interact", complement="announce a place to sit will be offered"),
            Command(action="look", complement="seat"),
            Command(action="find", complement="free seat"),
            Command(action="look", complement="free seat"),
            Command(action="interact", complement="indicate the user can sit in the direction is gazing")
        ]

        self.run()

    def commands_callback(self, commands_input: CommandList) -> None:
        """Receive processed commands from the interpreter and call executions from the queue"""
        rospy.loginfo("Received commands")

        if not commands_input:
            self.current_state = TaskManagerServer.STATE_ENUM["ERROR"]
            return

        if self.current_thread is not None:
            return

        if self.current_state != TaskManagerServer.STATE_ENUM["IDLE"]:
            rospy.logerr("Cancelling current commands and executing new received ")
            self.current_queue = []
            self.cancel_command()
            self.current_state = TaskManagerServer.STATE_ENUM["IDLE"]

        self.current_state = TaskManagerServer.STATE_ENUM["RECEIVE_COMMANDS"]
        self.current_queue = commands_input.commands
        self.past_state = self.current_state

        #self.say("I 'have finished my tasks, I'm going to rest now")

    def execute_command(self, command: Command) -> int:
        """Method for executing a single command inside its area submodule"""

        rospy.loginfo(f"Executing command: {command.action} -> {command.complement}")

        task_result = 0
        for area in AREAS:
            if command.action in TaskManagerServer.COMMANDS_CATEGORY[area] and AREA_ENABLED[area]:
                task_result = self.subtask_manager[area].execute_command(
                    command.action, command.complement, self.perceived_information
                )

        if task_result == -1:
            rospy.logerr("Error in task execution")
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

            if len(self.current_queue) > 0: #and self.current_thread == current_thread:
                self.current_command = self.current_queue.pop(0)
                self.status = self.execute_command(self.current_command)
                if self.status == TaskManagerServer.STATE_ENUM["ERROR"]:
                    rospy.logerr("Error in command manager")
                    self.current_state = TaskManagerServer.STATE_ENUM["ERROR"]
                    self.cancel_command()
                    self.current_queue = []
                    self.state = TaskManagerServer.STATE_ENUM["IDLE"]
                    self.past_state = self.status
                    continue
            elif self.current_state != TaskManagerServer.STATE_ENUM["IDLE"]:
                self.current_thread = None
                self.current_queue = []
                self.current_state = TaskManagerServer.STATE_ENUM["IDLE"]
                
                #self.subtask_manager["hri"].speak("I have finished my tasks, I'm going to rest now")
            self._rate.sleep()


if __name__ == "__main__":
    try:
        TaskManagerServer()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
