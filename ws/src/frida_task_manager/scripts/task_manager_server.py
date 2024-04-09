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
from frida_language_processing.msg import Command, CommandList
from frida_language_processing.msg import ConversateAction, ConversateFeedback, ConversateGoal, ConversateResult

### Python submodules
from hri_tasks import TasksHRI
from manipulation_tasks import TasksManipulation

COMMANDS_TOPIC = "/task_manager/commands"
SPEAK_TOPIC = "/speech/speak"
CONVERSATION_SERVER = "/conversation_as"

NAV_ENABLED = False
MANIPULATION_ENABLED = False
CONVERSATION_ENABLED = True

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
        "nav" : ["go", "follow", "stop", "approach"],
        "manipulation" : ["pick", "place", "grasp", "give", "open", "close"],
        "hri" : ["ask", "interact", "feedback"],
        "vision" : ["find", "identify", "count"]
    }

    def __init__(self) -> None:
        self._node = rospy.init_node("task_manager_server")
        self._rate = rospy.Rate(200)
        self._sub = rospy.Subscriber(COMMANDS_TOPIC, CommandList, self.commands_callback)

        if CONVERSATION_ENABLED:
            #asd = self.say_pub = rospy.Publisher('/robot_text', String, queue_size=10)
            self.hri_task_manager = TasksHRI()
        if MANIPULATION_ENABLED:
            self.manipulation_task_manager = TasksManipulation()

        self.current_state = TaskManagerServer.STATE_ENUM["IDLE"]
        self.current_command = None
        self.current_queue = []
        self.current_thread = None
        self.current_location = None
        self.past_location = None
        self.grabbed_object = ""
        self.perceived_information = ""

        #rospy.spin()
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
            rospy.logwarn("Cancelling current commands and executing new received ")
            self.current_queue = []
            self.cancel_command()
            self.current_state = TaskManagerServer.STATE_ENUM["IDLE"]

        current_thread = uuid.uuid1() # Generate a new thread id
        self.current_thread = current_thread

        self.current_state = TaskManagerServer.STATE_ENUM["RECEIVE_COMMANDS"]
        self.current_queue = commands_input.commands
        self.past_state = self.current_state
        self.past_location = self.current_location

        #self.say("I 'have finished my tasks, I'm going to rest now")

    def execute_command(self, command: Command) -> int:
        """Method for executing a single command inside its area submodule"""

        rospy.loginfo(f"Executing command: {command.action} -> {command.complement}")
        if command.action in TaskManagerServer.COMMANDS_CATEGORY["hri"]:
            return self.hri_task_manager.execute_command(command.action, command.complement, self.perceived_information)

        return TaskManagerServer.STATE_ENUM["EXECUTING_COMMANDS"]

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        if self.current_command in TaskManagerServer.COMMANDS_CATEGORY["hri"]:
            self.hri_task_manager.cancel_command()

    def run(self) -> None:
        """Main loop for the task manager"""
        while not rospy.is_shutdown():

            if len(self.current_queue) > 0: #and self.current_thread == current_thread:
                self.current_command = self.current_queue.pop(0)
                self.status = self.execute_command(self.current_command)
                if self.status == TaskManagerServer.STATE_ENUM["ERROR"]:
                    rospy.logerror("Error in command manager")
                    self.current_state = TaskManagerServer.STATE_ENUM["ERROR"]
                    self.cancel_command()
                    self.current_queue = []
                    self.state = TaskManagerServer.STATE_ENUM["IDLE"]
                    self.past_state = self.status
                    break
            else:
                self.current_thread = None
                self.current_queue = []
                self.current_state = TaskManagerServer.STATE_ENUM["IDLE"]
            self._rate.sleep()


if __name__ == "__main__":
    try:
        TaskManagerServer()
    except rospy.ROSInterruptException as e:
        rospy.logerr("Error: {}".format(e))
        pass