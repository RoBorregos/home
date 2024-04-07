#!/usr/bin/env python3

"""
This script manages the implementation of each HRI tasks
"""

### Import libraries
import rospy
import actionlib

### ROS messages
from std_msgs.msg import String
from frida_language_processing.msg import Command, CommandList
from frida_language_processing.msg import ConversateAction, ConversateFeedback, ConversateGoal, ConversateResult

COMMANDS_TOPIC = "/task_manager/commands"
SPEAK_TOPIC = "/speech/speak"
CONVERSATION_SERVER = "/conversation_as"

NAV_ENABLED = False
MANIPULATION_ENABLED = False
CONVERSATION_ENABLED = True

class TasksHRI:
    STATE_ENUM = {
        "IDLE": 0,
        "RECEIVE_COMMANDS": 1,
        "EXECUTING_COMMANDS": 2,
        "STOPPING": 3,
        "ERROR": 4,
        "SHUTDOWN": 5
    }

    AREA_TASKS = ["ask", "interact", "feedback"]

    def __init__(self) -> None:
        self.conversation_client = actionlib.SimpleActionClient(CONVERSATION_SERVER, ConversateAction)
        self.pub_speak = rospy.Publisher(SPEAK_TOPIC, String, queue_size=10)
        rospy.loginfo("HRI Task Manager initialized")

    def execute_command(self, command: str, complement: str, perceived_information: str) -> int:
        """Method to execute each command"""
        rospy.loginfo("HRI Command")
        composed_request = f"{command}: {complement}, perceived information: {perceived_information}"

        goal = ConversateGoal()
        goal.request = composed_request
        goal.wait = 0 if command == "feedback" else 1

        self.conversation_client.send_goal(goal)
        if goal.wait:
            self.conversation_client.wait_for_result()
            result = self.conversation_client.get_result()
            rospy.loginfo(f"Result: {result.success}")
            return result.success
        
    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        self.conversation_client.cancel_all_goals()
        rospy.loginfo("Command canceled HRI")

    def speak(self, text: str) -> None:
        """Method to publish directly text to the speech node"""
        self.pub_speak.publish(text)

if __name__ == "__main__":
    try:
        TaskManagerServer()
    except rospy.ROSInterruptException as e:
        rospy.logerr("Error: {}".format(e))
        pass