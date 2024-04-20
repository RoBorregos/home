#!/usr/bin/env python3

"""
This script manages the implementation of each HRI tasks
"""

### Import libraries
import rospy
import actionlib

### ROS messages
from std_msgs.msg import String
from frida_hri_interfaces.msg import ConversateAction, ConversateFeedback, ConversateGoal, ConversateResult
from frida_hri_interfaces.srv import Speak
from frida_hri_interfaces.srv import ItemsCategory

SPEAK_TOPIC = "/speech/speak"
CONVERSATION_SERVER = "/conversation_as"

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

    def __init__(self, fake = False) -> None:
        self.FAKE_TASK = fake
        if not self.FAKE_TASK:
            # self.conversation_client = actionlib.SimpleActionClient(CONVERSATION_SERVER, ConversateAction)
            #self.pub_speak = rospy.Publisher(SPEAK_TOPIC, String, queue_size=10)
            rospy.loginfo("[INFO] Waiting for conversation server")
            rospy.wait_for_service(SPEAK_TOPIC, timeout=10.0)
            self.speak_client = rospy.ServiceProxy(SPEAK_TOPIC, Speak)
            # rospy.loginfo("[SUCCESS] Conversation server initialized")
            # rospy.loginfo("[INFO] Waiting for items category server")
            # rospy.wait_for_service("/items_category")
            # self.category_client = rospy.ServiceProxy("/items_category", ItemsCategory)
        else:
            rospy.loginfo("[INFO] Fake HRI Task Manager initialized")

        rospy.loginfo("[SUCCESS] HRI Task Manager initialized")

    def execute_command(self, command: str, complement: str, perceived_information: str) -> int:
        """Method to execute each command"""
        rospy.loginfo("[INFO] HRI Command")
        composed_request = f"{command}: {complement}, perceived info: {perceived_information}"

        goal = ConversateGoal()
        goal.request = composed_request
        goal.wait = 0 if command == "feedback" else 1

        if not self.FAKE_TASK:
            self.conversation_client.send_goal(goal)
            if goal.wait:
                self.conversation_client.wait_for_result()
                result = self.conversation_client.get_result()
                #rospy.loginfo(f"Result: {result.success}")
                return result.success
        return 1
        
    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        self.conversation_client.cancel_all_goals()
        rospy.loginfo("[INFO] Command canceled HRI")

    def speak(self, text: str) -> None:
        """Method to publish directly text to the speech node"""
        #self.pub_speak.publish(text)
        if not self.FAKE_TASK:
            pass
            self.speak_client(text)
        else:
            rospy.loginfo(f"[INFO] Speaking: {text}")
    
    def get_objects_category(self, object_names) -> str:
        """Method to get the category of a list of object names"""
        if self.FAKE_TASK:
            return "sample_category2"
        response = self.category_client(items=object_names)
        return response.category

if __name__ == "__main__":
    try:
        TasksHRI()
    except rospy.ROSInterruptException as e:
        rospy.logerr("Error: {}".format(e))
        pass