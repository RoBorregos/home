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
from frida_hri_interfaces.msg import GuestAnalysisAction, GuestAnalysisFeedback, GuestAnalysisGoal, GuestAnalysisResult
from frida_hri_interfaces.srv import GuestInfo, GuestInfoResponse

SPEAK_TOPIC = "/speech/speak"
SPEAK_NOW_TOPIC = "/speech/speak_now"
CONVERSATION_SERVER = "/conversation_as"
GUEST_INFO_SERVICE = "/guest_info"
GUEST_ANALYSIS_SERVER = "/guest_analysis_as"

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
        self.pub_speak = rospy.Publisher(SPEAK_NOW_TOPIC, String, queue_size=10)

        try:
            rospy.wait_for_service(SPEAK_TOPIC, timeout=5.0)
        except rospy.ROSException:
            rospy.logerr("Speaker service not available")
        self.speak_client = rospy.ServiceProxy(SPEAK_TOPIC, Speak)

        self.guest_description = ["", "", ""]

        rospy.loginfo("HRI Task Manager initialized")

    def execute_command(self, command: str, complement: str, perceived_information: str) -> int:
        """Method to execute each command"""
        rospy.loginfo("HRI Command")
        composed_request = f"{command}: {complement}, perceived info: {perceived_information}"

        goal = ConversateGoal()
        goal.request = composed_request
        goal.wait = 0 if command == "feedback" else 1

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
        rospy.loginfo("Command canceled HRI")

    def get_guest_info(self, guest_id: int):
        """Method to get the guest information
        Returns the name and favorite drink of the guest"""
        rospy.wait_for_service("/guest_info")
        try:
            guest_info = rospy.ServiceProxy(GUEST_INFO_SERVICE, GuestInfo)
            response = guest_info(guest_id)
            if response.success:
                return response.name, response.favorite_drink
            return "error", "error"
        except rospy.ServiceException:
            rospy.logerr("Service call failed")
            return "error", "error"
        
    def analyze_guest(self, guest_id: int) -> str:
        """Method to analyze the guest
        Returns the guest name and favorite drink"""
        client = actionlib.SimpleActionClient(GUEST_ANALYSIS_SERVER, GuestAnalysisAction)
        client.wait_for_server()

        goal = GuestAnalysisGoal()
        goal.guest_id = guest_id
        client.send_goal(
            goal,
            done_cb=self.guest_analysis_done
        )
    
    def guest_analysis_done(self, status, result) -> None:
        """Callback for the guest analysis"""
        rospy.loginfo(f"Guest analysis result: {result.description}")
        self.guest_description[result.guest_id] = result.description
    
    def get_guest_description(self, guest_id: int) -> str:
        """Method to get the guest description stored"""
        return self.guest_description[guest_id]

    def speak(self, text: str, now: bool = False) -> None:
        """Method to publish directly text to the speech node"""
        if now:
            self.pub_speak.publish(text)
        else:
            self.speak_client(text)

if __name__ == "__main__":
    try:
        TasksHRI()
    except rospy.ROSInterruptException as e:
        rospy.logerr("Error: {}".format(e))
        pass