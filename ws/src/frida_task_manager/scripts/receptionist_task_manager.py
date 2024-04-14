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
from frida_vision_interfaces.msg import Person, PersonList

### Python submodules
from hri_tasks import TasksHRI
from manipulation_tasks import TasksManipulation
from nav_tasks import TasksNav
from vision_tasks import TasksVision

SPEAK_TOPIC = "/speech/speak"
CONVERSATION_SERVER = "/conversation_as"
FACE_LOCATIONS_TOPIC = "/person_list"

NAV_ENABLED = False
MANIPULATION_ENABLED = True
CONVERSATION_ENABLED = False
VISION_ENABLED = True

AREAS = ["nav", "manipulation", "hri", "vision"]

AREA_ENABLED = {
    "nav": NAV_ENABLED,
    "manipulation": MANIPULATION_ENABLED,
    "hri": CONVERSATION_ENABLED,
    "vision": VISION_ENABLED
}

STATES = {
    "WAITING_GUEST": 0,
    "SELF_INTRODUCTION": 1,
    "REQUEST_GUEST_INFORMATION": 2,
    "STORE_INFORMATION": 3,
    "ERROR": 4,
    "SHUTDOWN": 5
}

class ReceptionistTaskManager:
    """Class to manage different tasks divided by categories"""
    COMMANDS_CATEGORY = {
        "nav" : ["go", "follow", "stop", "approach", "remember"],
        "manipulation" : ["pick", "place", "grasp", "give", "open", "close", "pour"],
        "hri" : ["ask", "interact", "feedback"],
        "vision" : ["find", "identify", "count", "wait"]
    }

    def __init__(self) -> None:
        self._node = rospy.init_node("task_manager_server")
        self._rate = rospy.Rate(200)
        
        self.last_time = rospy.Time.now()
        self.follow_sample_time = 0.5

        # Creates an empty dictionary to store the subtask manager of each area
        self.subtask_manager = dict.fromkeys(AREAS, None)

        if CONVERSATION_ENABLED:
            self.subtask_manager["hri"] = TasksHRI()
            self.subtask_manager["hri"].speak("Hi, my name is Frida. I'm here to help you with your domestic tasks")
        if MANIPULATION_ENABLED:
            self.subtask_manager["manipulation"] = TasksManipulation()
        if NAV_ENABLED:
            self.subtask_manager["nav"] = TasksNav()
        if VISION_ENABLED:
            self.subtask_manager["vision"] = TasksVision()
            rospy.Subscriber(FACE_LOCATIONS_TOPIC, PersonList, self.get_face_locations)

        self.current_state = STATES["WAITING_GUEST"]
        self.current_past_state = None
        self.current_command = None
        self.current_queue = []
        self.perceived_information = ""

        self.detected_faces = []
        self.following_face = True
        self.followed_person = "Unknown"
        self.arm_moving = False

        self.current_queue = []

        self.run()

    def get_face_locations(self, data: PersonList) -> None:
        """Callback to receive the face locations"""
        self.detected_faces = data.list

    def follow_face(self) -> bool:
        """Calls the arm joints server to follow a face
        Returns: Movement of the arm executed"""
        if self.following_face and not self.arm_moving and self.detected_faces:
            rospy.loginfo(f"Following {self.followed_person}")
            self.arm_moving = True
            for face in self.detected_faces:
                if face.name == self.followed_person:
                    self.subtask_manager["manipulation"].move_arm_joints(face.x, face.y)
                    self.arm_moving = False
                    self.detected_faces = []
                    return True
        return False

    def execute_command(self, command: Command) -> int:
        """Method for executing a single command inside its area submodule"""

        rospy.loginfo(f"Executing command: {command.action} -> {command.complement}")

        task_result = 0
        for area in AREAS:
            if command.action in ReceptionistTaskManager.COMMANDS_CATEGORY[area] and AREA_ENABLED[area]:
                task_result = self.subtask_manager[area].execute_command(
                    command.action, command.complement, self.perceived_information
                )

        if task_result == -1:
            rospy.logerr("Error in task execution")
            return STATES["ERROR"]

        self.perceived_information += f"{command.action} {command.complement} {task_result} "
        return STATES["EXECUTING_COMMANDS"]

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        for area in AREAS:
            if self.current_command in ReceptionistTaskManager.COMMANDS_CATEGORY[area]:
                self.subtask_manager[area].cancel_command()

    def run(self) -> None:
        """Main loop for the task manager"""
        while not rospy.is_shutdown():
            dt = rospy.Time.now() - self.last_time # Time from last iteration

            if self.current_state == STATES["WAITING_GUEST"]:
                if dt.to_sec() > self.follow_sample_time:
                    self.last_time = rospy.Time.now()
                    if self.follow_face():
                        self.current_state = STATES["SELF_INTRODUCTION"]

            self._rate.sleep()


if __name__ == "__main__":
    try:
        ReceptionistTaskManager()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
