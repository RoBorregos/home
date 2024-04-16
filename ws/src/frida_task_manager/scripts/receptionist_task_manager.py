#!/usr/bin/env python3

"""
Task manager for the Breakfast task of RoboCup @Home 2024
"""

### Import libraries
import rospy
import actionlib
import time

### ROS messages
from std_msgs.msg import String
from frida_vision_interfaces.msg import Person, PersonList
from frida_hri_interfaces.msg import Command, CommandList
from frida_hri_interfaces.msg import GuestAnalysisAction, GuestAnalysisFeedback, GuestAnalysisGoal, GuestAnalysisResult
from frida_hri_interfaces.srv import GuestInfo, GuestInfoResponse

### Python submodules
from hri_tasks import TasksHRI
from manipulation_tasks import TasksManipulation
from nav_tasks import TasksNav
from vision_tasks import TasksVision

SPEAK_TOPIC = "/speech/speak"
CONVERSATION_SERVER = "/conversation_as"
FACE_LOCATIONS_TOPIC = "/person_list"

NAV_ENABLED = True
MANIPULATION_ENABLED = True
CONVERSATION_ENABLED = True
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
    "SAVE_USER_FACE": 3,
    "GO_TO_LIVING_ROOM": 4,
    "INTRODUCE_PEOPLE_TO_GUEST": 5,
    "GAZE_AT_GUEST": 6,
    "FIND_FREE_SEAT": 7,
    "WAIT_USER_TO_SIT": 8,
    "GO_TO_ENTRANCE": 9,
    "ERROR": 20,
    "SHUTDOWN": 100
}

class Guest:
    """Class to store the information of the guest"""
    def __init__(self, guest_id: int = 0, name: str = "", favorite_drink: str = "", description: str = "") -> None:
        self.guest_id = guest_id
        self.name = name
        self.favorite_drink = favorite_drink
        self.description = description
    def set_info(self, name: str, favorite_drink: str) -> None:
        self.name = name
        self.favorite_drink = favorite_drink
    def set_description(self, description: str) -> None:
        self.description = description

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

        self.current_guest = 1

        self.guests = [
            Guest(0, "Adan", "beer", "Is wearing glasses, has dark brown hair, a blue t-shirt and shorts."),
            Guest(1),
            Guest(2),
        ]

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
                self.subtask_manager["manipulation"].move_arm_joints(0, 0, "face_detection")
                if self.subtask_manager["vision"].check_person():
                    self.follow_face()
                    self.current_state = STATES["SELF_INTRODUCTION"]

            ### Self introduction
            elif self.current_state == STATES["SELF_INTRODUCTION"]:
                self.subtask_manager["hri"].speak("Hi, my name is Frida. I'll be your receptionist today.", now=False)
                self.follow_face()
                self.current_state = STATES["REQUEST_GUEST_INFORMATION"]

            ### Request name and favorite drink and store in current guest object
            elif self.current_state == STATES["REQUEST_GUEST_INFORMATION"]:
                self.subtask_manager["hri"].speak("Could you tell me your name and your favorite drink?", now=False)
                name, drink = self.subtask_manager["hri"].get_guest_info( self.current_guest )
                self.guests[self.current_guest].set_info(name, drink)
                if name != "error":
                    self.subtask_manager["hri"].speak(f"Nice to meet you {name}, please stay in front of me while I recognize your face.", now=False)
                    self.current_state = STATES["SAVE_USER_FACE"]
                else:
                    self.subtask_manager["hri"].speak("I'm sorry, I didn't get your information.", now=False)

            ### Store user face with its name and call image analysis
            elif self.current_state == STATES["SAVE_USER_FACE"]:
                if self.follow_face():
                    self.subtask_manager["hri"].analyze_guest( self.current_guest )
                    self.subtask_manager["vision"].save_face_name( self.guests[self.current_guest].name )
                    self.subtask_manager["hri"].speak("I have saved your face, thank you. Please follow me to the living room.", now=False)
                    self.current_state = STATES["GO_TO_LIVING_ROOM"]
                else:
                    self.subtask_manager["hri"].speak("I'm sorry, I couldn't recognize your face. Please stay in front of me.", now=False)

            ### Navigate to the living room
            elif self.current_state == STATES["GO_TO_LIVING_ROOM"]:
                self.subtask_manager["nav"].execute_command("remember", "past location", "")
                self.subtask_manager["hri"].speak("The host is already waiting for you there.", now=True)
                self.subtask_manager["manipulation"].move_arm_joints(0, 0, "face_detection")
                #TODO: Face to front default arm position with arm server
                self.subtask_manager["nav"].execute_command("go", "soccer", "")
                self.current_state = STATES["INTRODUCE_PEOPLE_TO_GUEST"]

            ### Introduce people to the guest
            elif self.current_state == STATES["INTRODUCE_PEOPLE_TO_GUEST"]:
                self.guests[self.current_guest].set_description( self.subtask_manager["hri"].get_guest_description( self.current_guest ) )
                for g_id in range(len(self.current_guest)): # Check all guests before the current one
                    self.followed_person = self.guests[g_id].name
                    while not self.follow_face(): # Keep following the face until it's recognize IMPROVE
                        pass
                    self.subtask_manager["hri"].speak(f"This is {self.guests[g_id].name}.", now=False)
                    self.subtask_manager["hri"].speak(f"It's favorite drink is {self.guests[g_id].favorite_drink}.", now=False)
                    #TODO: Extract interpreted info
                    self.subtask_manager["hri"].speak(f"{self.guests[g_id].description}", now=False)
                self.current_state = STATES["GAZE_AT_GUEST"]

            ### Gaze at the current guest and inform next action
            elif self.current_state == STATES["GAZE_AT_GUEST"]:
                self.subtask_manager["manipulation"].move_arm_joints(0, 0, "back")
                self.followed_person = self.guests[self.current_guest].name
                while not self.follow_face():
                    pass
                self.subtask_manager["hri"].speak("I'll find you a free seat, please wait.", now=False)
                self.subtask_manager["manipulation"].move_arm_joints(0, 0, "face_detection")
                self.current_state = STATES["FIND_FREE_SEAT"]

            ### Find a free seat for the guest
            elif self.current_state == STATES["FIND_FREE_SEAT"]:
                seat_angle = self.subtask_manager["vision"].find_seat( self.current_guest )
                self.subtask_manager["manipulation"].move_arm_joints(0, 0, "seat")
                self.subtask_manager["manipulation"].move_arm_joints(seat_angle, 0)
                self.subtask_manager["hri"].speak("I have found a free seat for you, please follow the direction of my arm.", now=False)
                self.current_state = STATES["WAIT_USER_TO_SIT"]

            ### Wait for the user to sit
            elif self.current_state == STATES["WAIT_USER_TO_SIT"]:
                time.sleep(2)
                while not self.follow_face():
                    pass
                self.subtask_manager["hri"].speak("I've detected you took your seat. I'll go back to the entrance now.", now=False)
                if self.current_guest < len(self.guests):
                    self.current_guest += 1
                    self.current_state = STATES["GO_TO_ENTRANCE"]
                else:
                    self.current_state = STATES["SHUTDOWN"] 

            ### Go back to the entrance
            elif self.current_state == STATES["GO_TO_ENTRANCE"]:
                self.subtask_manager["nav"].execute_command("go", "past location", "")
                self.subtask_manager["manipulation"].move_arm_joints(0, 0, "face_detection")
                self.current_state = STATES["WAITING_GUEST"]

            self._rate.sleep()


if __name__ == "__main__":
    try:
        ReceptionistTaskManager()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
