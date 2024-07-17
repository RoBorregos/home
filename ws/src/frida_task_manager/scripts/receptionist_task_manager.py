#!/usr/bin/env python3

"""
Task manager for the Breakfast task of RoboCup @Home 2024
"""

### Import libraries
import rospy
import actionlib
import time
import copy

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

STATES = {
    "WAITING_GUEST": 0,
    "SELF_INTRODUCTION": 1,
    "REQUEST_GUEST_INFORMATION": 2,
    "SAVE_USER_FACE": 3,
    "GO_TO_LIVING_ROOM": 4,
    "INTRODUCE_GUEST_1": 5,
    "INTRODUCE_GUEST_2": 6,
    "GAZE_AT_GUEST": 7,
    "FIND_FREE_SEAT": 8,
    "WAIT_USER_TO_SIT": 9,
    "GO_TO_ENTRANCE": 10,
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
            self.subtask_manager["hri"] = TasksHRI(fake=FAKE_HRI)
        if MANIPULATION_ENABLED:
            self.subtask_manager["manipulation"] = TasksManipulation(fake=FAKE_MANIPULATION)
        if NAV_ENABLED:
            self.subtask_manager["nav"] = TasksNav(fake=FAKE_NAV)
        if VISION_ENABLED:
            self.subtask_manager["vision"] = TasksVision(fake=FAKE_VISION)
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

        self.find_sit_attempts = 0

        self.current_guest = 1

        self.hostname = "Adan"
        self.host_drink = "water"

        self.guests = [
            Guest(0, self.hostname, self.host_drink, ""),
            Guest(1),
            Guest(2),
        ]

        self.host_identified = False

        self.run()

    def get_face_locations(self, data: PersonList) -> None:
        """Callback to receive the face locations"""
        self.detected_faces = data.list

    def follow_face(self) -> bool:
        """Calls the arm joints server to follow a face
        Returns: Movement of the arm executed"""
        if FAKE_VISION:
            return True
        
        if self.following_face and not self.arm_moving and self.detected_faces:
            rospy.loginfo(f"Following {self.followed_person}")
            self.arm_moving = True
            for face in self.detected_faces:
                if face.name == self.followed_person:
                    print("vefore")
                    self.subtask_manager["manipulation"].move_arm_joints(face.x, face.y)
                    print("after")
                    self.arm_moving = False
                    self.detected_faces = []
                    return True
            self.arm_moving = False
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
                rospy.loginfo("Waiting for guest")
                self.followed_person = "Unknown"
                self.subtask_manager["manipulation"].move_arm_joints(0, 0, "face_detection")
                if self.subtask_manager["vision"].check_person():
                    self.follow_face()
                    self.current_state = STATES["SELF_INTRODUCTION"]

            ### Self introduction
            elif self.current_state == STATES["SELF_INTRODUCTION"]:
                rospy.loginfo("Self introduction")
                self.follow_face()
                self.subtask_manager["hri"].speak("Hi, my name is Frida. I'll be your receptionist today. Could you tell me your name and your favorite drink?", now=False)
                self.current_state = STATES["REQUEST_GUEST_INFORMATION"]

            ### Request name and favorite drink and store in current guest object
            elif self.current_state == STATES["REQUEST_GUEST_INFORMATION"]:
                rospy.loginfo("Request guest information")
                #self.subtask_manager["hri"].speak("Could you tell me your name and your favorite drink?", now=False)
                name, drink = self.subtask_manager["hri"].get_guest_info( self.current_guest )
                if name != "error":
                    self.guests[self.current_guest].set_info(name, drink)
                    self.subtask_manager["hri"].speak(f"Nice to meet you {name}, please stay in front of me while I recognize your face.", now=False)
                    self.current_state = STATES["SAVE_USER_FACE"]
                else:
                    self.subtask_manager["hri"].speak("I'm sorry, I didn't get your information.", now=False)

            ### Store user face with its name and call image analysis
            elif self.current_state == STATES["SAVE_USER_FACE"]:
                rospy.loginfo("Save user face")
                if self.follow_face():
                    self.subtask_manager["vision"].analyze_guest( self.current_guest )
                    self.subtask_manager["vision"].save_face_name( self.guests[self.current_guest].name )
                    self.subtask_manager["hri"].speak("I have saved your face, thank you. Please follow me to the living room.", now=False)
                    self.current_state = STATES["GO_TO_LIVING_ROOM"]
                else:
                    self.subtask_manager["hri"].speak("I'm sorry, I couldn't recognize your face. Please stay in front of me.", now=False)

            ### Navigate to the living room
            elif self.current_state == STATES["GO_TO_LIVING_ROOM"]:
                rospy.loginfo("Go to living room")
                #self.subtask_manager["nav"].execute_command("remember", "past location", "")
                self.subtask_manager["hri"].speak("The host is already waiting for you there. Please stay behind me until I find your seat.", now=False)
                self.subtask_manager["manipulation"].move_arm_joints(0, 0, "face_detection")
                #TODO: Face to front default arm position with arm server

                #self.subtask_manager["nav"].execute_command("go", "living_room", "")
                if self.current_guest == 1:
                    self.current_state = STATES["INTRODUCE_GUEST_1"]
                else:
                    self.current_state = STATES["INTRODUCE_GUEST_2"]

            ### Introduce guest 1
            elif self.current_state == STATES["INTRODUCE_GUEST_1"]:
                rospy.loginfo("Introduce guest 1 to host")
                description = self.subtask_manager["vision"].get_guest_description( 1 )
                self.guests[1].set_description( description )

                if not self.host_identified:
                    self.followed_person = "Unknown" #self.guests[0].name

                    self.subtask_manager["manipulation"].move_arm_joints(0, 0, "face_detection")
                    rospy.loginfo("Save host face")
                    while True and not self.host_identified:
                        if not self.follow_face():
                            time.sleep(1)
                            timeout_face += 1
                            rospy.loginfo("Expecting unkown face")
                            continue

                        self.subtask_manager["vision"].analyze_guest(0)
                        self.subtask_manager["vision"].save_face_name( self.guests[0].name )
                        description = self.subtask_manager["vision"].get_guest_description(0)
                        self.guests[0].set_description( description )

                        self.subtask_manager["hri"].speak(f"I have saved {self.guests[0].name}.", now=False)
                        self.host_identified = True

                self.subtask_manager["manipulation"].move_arm_joints(0, 0, "seat")
                timeout_face = 0
                while not self.follow_face() and timeout_face < 10: # Keep following the face until it's recognize IMPROVE
                    time.sleep(1)
                    timeout_face += 1
                    rospy.loginfo("Expecting host face")

                self.subtask_manager["hri"].speak(f"Hi {self.guests[0].name}, this is {self.guests[1].name}. It's favorite drink is {self.guests[1].favorite_drink}", now=False)
                self.subtask_manager["hri"].speak(f"{self.guests[1].description}", now=False)
                self.current_state = STATES["GAZE_AT_GUEST"]

            # Introduce guest 2
            elif self.current_state == STATES["INTRODUCE_GUEST_2"]:
                rospy.loginfo("Introduce guest 2 to host and guest 1")
                description = self.subtask_manager["vision"].get_guest_description( 2 )
                self.guests[2].set_description( description )
                
                introduced_to = 0
                face_locations = ["left_face", "face_detection", "right_face"]

                for location in face_locations:
                    if introduced_to >= 2:
                        break
                    self.subtask_manager["manipulation"].move_arm_joints(0, 0, location)
                    self.detected_faces = []
                    time.sleep(2)
                    current_faces = copy.deepcopy( self.detected_faces )

                    for face in current_faces:
                        self.followed_person = face.name
                        if self.follow_face():
                            introduced_to += 1
                            self.subtask_manager["hri"].speak(f"Hi {face.name}, this is {self.guests[2].name}. It's favorite drink is {self.guests[2].favorite_drink}", now=False) 
                            self.subtask_manager["hri"].speak(f"{self.guests[2].description}", now=False)

                self.current_state = STATES["GAZE_AT_GUEST"]
            
            ### Gaze at the current guest and inform next action
            elif self.current_state == STATES["GAZE_AT_GUEST"]:
                rospy.loginfo("Gaze at guest")
                self.subtask_manager["manipulation"].move_arm_joints(0, 0, "back")
                self.followed_person = self.guests[self.current_guest].name
                timeout_face = 0
                while not self.follow_face() and timeout_face < 10: # Keep following the face until it's recognize IMPROVE
                    time.sleep(1)
                    timeout_face += 1
                    rospy.loginfo("Expecting guest face")
                self.subtask_manager["hri"].speak(f"I'll find you a free seat {self.guests[self.current_guest].name}, please wait.", now=True)

                self.subtask_manager["manipulation"].move_arm_joints(0, 0, "seat")
                self.current_state = STATES["FIND_FREE_SEAT"]

            ### Find a free seat for the guest
            elif self.current_state == STATES["FIND_FREE_SEAT"]:
                rospy.loginfo("Find free seat")
                time.sleep(5)
                seat_angle = self.subtask_manager["vision"].find_seat()
                if seat_angle == 300:
                    self.subtask_manager["hri"].speak("I'm sorry, I couldn't find a free seat for you. Please sit where you prefer", now=True)
                else:
                    #self.subtask_manager["manipulation"].move_arm_joints(0, 0, "seat")
                    self.subtask_manager["hri"].speak("I have found a free seat for you, please follow the direction of my arm.", now=True)
                    self.subtask_manager["manipulation"].move_arm_joints(seat_angle, 20)
                self.current_state = STATES["WAIT_USER_TO_SIT"]

            ### Wait for the user to sit
            elif self.current_state == STATES["WAIT_USER_TO_SIT"]:
                rospy.loginfo("Wait user to sit")
                time.sleep(2)
                timeout_face = 0
                while not self.follow_face() and timeout_face < 10: # Keep following the face until it's recognize IMPROVE
                    time.sleep(1)
                    timeout_face += 1
                    rospy.loginfo("Expecting guest face")

                self.subtask_manager["hri"].speak("I've detected you took your seat. I'll go back to the entrance now.", now=True)
                if self.current_guest < 2:
                    self.current_guest += 1
                    self.current_state = STATES["GO_TO_ENTRANCE"]
                else:
                    self.subtask_manager["hri"].speak("I have finished my task.", now=True)
                    self.current_state = STATES["SHUTDOWN"]

            ### Go back to the entrance
            elif self.current_state == STATES["GO_TO_ENTRANCE"]:
                rospy.loginfo("Go to entrance")
                #self.subtask_manager["nav"].execute_command("go", "entrance", "")
                self.subtask_manager["manipulation"].move_arm_joints(0, 0, "face_detection")
                self.current_state = STATES["WAITING_GUEST"]

            self._rate.sleep()


if __name__ == "__main__":
    try:
        ReceptionistTaskManager()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
